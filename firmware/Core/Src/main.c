/**
 * main.c - STM32C031 DDC/CI monitor-switch keypad.
 *
 * Board:   NUCLEO-C031C6 (STM32C031C6, Cortex-M0+)
 * Drivers: STM32Cube HAL
 *
 * Six buttons choose which computer (PC / Mac) each monitor shows. A press
 * becomes a "switch job": the firmware sends DDC/CI Set VCP 0x60 (input
 * source) to the monitor over I2C, reads the value back with Get VCP and
 * retries on failure. Logs go to USART2 (115200 8N1).
 *
 * Interrupt / execution model
 *   EXTI ISR      button edge -> keypad_on_edge() (masks that line)
 *   SysTick 1 ms  keypad_tick_1ms(): debounce, push events to a ring buffer
 *   main loop     pop events -> switcher_request(); switcher_poll()
 *                 (non-blocking state machine); then __WFI() until next IRQ
 *
 * Note on the vector table: Wokwi's "STM32 HAL" template links against the
 * STM32duino core, which already defines EXTIx_IRQHandler (and the HAL EXTI
 * callbacks) for attachInterrupt(). Instead of fighting the linker, we copy
 * the vector table into RAM, point SCB->VTOR at the copy and install our own
 * register-level EXTI handler. The same code works on a bare CubeIDE project.
 */
#include <stdio.h>
#include <string.h>
#include "stm32c0xx_hal.h"

#include "config.h"
#include "ddc.h"
#include "keypad.h"
#include "switcher.h"
#include "sw_i2c.h"

#define LOG_I2C_BYTES  1      /* 1 = hex-dump every DDC/CI transfer */

/* Sleep between interrupts with __WFI. Keep 0 in Wokwi: its STM32C0 model
 * does not wake the core from WFI on SysTick (verified - the main loop
 * stalls), so events are never processed. Set 1 on real hardware. */
#define USE_WFI        0

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} gpio_t;

static const gpio_t BTN_PINS[]    = { BTN_PINS_DEF };

typedef struct {
    GPIO_TypeDef *port;
    uint8_t       scl;     /* pin numbers 0-15 */
    uint8_t       sda;
} ddc_pins_t;

static const ddc_pins_t DDC_PINS[NUM_MONITORS] = { DDC_BUS_PINS_DEF };
static UART_HandleTypeDef huart2;

static volatile uint32_t s_systick_hook_count;
static volatile uint8_t  s_tick_in_isr;     /* 1 = keypad ticked from SysTick  */
static uint8_t           s_prev_pressed[NUM_BUTTONS];

/* Which path first saw each press: lets us verify on the serial log that the
 * EXTI interrupt really fires (the 1 ms sampler is only a safety net). */
#define EDGE_SRC_EXTI  1u
#define EDGE_SRC_POLL  2u
static volatile uint8_t  s_edge_src[NUM_BUTTONS];
static volatile uint32_t s_exti_irq_count;
static uint32_t          s_btn_line_mask;    /* EXTI lines used by buttons */
static uint32_t          s_led_off_at;
static uint8_t           s_led_active;

static void Error_Handler(void);

/* ======================================================================
 *  printf -> USART2
 * ==================================================================== */
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* ======================================================================
 *  Peripheral init
 * ==================================================================== */
static void uart2_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;          /* PA2 TX, PA3 RX */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = 115200;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    setvbuf(stdout, NULL, _IONBF, 0);               /* print immediately */
}

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LEDs, all off */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_BUSY_PIN; HAL_GPIO_Init(LED_BUSY_PORT, &g);
    g.Pin = LED_OK_PIN;   HAL_GPIO_Init(LED_OK_PORT, &g);
    g.Pin = LED_ERR_PIN;  HAL_GPIO_Init(LED_ERR_PORT, &g);

    /* Buttons: pull-up, interrupt on falling edge (press = short to GND).
     * HAL_GPIO_Init also routes the pin to its EXTI line (EXTICR). */
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    for (unsigned i = 0; i < NUM_BUTTONS; i++) {
        g.Pin = BTN_PINS[i].pin;
        HAL_GPIO_Init(BTN_PINS[i].port, &g);
        s_btn_line_mask |= BTN_PINS[i].pin;
    }

    /* EXTI and SysTick share keypad state, so give them the SAME priority:
     * then neither can pre-empt the other and no locking is needed. */
    HAL_NVIC_SetPriority(SysTick_IRQn,   3, 0);
    HAL_NVIC_SetPriority(EXTI0_1_IRQn,   3, 0);
    HAL_NVIC_SetPriority(EXTI2_3_IRQn,   3, 0);
    HAL_NVIC_SetPriority(EXTI4_15_IRQn,  3, 0);
    /* NVIC lines are enabled in main() once our handlers are installed. */
}

/* ---- DDC buses: one software I2C bus per monitor ----
 * Every monitor answers at 0x37, so each needs its own bus. The STM32C031
 * has a single I2C peripheral (which also stalls with custom chips in
 * Wokwi), so both buses are bit-banged on plain GPIOs, set in config.h.
 *
 * Pseudo open-drain: ODR is held at 0 and a line is "pulled low" by making
 * the pin an output, or "released" by making it an input so the pull-up
 * raises it. Unlike GPIO_MODE_OUTPUT_OD this behaves identically on real
 * silicon and in simulators that don't model open-drain outputs. */
static void line_write(GPIO_TypeDef *port, uint32_t pin, bool high)
{
    uint32_t moder = port->MODER & ~(3u << (pin * 2u));
    port->MODER = high ? moder : (moder | (1u << (pin * 2u)));   /* 00 in, 01 out */
}
static bool line_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1u;
}
static void ddc_scl_write(void *ctx, bool high)
{
    const ddc_pins_t *p = ctx;
    line_write(p->port, p->scl, high);
}
static void ddc_sda_write(void *ctx, bool high)
{
    const ddc_pins_t *p = ctx;
    line_write(p->port, p->sda, high);
}
static bool ddc_scl_read(void *ctx)
{
    const ddc_pins_t *p = ctx;
    return line_read(p->port, p->scl);
}
static bool ddc_sda_read(void *ctx)
{
    const ddc_pins_t *p = ctx;
    return line_read(p->port, p->sda);
}

/* Busy-wait using the SysTick down-counter (no extra timer needed). */
static void delay_us(uint32_t us)
{
    uint32_t load = SysTick->LOAD + 1u;
    uint32_t need = us * (SystemCoreClock / 1000000u);
    uint32_t prev = SysTick->VAL, elapsed = 0;
    while (elapsed < need) {
        uint32_t now = SysTick->VAL;
        elapsed += (prev >= now) ? (prev - now) : (prev + load - now);
        prev = now;
    }
}

static sw_i2c_t s_ddc_bus[NUM_MONITORS];
static const sw_i2c_t *s_active_bus;       /* NULL = no monitor selected */

static void ddc_buses_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    for (uint8_t m = 0; m < NUM_MONITORS; m++) {
        const ddc_pins_t *p = &DDC_PINS[m];
        g.Pin   = (uint16_t)((1u << p->scl) | (1u << p->sda));
        g.Mode  = GPIO_MODE_INPUT;           /* both lines released */
        g.Pull  = GPIO_PULLUP;               /* real HW: add 4.7k external */
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(p->port, &g);
        p->port->BSRR = (uint32_t)g.Pin << 16;   /* ODR = 0 */

        s_ddc_bus[m] = (sw_i2c_t){
            .ctx                = (void *)p,
            .scl_write          = ddc_scl_write,
            .sda_write          = ddc_sda_write,
            .scl_read           = ddc_scl_read,
            .sda_read           = ddc_sda_read,
            .delay_us           = delay_us,
            .half_period_us     = 5u,        /* ~100 kHz, the DDC/CI maximum */
            .stretch_timeout_us = 10000u,
        };
        sw_i2c_recover(&s_ddc_bus[m]);
    }
}

/* ======================================================================
 *  Interrupt handlers (installed into a RAM vector table)
 * ==================================================================== */
#define NUM_VECTORS  48u    /* 16 Cortex-M0+ exceptions + 32 STM32C0 IRQs */

/* VTOR requires the table to be aligned to the next power of two >= its
 * size: 48 * 4 = 192 bytes -> 256-byte alignment. */
static uint32_t s_ram_vectors[NUM_VECTORS] __attribute__((aligned(256)));

/* One handler serves EXTI0_1, EXTI2_3 and EXTI4_15: read which enabled
 * lines saw a falling edge, acknowledge them (write-1-to-clear), dispatch. */
static void buttons_exti_isr(void)
{
    uint32_t pending = EXTI->FPR1 & EXTI->IMR1 & s_btn_line_mask;
    EXTI->FPR1 = pending;
    s_exti_irq_count++;
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        if ((pending & BTN_PINS[i].pin) && keypad_on_edge(i)) {  /* line n == pin n */
            s_edge_src[i] = EDGE_SRC_EXTI;
        }
    }
}

/* Returns true if the CPU accepted the relocated table. */
static bool install_ram_vectors(void)
{
    const uint32_t *active = (const uint32_t *)SCB->VTOR;   /* usually flash */

    for (uint32_t i = 0; i < NUM_VECTORS; i++) {
        s_ram_vectors[i] = active[i];
    }
    s_ram_vectors[16u + EXTI0_1_IRQn]  = (uint32_t)buttons_exti_isr;
    s_ram_vectors[16u + EXTI2_3_IRQn]  = (uint32_t)buttons_exti_isr;
    s_ram_vectors[16u + EXTI4_15_IRQn] = (uint32_t)buttons_exti_isr;

    __disable_irq();
    SCB->VTOR = (uint32_t)s_ram_vectors;
    __DSB();
    __ISB();
    __enable_irq();

    return SCB->VTOR == (uint32_t)s_ram_vectors;
}

/* The STM32duino core under Wokwi's template calls osSystickHandler() from
 * its SysTick_Handler every 1 ms (after HAL_IncTick). We use it as the debounce
 * timebase. If the hook is never called, main() falls back to ticking the
 * keypad from the main loop (see tick_selftest()). */
static void keypad_tick(void)
{
    /* Safety net: also sample the pins every 1 ms. If the EXTI ISR already
     * caught the edge, keypad_on_edge() returns false and nothing happens;
     * if the interrupt was missed (e.g. a simulator that ignores VTOR), the
     * press is still detected - and the log says which path caught it. */
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        uint8_t now = HAL_GPIO_ReadPin(BTN_PINS[i].port, BTN_PINS[i].pin) == GPIO_PIN_RESET;
        if (now && !s_prev_pressed[i] && keypad_on_edge(i)) {
            s_edge_src[i] = EDGE_SRC_POLL;
        }
        s_prev_pressed[i] = now;
    }
    keypad_tick_1ms();
}

void osSystickHandler(void)
{
    s_systick_hook_count++;
    if (s_tick_in_isr) {
        keypad_tick();
    }
}

/* ======================================================================
 *  keypad hardware port
 * ==================================================================== */
static bool kp_is_pressed(uint8_t idx)
{
    return HAL_GPIO_ReadPin(BTN_PINS[idx].port, BTN_PINS[idx].pin) == GPIO_PIN_RESET;
}

static void kp_set_irq(uint8_t idx, bool enable)
{
    uint32_t mask = BTN_PINS[idx].pin;      /* EXTI line n == pin number n */
    if (enable) {
        EXTI->FPR1 = mask;                  /* drop edges latched while masked */
        EXTI->RPR1 = mask;
        EXTI->IMR1 |= mask;
    } else {
        EXTI->IMR1 &= ~mask;
    }
}

static const keypad_hw_t KEYPAD_HW = {
    .is_pressed = kp_is_pressed,
    .set_irq    = kp_set_irq,
};

/* ======================================================================
 *  switcher (DDC/CI transport) port
 * ==================================================================== */
#if LOG_I2C_BYTES
static void log_bytes(const char *dir, const uint8_t *b, size_t n, bool ack)
{
    printf("    %s", dir);
    for (size_t i = 0; i < n; i++) {
        printf(" %02X", b[i]);
    }
    printf("%s\r\n", ack ? "" : "  <NACK>");
}
#endif

/* Each monitor is on its own bus, so "selecting" one just picks which
 * bus the next transfers use. Nothing is switched in hardware. */
static void port_select_bus(void *ctx, uint8_t monitor)
{
    (void)ctx;
    s_active_bus = (monitor < NUM_MONITORS) ? &s_ddc_bus[monitor] : NULL;
}

static bool port_i2c_write(void *ctx, const uint8_t *buf, size_t len)
{
    (void)ctx;
    bool ok = s_active_bus && sw_i2c_write(s_active_bus, DDC_I2C_ADDR_7BIT, buf, len);
#if LOG_I2C_BYTES
    log_bytes("TX 6E:", buf, len, ok);
#endif
    return ok;
}

static bool port_i2c_read(void *ctx, uint8_t *buf, size_t len)
{
    (void)ctx;
    bool ok = s_active_bus && sw_i2c_read(s_active_bus, DDC_I2C_ADDR_7BIT, buf, len);
#if LOG_I2C_BYTES
    log_bytes("RX 6F:", buf, ok ? len : 0, ok);
#endif
    return ok;
}

static uint32_t port_now_ms(void *ctx)
{
    (void)ctx;
    return HAL_GetTick();
}

static void show_result_led(bool ok)
{
    HAL_GPIO_WritePin(LED_OK_PORT,  LED_OK_PIN,  ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ERR_PORT, LED_ERR_PIN, ok ? GPIO_PIN_RESET : GPIO_PIN_SET);
    s_led_off_at = HAL_GetTick() + RESULT_LED_MS;
    s_led_active = 1;
}

static void port_on_result(void *ctx, const switch_result_t *r)
{
    (void)ctx;
    if (r->status == SW_OK) {
        printf("[OK]   %s monitor -> %s (0x%02X) in %u ms, %u attempt(s)\r\n",
               MONITOR_NAMES[r->monitor], ddc_input_name(r->requested),
               r->requested, (unsigned)r->elapsed_ms, r->attempts);
    } else {
        printf("[FAIL] %s monitor -> %s: %s",
               MONITOR_NAMES[r->monitor], ddc_input_name(r->requested),
               switcher_status_str(r->status));
        if (r->status == SW_ERR_PROTOCOL) {
            printf(" (%s)", ddc_status_str(r->ddc_err));
        } else if (r->status == SW_ERR_MISMATCH) {
            printf(" (monitor reports 0x%02X)", r->actual);
        }
        printf(" after %u attempt(s)\r\n", r->attempts);
    }
    show_result_led(r->status == SW_OK);
}

static const switcher_port_t SWITCHER_PORT = {
    .ctx        = NULL,
    .select_bus = port_select_bus,
    .i2c_write  = port_i2c_write,
    .i2c_read   = port_i2c_read,
    .now_ms     = port_now_ms,
    .on_result  = port_on_result,
};

/* ======================================================================
 *  Application
 * ==================================================================== */
static void handle_button(uint8_t idx)
{
    const button_action_t *a = &BUTTON_ACTIONS[idx];
    printf("\r\n> Button: %s   [edge seen by %s, EXTI IRQs so far: %lu]\r\n", a->label,
           s_edge_src[idx] == EDGE_SRC_EXTI ? "EXTI interrupt" : "1 ms sampler",
           (unsigned long)s_exti_irq_count);

    for (uint8_t m = 0; m < NUM_MONITORS; m++) {
        if (a->monitor != ALL_MONITORS && a->monitor != m) {
            continue;
        }
        uint16_t code = INPUT_MAP[m][a->target];
        if (!switcher_request(m, code)) {
            printf("[FAIL] job queue full, dropped %s\r\n", MONITOR_NAMES[m]);
        }
    }
}

/* Decide where the 1 ms debounce tick runs: SysTick hook if available,
 * otherwise the main loop. */
static void tick_selftest(void)
{
    uint32_t before = s_systick_hook_count;
    uint32_t t0 = HAL_GetTick();
    uint32_t guard = 0;
    while (HAL_GetTick() - t0 < 5u && ++guard < 2000000u) { }

    if (guard >= 2000000u) {
        printf("WARNING: HAL tick is not running (SysTick)\r\n");
    }
    s_tick_in_isr = (s_systick_hook_count != before) ? 1u : 0u;
    printf("Debounce tick: %s\r\n", s_tick_in_isr ? "SysTick ISR" : "main loop (fallback)");
}

static void print_banner(void)
{
    printf("\r\n=== STM32 DDC/CI Monitor Switch ===\r\n");
    printf("Core clock %lu MHz, DDC: one software I2C bus per monitor\r\n",
           (unsigned long)(SystemCoreClock / 1000000u));
    for (uint8_t m = 0; m < NUM_MONITORS; m++) {
        printf("%-5s monitor:", MONITOR_NAMES[m]);
        for (uint8_t t = 0; t < NUM_TARGETS; t++) {
            printf("  %s=%s (0x%02X)", TARGET_NAMES[t],
                   ddc_input_name(INPUT_MAP[m][t]), INPUT_MAP[m][t]);
        }
        printf("\r\n");
    }
}

int main(void)
{
    HAL_Init();
    /* The clock may already have been raised (Wokwi's template / STM32duino
     * sets 48 MHz before main) while the SystemCoreClock variable still says
     * 12 MHz, so HAL_Init() programs SysTick 4x too fast and every "ms" is
     * 0.25 ms. Re-read the real clock and re-arm the 1 ms tick. */
    SystemCoreClockUpdate();
    HAL_InitTick(TICK_INT_PRIORITY);

    uart2_init();
    gpio_init();
    ddc_buses_init();

    print_banner();
    tick_selftest();

    if (install_ram_vectors()) {
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
        printf("Buttons: EXTI interrupts (vector table in RAM @ 0x%08lX) + 1 ms sampler backup\r\n",
               (unsigned long)SCB->VTOR);
    } else {
        printf("Buttons: VTOR not writable, using the 1 ms sampler only\r\n");
    }

    keypad_init(&KEYPAD_HW, (uint8_t)NUM_BUTTONS);
    switcher_init(&SWITCHER_PORT);
#if USE_WFI
    printf("Sleep test (WFI)... ");
    uint32_t t_wfi = HAL_GetTick();
    __WFI();                                   /* SysTick should wake us within 1 ms */
    printf("ok (%lu ms)\r\n", (unsigned long)(HAL_GetTick() - t_wfi));
#endif
    printf("Ready. Press a button.\r\n");

    uint32_t last_tick = HAL_GetTick();

    while (1) {
        /* Fallback debounce timebase when the SysTick hook isn't present. */
        if (!s_tick_in_isr) {
            uint32_t now = HAL_GetTick();
            while (last_tick != now) {
                last_tick++;
                __disable_irq();          /* keypad state is shared with EXTI */
                keypad_tick();
                __enable_irq();
            }
        }

        uint8_t btn;
        while (keypad_pop(&btn)) {
            handle_button(btn);
        }

        switcher_poll();

        HAL_GPIO_WritePin(LED_BUSY_PORT, LED_BUSY_PIN,
                          switcher_busy() ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (s_led_active && (int32_t)(HAL_GetTick() - s_led_off_at) >= 0) {
            HAL_GPIO_WritePin(LED_OK_PORT,  LED_OK_PIN,  GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_ERR_PORT, LED_ERR_PIN, GPIO_PIN_RESET);
            s_led_active = 0;
        }

#if USE_WFI
        __WFI();    /* sleep until the next interrupt (SysTick wakes us every 1 ms) */
#endif
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
