/**
 * keypad.c - see keypad.h
 */
#include "keypad.h"
#include <stddef.h>

typedef enum {
    BTN_IDLE = 0,        /* IRQ armed, waiting for an edge          */
    BTN_DEBOUNCE,        /* edge seen, IRQ masked, waiting to sample */
    BTN_HELD             /* press reported, waiting for release      */
} btn_state_t;

typedef struct {
    btn_state_t state;
    uint8_t     timer_ms;
} btn_t;

static const keypad_hw_t *s_hw;
static uint8_t            s_num;
static btn_t              s_btn[KEYPAD_MAX_BUTTONS];

/* Single-producer (tick ISR) / single-consumer (main loop) ring buffer.
 * Head is written only by the producer, tail only by the consumer, so no
 * lock is needed as long as index updates are atomic (true for uint8_t). */
static volatile uint8_t  s_queue[KEYPAD_QUEUE_SIZE];
static volatile uint8_t  s_head;
static volatile uint8_t  s_tail;
static volatile uint32_t s_dropped;

static void queue_push(uint8_t idx)
{
    uint8_t next = (uint8_t)((s_head + 1u) & (KEYPAD_QUEUE_SIZE - 1u));
    if (next == s_tail) {
        s_dropped++;
        return;
    }
    s_queue[s_head] = idx;
    s_head = next;          /* publish after the data is written */
}

void keypad_init(const keypad_hw_t *hw, uint8_t num_buttons)
{
    s_hw  = hw;
    s_num = (num_buttons > KEYPAD_MAX_BUTTONS) ? KEYPAD_MAX_BUTTONS : num_buttons;
    s_head = s_tail = 0;
    s_dropped = 0;
    for (uint8_t i = 0; i < s_num; i++) {
        s_btn[i].state = BTN_IDLE;
        s_btn[i].timer_ms = 0;
        s_hw->set_irq(i, true);
    }
}

bool keypad_on_edge(uint8_t idx)
{
    if (idx >= s_num || s_btn[idx].state != BTN_IDLE) {
        return false;                       /* already being handled */
    }
    s_hw->set_irq(idx, false);              /* ignore the bounce storm */
    s_btn[idx].timer_ms = KEYPAD_DEBOUNCE_MS;  /* timer before state, so a  */
    s_btn[idx].state = BTN_DEBOUNCE;           /* tick never sees timer = 0 */
    return true;
}

void keypad_tick_1ms(void)
{
    for (uint8_t i = 0; i < s_num; i++) {
        btn_t *b = &s_btn[i];
        bool pressed = s_hw->is_pressed(i);

        switch (b->state) {
        case BTN_IDLE:
            break;

        case BTN_DEBOUNCE:
            if (--b->timer_ms == 0) {
                if (pressed) {
                    queue_push(i);
                    b->state = BTN_HELD;
                    b->timer_ms = KEYPAD_RELEASE_MS;
                } else {
                    /* glitch shorter than the debounce window: ignore */
                    b->state = BTN_IDLE;
                    s_hw->set_irq(i, true);
                }
            }
            break;

        case BTN_HELD:
            if (pressed) {
                b->timer_ms = KEYPAD_RELEASE_MS;    /* still held: restart */
            } else if (--b->timer_ms == 0) {
                b->state = BTN_IDLE;
                s_hw->set_irq(i, true);
            }
            break;
        }
    }
}

bool keypad_pop(uint8_t *idx)
{
    if (s_tail == s_head) {
        return false;
    }
    *idx = s_queue[s_tail];
    s_tail = (uint8_t)((s_tail + 1u) & (KEYPAD_QUEUE_SIZE - 1u));
    return true;
}

uint32_t keypad_dropped(void)
{
    return s_dropped;
}
