/**
 * config.h - Board wiring and button/input mapping. Edit this file to match
 * your monitors; nothing else needs to change.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "ddc.h"

/* ---------------- Monitors ----------------
 * Every DDC/CI monitor answers at the same I2C address (0x37), so two
 * monitors can't share one bus. Each monitor has its own cable, and so its
 * own DDC wires, so each one gets its own software I2C bus on its own pair
 * of GPIOs (see DDC_BUS_PINS_DEF below). No mux or bus switch is needed. */
#define NUM_MONITORS     2u
#define MON_LEFT         0u
#define MON_RIGHT        1u

/* Which input each computer is plugged into, per monitor. */
#define TARGET_PC        0u
#define TARGET_MAC       1u
#define NUM_TARGETS      2u

static const uint16_t INPUT_MAP[NUM_MONITORS][NUM_TARGETS] = {
    /*            PC            Mac        */
    [MON_LEFT]  = { INPUT_HDMI1, INPUT_USBC },
    [MON_RIGHT] = { INPUT_HDMI2, INPUT_USBC },
};

static const char *const MONITOR_NAMES[NUM_MONITORS] = { "Left", "Right" };
static const char *const TARGET_NAMES[NUM_TARGETS]   = { "PC", "Mac" };

/* ---------------- Buttons ---------------- */
#define ALL_MONITORS     0xFFu

typedef struct {
    uint8_t monitor;     /* MON_x or ALL_MONITORS */
    uint8_t target;      /* TARGET_x              */
    const char *label;
} button_action_t;

static const button_action_t BUTTON_ACTIONS[] = {
    { MON_LEFT,     TARGET_PC,  "Left > PC"   },
    { MON_LEFT,     TARGET_MAC, "Left > Mac"  },
    { MON_RIGHT,    TARGET_PC,  "Right > PC"  },
    { MON_RIGHT,    TARGET_MAC, "Right > Mac" },
    { ALL_MONITORS, TARGET_PC,  "All > PC"    },
    { ALL_MONITORS, TARGET_MAC, "All > Mac"   },
};
#define NUM_BUTTONS (sizeof(BUTTON_ACTIONS) / sizeof(BUTTON_ACTIONS[0]))

/* ---------------- Pins (Nucleo-C031C6) ----------------
 * Button pins each use a different EXTI line number (0,1,4,10,3,5).
 * Avoided: PA2/PA3 (UART), PA5 (on-board LED), PA13/PA14 (SWD),
 *          PB8/PB9 and PA6/PA7 (the two DDC buses). */
#define BTN_PINS_DEF                                   \
    { GPIOA, GPIO_PIN_0  },  /* A0 : Left  > PC   */   \
    { GPIOA, GPIO_PIN_1  },  /* A1 : Left  > Mac  */   \
    { GPIOA, GPIO_PIN_4  },  /* A2 : Right > PC   */   \
    { GPIOA, GPIO_PIN_10 },  /* D2 : Right > Mac  */   \
    { GPIOB, GPIO_PIN_3  },  /* D3 : All   > PC   */   \
    { GPIOB, GPIO_PIN_5  },  /* D6 : All   > Mac  */

/* One software I2C bus per monitor, in MON_x order: { port, SCL pin, SDA pin }.
 * Both lines of a bus must be on the same port. */
#define DDC_BUS_PINS_DEF                                         \
    { GPIOB, 8u, 9u },   /* Left : SCL PB8 (D15), SDA PB9 (D14) */  \
    { GPIOA, 6u, 7u },   /* Right: SCL PA6 (D12), SDA PA7 (D11) */

#define LED_BUSY_PORT    GPIOA          /* on-board green LED LD4 (D13) */
#define LED_BUSY_PIN     GPIO_PIN_5
#define LED_OK_PORT      GPIOC          /* D9 */
#define LED_OK_PIN       GPIO_PIN_7
#define LED_ERR_PORT     GPIOA          /* D8 */
#define LED_ERR_PIN      GPIO_PIN_9

#define RESULT_LED_MS    600u           /* how long OK/ERR LED stays lit */


#endif /* CONFIG_H */
