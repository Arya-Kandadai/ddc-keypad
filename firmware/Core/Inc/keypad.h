/**
 * keypad.h - Interrupt-driven button debouncing with an ISR-safe event queue.
 *
 * Flow for each button:
 *   1. GPIO falling edge -> EXTI ISR calls keypad_on_edge(). The line's
 *      interrupt is masked so contact bounce can't flood the CPU.
 *   2. A 1 ms timer tick calls keypad_tick_1ms(). After KEYPAD_DEBOUNCE_MS
 *      the pin is sampled; if still pressed, a press event is queued.
 *   3. Once the pin has been released for KEYPAD_RELEASE_MS, the pending
 *      flag is cleared and the interrupt is unmasked again.
 *
 * Hardware access goes through keypad_hw_t, so this module is unit-testable
 * on a PC. Both entry points must run at the same interrupt priority (they
 * share state without locks); keypad_pop() is called from the main loop.
 */
#ifndef KEYPAD_H
#define KEYPAD_H

#include <stdint.h>
#include <stdbool.h>

#define KEYPAD_MAX_BUTTONS   8u
#define KEYPAD_DEBOUNCE_MS   20u
#define KEYPAD_RELEASE_MS    20u
#define KEYPAD_QUEUE_SIZE    16u   /* must be a power of two */

typedef struct {
    /* Return true if button idx is currently pressed (pin level LOW). */
    bool (*is_pressed)(uint8_t idx);
    /* Enable/disable the edge interrupt for button idx. When re-enabling,
     * the implementation should clear any stale pending flag first. */
    void (*set_irq)(uint8_t idx, bool enable);
} keypad_hw_t;

void     keypad_init(const keypad_hw_t *hw, uint8_t num_buttons);
bool     keypad_on_edge(uint8_t idx);   /* call from EXTI ISR; true if  */
                                        /* it started a new debounce    */
void     keypad_tick_1ms(void);         /* call from 1 ms timer ISR     */
bool     keypad_pop(uint8_t *idx);      /* call from main loop          */
uint32_t keypad_dropped(void);          /* events lost to a full queue  */

#endif /* KEYPAD_H */
