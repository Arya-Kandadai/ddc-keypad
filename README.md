# STM32 DDC/CI Monitor-Switch Keypad

An STM32 keypad that switches two monitors between a PC and a Mac by sending
**DDC/CI** commands straight to each monitor over **I²C**. No software runs on
either computer, and no video passes through the keypad: it's a remote control
for the monitors' input selection.

**▶ [Run it in your browser on Wokwi](https://wokwi.com/projects/YOUR-PROJECT-ID)**

![Simulated monitor screens](docs/monitor-screens.png)

```
 PC  --HDMI-------------+            +------------- Mac (USB-C)
                        v            v
                 [ Monitor 1: HDMI 1 | USB-C ]
                 [ Monitor 2: HDMI 2 | USB-C ]
                        ^
          DDC wires only, one I²C bus per monitor
                  (Monitor 1: PB8/PB9, Monitor 2: PA6/PA7)
                        |
                 [ STM32 keypad ]   "Monitor 1: show USB-C"
```

**Features**
- **Interrupt-driven input:** GPIO EXTI on falling edges. The line is masked
  during bounce, a 1 ms SysTick tick debounces it, and events go through a
  lock-free ring buffer to the main loop.
- **DDC/CI driver written from the VESA spec:** packet framing and XOR
  checksums, Set/Get VCP feature `0x60` (input source), spec timing
  (50 ms after a set, 40 ms before reading a reply), and null-message handling.
- **Non-blocking switch state machine:** set → wait → read back → verify,
  retrying with linear back-off. It reports OK, NACK, protocol error or
  mismatch. On real hardware the CPU can sleep (`__WFI`) between ticks
  (`USE_WFI 1`); it's off for Wokwi, whose STM32C0 model doesn't wake from WFI.
- **Simulated monitor (custom Wokwi chip in C):**
  - Validates packets and enforces DDC/CI timing.
  - Has an on-screen display.
  - Has a **fault-injection slider** that produces random NACKs and corrupted
    checksums, so you can watch the retry logic work.
- **One I²C bus per monitor:** every monitor answers at `0x37`, so each gets
  its own bit-banged bus on its own two GPIOs. No mux or bus switch.

---

## Step-by-step: run it in Wokwi (browser, free)

### 1. Start from Wokwi's STM32 HAL template
1. Sign in at https://wokwi.com. You need an account to save projects with
   custom chips.
2. Open the official **Nucleo-C031C6 STM32 HAL Example**:
   https://wokwi.com/projects/365551778332549121
3. Click **Save** (or **Save a copy**) so you have your own editable project.

> Start from this example rather than "New Project → STM32". The example is
> set up for plain C with the STM32Cube HAL, while the new-project route
> gives you an Arduino sketch.

### 2. Replace `main.c`
Open the `main.c` tab, delete everything, and paste in the contents of
**`wokwi/main.c`** from this project.

### 3. Add the simulated monitor chip
1. In the diagram pane, click the blue **+** button, then **Custom Chip**.
2. Name it exactly **`ddc-monitor`**, choose **C**, then click **Create Chip**.
   This creates two new tabs: `ddc-monitor.chip.json` and `ddc-monitor.chip.c`.
3. Replace the contents of each tab with the matching file from `wokwi/`.

The chip name matters: the wiring file refers to it as `chip-ddc-monitor`.

### 4. Replace `diagram.json`
Open the `diagram.json` tab and replace everything with **`wokwi/diagram.json`**.
The circuit appears with:
- the Nucleo board,
- two monitor chips with screens,
- six buttons,
- OK/ERR LEDs,
- a logic analyzer.

### 5. Run it
Press the green **▶ Play** button. The serial monitor should show:
```
=== STM32 DDC/CI Monitor Switch ===
Core clock 48 MHz, DDC: one software I2C bus per monitor
Left  monitor:  PC=HDMI1 (0x11)  Mac=USB-C (0x1B)
Right monitor:  PC=HDMI2 (0x12)  Mac=USB-C (0x1B)
Debounce tick: SysTick ISR
Buttons: EXTI interrupts (vector table in RAM @ 0x20000xxx)
Ready. Press a button.
```
Click **Left > PC**:
```
> Button: Left > PC
    TX 6E: 51 84 03 60 00 11 C9          <- Set VCP 0x60 = HDMI1
    TX 6E: 51 82 01 60 DC                <- Get VCP 0x60 (50 ms later)
    RX 6F: 6E 88 02 00 60 00 00 1B 00 11 DE   <- reply: value 0x11
[OK]   Left monitor -> HDMI1 (0x11) in 92 ms, 1 attempt(s)
```
- Monitor 1's screen turns green and shows **HDMI 1**.
- The green **OK** LED flashes.
- The board's on-board LED stays lit while the switch is in progress.

### 6. Show off the retry logic
1. While the simulation is running, click a monitor chip. A
   **Fault injection (%)** slider appears.
2. Set it to about 15 and press buttons. You'll see `<NACK>` lines in the log, and results
   such as `2 attempt(s)` where a retry recovered the switch.
3. At 40–50%, some switches fail after 3 attempts and the red **ERR** LED
   lights. The monitor screen counts `ERR` and `NACK` events.

### 7. Capture I²C waveforms (great for the README / portfolio)
The logic analyzer records both DDC buses: monitor 1's SCL/SDA on D0/D1 and
monitor 2's SCL/SDA on D2/D3.
1. Run the simulation, press a few buttons, then **stop** the simulation.
   Wokwi downloads a `.vcd` file.
2. Open it in **PulseView** (free, https://sigrok.org).
3. Add an **I²C** decoder with SCL = D0 and SDA = D1 for monitor 1 (add a
   second decoder with D2/D3 for monitor 2). You'll see
   `Address write: 37`, then `51 84 03 60 00 11 C9`, and so on.
4. Take a screenshot for your GitHub README.

---

## Project layout
```
firmware/Core/Inc/   ddc.h  keypad.h  switcher.h  sw_i2c.h  config.h   <- edit config.h for your monitors
firmware/Core/Src/   ddc.c  keypad.c  switcher.c  sw_i2c.c  main.c     <- main.c = HAL init + glue only
chip/                ddc-monitor.chip.c/.json                <- simulated DDC/CI monitor
wokwi/               files to paste into wokwi.com (main.c is generated)
tools/amalgamate.py  regenerates wokwi/main.c from the modules
```
After editing anything in `firmware/`, run `python3 tools/amalgamate.py` and
paste the new `wokwi/main.c` into Wokwi.

## Design notes
- **Why one bus per monitor?** Every DDC/CI monitor answers at address
  `0x37`, so two monitors on one bus would both answer at once (a single
  Set VCP would switch both). Each monitor has its own cable, and so its own DDC wires, so the
  simplest fix is a separate bus per monitor: bit-banged I²C runs on any two
  GPIOs, so there's no mux or bus switch. For many monitors, a TCA9548A mux
  would put up to 8 buses behind 2 pins instead.
- **Why mask EXTI instead of a debounce delay in the ISR?** A bouncing contact
  can fire dozens of interrupts. Masking the line after the first edge costs
  one interrupt per press. The timer then decides whether the press was real.
- **Why +1 ms on every deadline?** `HAL_GetTick()` has 1 ms resolution, so a
  plain `now + 50` could expire after only 49.x ms and break the spec's
  minimum delay. The simulated monitor enforces that minimum, which is how
  this was caught.
- **Why read back after setting?** A Set VCP command has no acknowledgement
  beyond the I²C ACK, so the only proof the input changed is to read
  VCP `0x60` back.
- **Why is the vector table relocated to RAM?** Wokwi's "STM32 HAL"
  template links against the STM32duino core, which already defines
  `EXTIx_IRQHandler` for `attachInterrupt()`, so defining them again fails
  at link time. At boot the firmware copies the 48-entry vector table into
  a 256-byte-aligned RAM array. It then patches the three EXTI entries with
  its own register-level ISR and points `SCB->VTOR` at the copy. If VTOR
  can't be written, it falls back to sampling the pins in the 1 ms tick.
- **Why is `USE_WFI` 0?** Debugging in Wokwi showed the main loop froze
  right after `Ready` because the simulated STM32C0 never woke from `__WFI`.
  A minimal test isolated it: with sleep removed, the SysTick hook, the RAM
  vector table and the EXTI interrupt all worked. Sleep stays available for
  real hardware.
- **Why software I²C?** In Wokwi, the STM32C0's I2C1 peripheral sends the
  address, gets the ACK, then stalls forever (`HAL_ERROR`, error `0x20` timeout,
  `ISR = 0x800B`). The same code worked against Wokwi's built-in LCD, which
  narrowed it to the peripheral model plus custom chips. The DDC buses are
  bit-banged instead (`sw_i2c.c`), with clock-stretch support and a bus
  recovery routine. The lines use pseudo open-drain: a line is pulled low by
  switching the pin to output-low and released by switching it to input. The
  STM32C031 has only one I²C peripheral anyway, so a bus per monitor needs
  software I²C regardless.
- **The 4× clock bug:** Wokwi's template (via STM32duino) raises the core to
  48 MHz before `main()`, but `SystemCoreClock` still said 12 MHz, so
  `HAL_Init()` set SysTick 4× too fast. Every "50 ms" DDC delay was really
  12.5 ms, and the monitor model correctly refused commands that arrived
  inside its busy window. The fix is `SystemCoreClockUpdate()` followed by
  `HAL_InitTick()`. A simulator that enforces the protocol's timing is what
  caught this.
- **Same-priority ISRs:** EXTI and SysTick share keypad state, so they run at
  equal NVIC priority and can't pre-empt each other. That makes locking
  unnecessary.

## Moving to real hardware later
- **Board:** a NUCLEO-C031C6 (about $15 AUD). The code is unchanged.
- **Connecting to a monitor:** an HDMI or DP breakout gives access to the
  monitor's DDC SDA/SCL (pins 15/16 on HDMI).
- **Voltage:** DDC runs at **5 V**, so use a level shifter (BSS138 or PCA9306).
- **Pull-ups:** 4.7 kΩ to 3.3 V on the MCU side.
- **Multiple monitors:** each monitor gets its own SCL/SDA pair (and its own
  level shifter channel). Add a monitor by adding a line to
  `DDC_BUS_PINS_DEF` in `config.h`. For more monitors than spare pins, use a
  TCA9548A mux and make `port_select_bus()` write the channel to it.

## Troubleshooting
| Symptom | Fix |
|---|---|
| Compile errors mentioning `setup`/`loop` or Arduino | You started from an Arduino project. Redo step 1 from the HAL example. |
| `Unknown part type chip-ddc-monitor` | The chip must be named `ddc-monitor`. The tabs should be `ddc-monitor.chip.c` and `.json`. |
| Every switch fails with NACK | Check that monitor 1's SCL/SDA go to D15/D14 and monitor 2's to D12/D11. Check the fault slider is at 0. |
| Diagram errors mentioning `BUS_EN` | You have the old chip files. Replace `ddc-monitor.chip.json` and `.chip.c` with the ones in `wokwi/`. |
| `Debounce tick: main loop (fallback)` | That's fine. The SysTick hook wasn't available, so debouncing runs from the main loop instead. |
| `multiple definition of EXTI..._IRQHandler` | You have an old `main.c`. Paste the current `wokwi/main.c`, which installs its handlers through a RAM vector table instead. |
| Every packet shows `<NACK>` | You have an older `main.c` (hardware I²C, wrong tick). Paste the current `wokwi/main.c`. The banner should say `DDC: one software I2C bus per monitor`. |
| Buttons do nothing, log stops after `Ready` | You have a build with `USE_WFI 1`. Paste the current `wokwi/main.c` (`USE_WFI 0`). |
| `Buttons: VTOR not writable...` | That's fine. Buttons are sampled every 1 ms instead of using interrupts, and debouncing still applies. |
| Nothing in the serial monitor | Check the `nucleo:PA2` / `PA3` ↔ `$serialMonitor` lines in `diagram.json`. |
