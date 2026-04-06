# Teensy Servo Control

> **Teensy 3.2 · ExpressLRS (CRSF) input · 9-Servo strictly sequential control · Switch-triggered · CH6 safety interlock**
> **Firmware version: v4.2**

A compact, reliable firmware for the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) that reads RC channel data from an **ExpressLRS (ELRS)** receiver over the CRSF serial protocol and drives **9 servos** in a **strictly sequential** fashion — one servo per switch flip, never two servos at once.  A dedicated **CH6 safety interlock** must be armed before any servo movement is permitted.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Required](#2-hardware-required)
3. [Wiring / Pin Connections](#3-wiring--pin-connections)
4. [Code Explanation](#4-code-explanation)
   - [Libraries Used](#41-libraries-used)
   - [CRSF Parsing](#42-crsf-parsing)
   - [Sequential Servo Control Logic](#43-sequential-servo-control-logic)
   - [Flip Counter & All-Close Sequence](#44-flip-counter--all-close-sequence)
   - [CH6 Safety Interlock](#45-ch6-safety-interlock)
   - [LED Feedback](#46-led-feedback)
5. [Setup & Upload Instructions](#5-setup--upload-instructions)
6. [Configuration Options](#6-configuration-options)
7. [Serial Debug Output](#7-serial-debug-output)
8. [Troubleshooting](#8-troubleshooting)
9. [License](#9-license)

---

## 1. Project Overview

This project was built for Project 399 to remotely actuate a mechanical servo-driven mechanism via a standard RC transmitter running ExpressLRS.

**What it does:**

- Receives CRSF frames from an ELRS receiver at 420 000 baud over UART1.
- Monitors a configurable trigger switch channel (default: **CH5 / AUX1**).
- Requires a hardware safety channel (default: **CH6 / AUX2**) to be in the ARMED position before any servo can move.
- Each **rising edge** on CH5 (OFF → ON) advances the servo sequence by exactly **one step**. Only one servo ever moves per flip — no simultaneous activation.

### Flip Sequence

| Flip # | Action |
|--------|--------|
| 1 | Servo 1 extends (index 0, pin 3) |
| 2 | Servo 2 extends (index 1, pin 4) |
| 3 | Servo 3 extends (index 2, pin 5) |
| 4 | Servo 4 extends (index 3, pin 6) |
| 5 | Servo 5 extends (index 4, pin 9) |
| 6 | Servo 6 extends (index 5, pin 10) |
| 7 | Servo 7 extends (index 6, pin 20) |
| 8 | Servo 8 extends (index 7, pin 21) |
| 9 | Servo 9 extends (index 8, pin 22) |
| 10 | All servos retract sequentially (9 → 1), then counter resets to 0 |

> **Critical design property:** Every servo extends independently on its own dedicated switch flip. No two servos are ever commanded simultaneously. The "all close" on flip 10 also retracts servos one at a time (highest index first), with a configurable hold between each.

**Why Teensy 3.2?**

- Hardware UART capable of 420 000 baud (required for CRSF).
- Plenty of PWM-capable pins for 9+ servos.
- 72 MHz ARM Cortex-M4 — more than enough headroom for real-time control.
- Small form factor, 3.3 V I/O compatible with ELRS receivers.

---

## 2. Hardware Required

| Component | Notes |
|-----------|-------|
| Teensy 3.2 | PJRC — `teensy32` board in Teensyduino |
| ELRS receiver | e.g. BetaFPV Nano, Happymodel EP1/EP2, BETAFPV SuperD |
| Servos (×9) | Standard PWM servos, 1000–2000 µs pulse range |
| 5 V BEC / regulator | Power servos externally — **never** from Teensy's 3.3 V pin |
| Logic-level wire | Receiver TX → Teensy RX1 |
| Shared GND | Teensy, receiver, BEC, and servos must share ground |

---

## 3. Wiring / Pin Connections

```
┌─────────────────────────────────────────────────────────┐
│  ELRS Receiver                     Teensy 3.2            │
│  ─────────────                     ──────────            │
│  TX ─────────────────────────────► Pin 0  (RX1 / UART1)  │
│  RX ◄─────────────────────────────  Pin 1  (TX1 / UART1) │  ← optional telemetry
│  GND ────────────────────────────── GND                  │
│  VCC ────────────────────────────── 3.3V or 5V *         │
│                                                           │
│  Servo 1 Signal ─────────────────► Pin 3  (PWM / FTM2)   │
│  Servo 2 Signal ─────────────────► Pin 4  (PWM / FTM1)   │
│  Servo 3 Signal ─────────────────► Pin 5  (PWM / FTM0)   │
│  Servo 4 Signal ─────────────────► Pin 6  (PWM / FTM0)   │
│  Servo 5 Signal ─────────────────► Pin 9  (PWM / FTM0)   │
│  Servo 6 Signal ─────────────────► Pin 10 (PWM / FTM0)   │
│  Servo 7 Signal ─────────────────► Pin 20 (PWM / FTM0)   │
│  Servo 8 Signal ─────────────────► Pin 21 (PWM / FTM0)   │
│  Servo 9 Signal ─────────────────► Pin 22 (PWM / FTM0)   │
│                                                           │
│  Servo Power (all) ──────────────── External 5V BEC      │
│  Servo GND (all) ────────────────── Shared GND           │
└─────────────────────────────────────────────────────────┘
```

> **⚠️ Power note:** 9 servos draw significant current (0.5–1 A each under load).  
> Always power them from a dedicated 5 V BEC — not the Teensy's onboard regulators.

> **⚠️ Receiver voltage:** Check your specific ELRS receiver datasheet.  
> Most nano receivers accept 3.3 V–5 V. When powering from the BEC (5 V), use the receiver's dedicated VCC pad, not Teensy's 3.3 V output.

### Pin Assignment Summary

| Servo # | Index | Teensy Pin | Timer Channel |
|---------|-------|-----------|--------------|
| 1 | 0 | 3 | FTM2 CH0 |
| 2 | 1 | 4 | FTM1 CH0 |
| 3 | 2 | 5 | FTM0 CH7 |
| 4 | 3 | 6 | FTM0 CH4 |
| 5 | 4 | 9 | FTM0 CH2 |
| 6 | 5 | 10 | FTM0 CH3 |
| 7 | 6 | 20 | FTM0 CH5 |
| 8 | 7 | 21 | FTM0 CH6 |
| 9 | 8 | 22 | FTM0 CH0 |

All selected pins are hardware PWM-capable on the Teensy 3.2. Pins 0 and 1 are reserved for CRSF serial (RX1/TX1).

---

## 4. Code Explanation

### 4.1 Libraries Used

| Library | Source | Purpose |
|---------|--------|---------|
| `Servo.h` | Built into Teensyduino | PWM servo output |
| `crsf_parser.h` | Included in this repo | CRSF frame parsing (no external dependency) |

`crsf_parser.h` is a self-contained, zero-dependency C++ class — no installation required. It is included in the sketch folder alongside the `.ino` file.

### 4.2 CRSF Parsing

`CrsfParser` implements the **CRSF (Crossfire Serial Protocol)** frame parser:

```
Frame layout:
  [0xC8 SYNC] [LEN] [TYPE] [PAYLOAD …] [CRC8/DVB-S2]
```

- **Sync detection** — the parser sits in `S_SYNC` state until it sees byte `0xC8`.
- **Length field** — covers TYPE + PAYLOAD + CRC (excludes SYNC and LEN itself).
- **Type filtering** — only `RC_CHANNELS_PACKED` frames (`0x16`) are decoded.
- **CRC validation** — uses CRC-8/DVB-S2 (polynomial `0xD5`) computed over TYPE + PAYLOAD.
- **Channel unpacking** — 22 payload bytes hold 16 channels × 11 bits, little-endian packed.

```cpp
// Feed bytes from hardware UART:
while (Serial1.available()) crsf.feed(Serial1.read());

// Check for decoded frame:
if (crsf.hasNewFrame()) {
    uint16_t ch5 = crsf.getChannel(5);  // 1-indexed, range 172–1811
}
```

Channel values are in the CRSF native range **172–1811** (midpoint ≈ 992).

### 4.3 Sequential Servo Control Logic

**Core principle: one servo, one flip, no chaining.**

Each time the trigger switch produces a **rising edge** (OFF → ON) while the safety is armed, exactly one servo is commanded — the next one in the sequence. The firmware never commands two servos in the same loop iteration.

```
Flip 1  → servos[0].writeMicroseconds(SERVO_MAX_US)   (only servo 0 moves)
Flip 2  → servos[1].writeMicroseconds(SERVO_MAX_US)   (only servo 1 moves)
...
Flip 9  → servos[8].writeMicroseconds(SERVO_MAX_US)   (only servo 8 moves)
Flip 10 → all-close retract sequence begins
```

Between flips, the firmware waits for the next rising edge. No servo moves unless the switch is toggled.

**Edge detection** uses a `lastSwitch` boolean compared against the current `switchOn` state. Only the transition OFF→ON triggers an action — holding the switch ON does not re-trigger.

**Non-blocking timing** — `millis()` is used throughout; no `delay()` calls anywhere in the code.

### 4.4 Flip Counter & All-Close Sequence

A `flipCount` variable tracks how many servos have been extended (0–9).

- **Flips 1–9** (`flipCount < NUM_SERVOS`): The servo at index `flipCount` is extended and `flipCount` is incremented.
- **Flip 10** (`flipCount == NUM_SERVOS`): The all-close retract sequence starts.

**All-close retract sequence:**

Servos retract **one at a time**, highest index first (servo 9 → servo 1), with `RETRACT_HOLD_MS` (default 300 ms) between each. This is driven by a small non-blocking state machine (`runRetractSequence()`) called every `loop()` iteration. Switch input is **ignored** while the retract sequence is running. When the last servo finishes its hold, `flipCount` resets to 0 and the cycle is ready to repeat.

### 4.5 CH6 Safety Interlock

A dedicated safety channel (CH6 / AUX2) must be in the ARMED position before any servo can move:

```
CH6 ≤ SAFETY_THRESHOLD  →  SAFE   (no servo movement permitted)
CH6 > SAFETY_THRESHOLD  →  ARMED  (CH5 trigger is active)
```

When `systemArmed` is `false`:
- `switchOn` is forced to `false` every loop, so no rising edges can register.
- The edge-detection latch (`lastSwitch`) is also cleared, preventing a false trigger when the system transitions back to ARMED.
- Any retract sequence already in progress is **allowed to complete** (failsafe: let the mechanism settle before locking out).

Serial debug output logs each ARM/SAFE transition so you can verify interlock state.

### 4.6 LED Feedback

The onboard LED (pin 13 / `LED_BUILTIN`) provides three distinct visual states.  
All LED transitions are **fully non-blocking** — implemented as a state machine driven by `millis()` with no `delay()` calls.

#### LED Behavior Table

| LED Pattern | Meaning |
|-------------|---------|
| **Solid ON** | Board is powered and firmware is running. No CRSF signal received yet. |
| **Continuous blink** (200 ms ON / 200 ms OFF) | CRSF signal is actively being received. Waiting for switch flips. |
| **Double blink** (ON 100ms · OFF 100ms · ON 100ms · OFF 300ms, repeat) | All-close retract sequence is executing. |

#### Priority

```
1. Double-blink   (highest) — retract sequence is active
2. Continuous blink          — CRSF signal present, no retract running
3. Solid ON       (lowest)  — board alive, no signal
```

---

## 5. Setup & Upload Instructions

### Prerequisites

1. **Arduino IDE** 1.8.x or 2.x — [download](https://www.arduino.cc/en/software)
2. **Teensyduino add-on** — [download from PJRC](https://www.pjrc.com/teensy/teensyduino.html)  
   Install over the top of your existing Arduino IDE installation.

### Steps

```
1. Clone or download this repository.

2. Open TeensyServoControl/TeensyServoControl.ino in the Arduino IDE.
   (Both .ino and crsf_parser.h must be in the same folder.)

3. In the Arduino IDE menus:
     Tools → Board → Teensyduino → Teensy 3.2

4. Set CPU Speed:
     Tools → CPU Speed → 72 MHz  (default — fine for this sketch)

5. Set USB Type:
     Tools → USB Type → Serial  (enables USB Serial for debug output)

6. Plug in your Teensy 3.2 via USB.

7. Click Upload (→).
   The Teensy Loader will open automatically and flash the board.
```

### Verify it's running

Open `Tools → Serial Monitor` at **115200 baud**. You should see:

```
TeensyServoControl v4.2 — ready
Servos on pins: 3, 4, 5, 6, 9, 10, 20, 21, 22
Trigger channel: CH5
Safety channel:  CH6  threshold: 1500
Servos per cycle: 9
Mode: strictly sequential — one servo per switch flip
Flip 1-9: extend servo 1-9 individually
Flip 10 : all servos retract sequentially (9→1)
LED pin: 13
LED modes: solid=powered, blink=signal, double-blink=retract active
Safety: SAFE (waiting for CH6 ARM)
```

**Expected behavior:**

1. Arm CH6 → serial shows `Safety: ARMED`.
2. Flip CH5 once → only Servo 1 extends. All others stay put.
3. Flip again → only Servo 2 extends. All others stay put.
4. Continue for Servos 3–9.
5. 10th flip → LED double-blinks while all servos retract one by one (9 → 1). Counter resets. Ready for flip 1 again.

---

## 6. Configuration Options

All user-configurable options are at the top of `TeensyServoControl.ino`:

| Constant | Default | Description |
|----------|---------|-------------|
| `CRSF_SERIAL` | `Serial1` | UART connected to ELRS receiver |
| `CRSF_BAUD` | `420000` | CRSF baud rate (do not change) |
| `TRIGGER_CHANNEL` | `5` | RC channel number that advances the sequence (1–16) |
| `SWITCH_THRESHOLD` | `1200` | Channel value above this = switch ON (range 172–1811) |
| `SAFETY_CHANNEL` | `6` | RC channel used as safety interlock (1–16) |
| `SAFETY_THRESHOLD` | `1500` | Channel value that must be exceeded to ARM the system |
| `SERVO_PINS[]` | `{3, 4, 5, 6, 9, 10, 20, 21, 22}` | Teensy pin numbers for each of the 9 servos |
| `SERVO_MIN_US` | `1000` | Servo minimum pulse width / retracted position (µs) |
| `SERVO_MAX_US` | `2000` | Servo maximum pulse width / extended position (µs) |
| `RETRACT_HOLD_MS` | `300` | Hold time (ms) between each servo during the all-close retract |
| `LED_PIN` | `LED_BUILTIN` (pin 13) | Onboard LED pin — do not change for Teensy 3.2 |
| `LED_SIGNAL_ON_MS` | `200` | Continuous blink ON duration (signal present) |
| `LED_SIGNAL_OFF_MS` | `200` | Continuous blink OFF duration (signal present) |
| `LED_SERVO_PULSE_MS` | `100` | Double-blink each pulse duration (retract active) |
| `LED_SERVO_GAP_MS` | `300` | Double-blink gap after second pulse (retract active) |
| `CRSF_SIGNAL_TIMEOUT_MS` | `500` | ms without a CRSF frame before signal is marked lost |

### Adjusting switch thresholds

- For a **2-position switch**: `SWITCH_THRESHOLD = 1200`, `SAFETY_THRESHOLD = 1500`.
- For a **3-position switch used as safety**: set `SAFETY_THRESHOLD = 1600` so only the full-up position arms the system.

### Adding / removing servos

Edit `SERVO_PINS[]`:

```cpp
static const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };  // 9 servos
```

`NUM_SERVOS` and `FLIPS_BEFORE_CLOSE` are computed automatically via `sizeof`. The flip counter and all-close logic scale automatically — no other changes needed.

Additional hardware PWM-capable pins on the Teensy 3.2: `23`, `25`, `32`.

---

## 7. Serial Debug Output

With USB connected and Serial Monitor open at **115200 baud**:

```
TeensyServoControl v4.2 — ready
...
Safety: SAFE (waiting for CH6 ARM)
CH6: 2000  Safety: ARMED
CH5: 172  Switch: OFF  FlipCount: 0
CH5: 1811  Switch: ON   FlipCount: 0
→ Flip 1: Servo 1 extended
CH5: 172  Switch: OFF  FlipCount: 1
CH5: 1811  Switch: ON   FlipCount: 1
→ Flip 2: Servo 2 extended
...
→ Flip 10: starting all-close retract sequence
  Retract servo 9
  Retract servo 8
  ...
  Retract servo 1
→ ALL CLOSED — cycle reset to 0
```

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No channel data / `CH5: 0` | Wrong UART RX pin or baud rate | Check wiring to Pin 0; verify CRSF output mode on receiver |
| Servos don't move | Safety not armed, wrong servo pins, or no external power | Confirm CH6 is ARMED; check `SERVO_PINS[]` and BEC voltage |
| Multiple servos move on one flip | Should not happen with v4.1+ — if seen, re-flash | All simultaneous-fire chaining was removed in v4.1 |
| All-close won't start | Switch ignored while retract running | Wait for the retract sequence to finish |
| Flips work but sequence restarts unexpectedly | `flipCount` reset mid-cycle | Only happens after a completed retract or power cycle — expected |
| Upload fails | Teensy not in bootloader mode | Press the button on the Teensy or check USB connection |
| `crsf_parser.h` not found | File not in same folder as `.ino` | Both files must be inside `TeensyServoControl/` folder |
| Servos jitter or draw excess current | Insufficient BEC capacity | Ensure BEC is rated for combined stall current (≥9 A recommended) |
| LED stays solid, won't blink | No CRSF signal received | Check receiver wiring/binding; LED blinks only when frames arrive |
| Safety always shows SAFE | Wrong channel or threshold | Verify `SAFETY_CHANNEL` and `SAFETY_THRESHOLD` match your transmitter mapping |

---

## 9. License

MIT License — free to use, modify, and distribute.  
© 2026 Jordan Temkin / Project 399

---

## Changelog

### v4.2
- **Live RC channel readout** on Serial Monitor (115200 baud)
- Prints CH5 and CH6 raw CRSF values every 200 ms — always active regardless of armed/safe state
- Output format: `CH5: 1811 | CH6: 172  [ARMED | Switch: ON  | Flip: 3]`
- Configurable via `RC_READOUT_INTERVAL_MS` (default 200 ms)
- Added `latestCh5` / `latestCh6` globals updated on every CRSF frame so the readout always reflects the most recent values
- Version bumped to v4.2

### v4.1
- **Strictly sequential servo activation** — one servo per switch flip, no exceptions
- Removed all simultaneous-fire chaining: previous `holdMs = 0` back-to-back deploy steps caused servos 2+3 and 6+7 to fire at the same time. Eliminated entirely.
- Removed `runSequence()`, `DEPLOY_SEQUENCE`, and `RETRACT_SEQUENCE` struct arrays.
- Replaced with **flip-counter** model:
  - Flip 1–9: extend servo 1–9 individually (one per flip, strictly)
  - Flip 10: all-close retract sequence (one servo at a time, index 8 → 0, with `RETRACT_HOLD_MS` between each)
- Preserved CH6 hardware safety interlock from v4.0
- Switch input ignored while retract sequence is running
- `flipCount` resets to 0 after retract completes for cycle repeat
- Version bumped to v4.1
- README rewritten to document strictly sequential behavior and safety interlock

### v4.0
- Added **CH6 hardware safety interlock**
  - New `SAFETY_CHANNEL` (CH6) and `SAFETY_THRESHOLD` (1500) constants
  - `systemArmed` flag: CH5 trigger is ignored when CH6 is in SAFE position
  - Edge-detection latch (`lastSwitch`) cleared on disarm to prevent false triggers
  - Serial debug logs each ARM/SAFE transition

### v3.1
- **Servo deadband** (`SERVO_DEADBAND_US`) to suppress jitter at neutral
- Reduced deadband from 100 µs to 10 µs after real-world testing
- Servo output only updates when commanded position differs from last position by more than the deadband threshold

### v3.0
- Revised LED feedback — three distinct non-blocking LED states
- LED driven by non-blocking state machine (`updateLed()`) — no `delay()` calls
- Added `signalPresent` flag and `CRSF_SIGNAL_TIMEOUT_MS` timeout
- Added `servoActive` flag set/cleared by the sequence state machine

### v2.1
- Added onboard LED feedback (pin 13 / `LED_BUILTIN`)
- `stepTimer` re-sampled with `millis()` after each blink

### v2.0
- Expanded from 4 to 9 servos
- Updated `SERVO_PINS[]` to all 9 hardware PWM pins

### v1.0
- Initial release — 4-servo sequencer

---

*Built with Teensyduino · ExpressLRS · Project 399*
