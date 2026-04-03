# Teensy Servo Control

> **Teensy 3.2 · ExpressLRS (CRSF) input · 9-Servo sequencing · Switch-triggered**

A compact, reliable firmware for the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) that reads RC channel data from an **ExpressLRS (ELRS)** receiver over the CRSF serial protocol and drives **9 servos** through a multi-step, timed sequence — triggered by a designated switch channel.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Required](#2-hardware-required)
3. [Wiring / Pin Connections](#3-wiring--pin-connections)
4. [Code Explanation](#4-code-explanation)
   - [Libraries Used](#41-libraries-used)
   - [CRSF Parsing](#42-crsf-parsing)
   - [Servo Control Logic](#43-servo-control-logic)
   - [Servo Deadband](#44-servo-deadband)
   - [Sequence Steps](#45-sequence-steps)
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
- Monitors a configurable switch channel (default: **CH5 / AUX1**).
- When the switch crosses a threshold (ON): executes a **deploy sequence** — moving each of the 9 servos to their target positions in a defined order with configurable hold times between steps.
- When the switch returns low (OFF): executes a **retract sequence** in reverse.
- State machine prevents mid-sequence interruption and ensures clean transitions.
- **Onboard LED (Pin 13)** provides three distinct visual states — see [LED Feedback](#45-led-feedback).

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
│  Servo 0 Signal ─────────────────► Pin 3  (PWM / FTM2)   │
│  Servo 1 Signal ─────────────────► Pin 4  (PWM / FTM1)   │
│  Servo 2 Signal ─────────────────► Pin 5  (PWM / FTM0)   │
│  Servo 3 Signal ─────────────────► Pin 6  (PWM / FTM0)   │
│  Servo 4 Signal ─────────────────► Pin 9  (PWM / FTM0)   │
│  Servo 5 Signal ─────────────────► Pin 10 (PWM / FTM0)   │
│  Servo 6 Signal ─────────────────► Pin 20 (PWM / FTM0)   │
│  Servo 7 Signal ─────────────────► Pin 21 (PWM / FTM0)   │
│  Servo 8 Signal ─────────────────► Pin 22 (PWM / FTM0)   │
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

| Servo Index | Teensy Pin | Timer Channel | Notes |
|-------------|-----------|--------------|-------|
| 0 | 3 | FTM2 CH0 | Hardware PWM |
| 1 | 4 | FTM1 CH0 | Hardware PWM |
| 2 | 5 | FTM0 CH7 | Hardware PWM |
| 3 | 6 | FTM0 CH4 | Hardware PWM |
| 4 | 9 | FTM0 CH2 | Hardware PWM |
| 5 | 10 | FTM0 CH3 | Hardware PWM |
| 6 | 20 | FTM0 CH5 | Hardware PWM |
| 7 | 21 | FTM0 CH6 | Hardware PWM |
| 8 | 22 | FTM0 CH0 | Hardware PWM |

All selected pins are hardware PWM-capable on the Teensy 3.2. Pins 0 and 1 are reserved for CRSF serial (RX1/TX1).

### CRSF Protocol note

CRSF uses a **single-wire, half-duplex** connection at 420 000 baud. Connect the receiver's **TX** pin to Teensy **Pin 0 (RX1)**. The TX line from Teensy (Pin 1) is only needed if you want to send telemetry back to the transmitter.

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

Channel values are in the CRSF native range **172–1811** (midpoint ≈ 992). Use `CrsfParser::toServoUs(val)` to map directly to 1000–2000 µs for servo control.

### 4.3 Servo Control Logic

Servos are driven by the standard Teensyduino `Servo` library using `writeMicroseconds()` for precise, sub-millisecond resolution.

Each servo's last-written position is tracked in `servoPositionUs[]`. Before any `writeMicroseconds()` call, the firmware computes the distance between the current and target positions and applies the [servo deadband](#44-servo-deadband) check.

The firmware uses a **non-blocking state machine** with four states:

| State | Description |
|-------|-------------|
| `STATE_IDLE` | Waiting for switch ON (rising edge) |
| `STATE_DEPLOYING` | Executing deploy sequence step-by-step |
| `STATE_DEPLOYED` | Sequence complete — waiting for switch OFF |
| `STATE_RETRACTING` | Executing retract sequence step-by-step |

**Edge detection** — the trigger fires on a _rising_ edge (OFF→ON) for deploy and _falling_ edge (ON→OFF) for retract, preventing re-triggering while the switch is held.

**Non-blocking timing** — `millis()` is used instead of `delay()`, so the CRSF parser continues to receive bytes even during sequence execution.

### 4.4 Servo Deadband

The **servo deadband** prevents continuous micro-adjustments that cause servo jitter, heat, and unnecessary current draw.

#### How It Works

Every servo's most recently commanded position is stored in the `servoPositionUs[]` array (one entry per servo, initialised to `SERVO_MIN_US` at startup). Before sending a new `writeMicroseconds()` command the firmware computes the absolute distance between the current stored position and the new target:

```
delta = |targetUs - currentUs|
```

If `delta ≤ SERVO_DEADBAND_US` the write is **suppressed** — no PWM update is sent to the servo hardware and `servoPositionUs[]` is left unchanged. If `delta > SERVO_DEADBAND_US` the write proceeds normally and `servoPositionUs[]` is updated to the new target.

```cpp
void moveServo(uint8_t idx, uint16_t us) {
  if (idx >= NUM_SERVOS) return;

  uint16_t current = servoPositionUs[idx];
  uint16_t delta   = (us > current) ? (us - current) : (current - us);

  if (delta <= SERVO_DEADBAND_US) {
    return;  // within deadband — skip write to suppress jitter
  }

  servos[idx].writeMicroseconds(us);
  servoPositionUs[idx] = us;
}
```

#### Configurable Parameter

| Constant | Default | Description |
|----------|---------|-------------|
| `SERVO_DEADBAND_US` | `100` | Minimum position change (µs) required to issue a new servo write |

The default of **100 µs** (10% of the full 1000–2000 µs range) is a conservative value that eliminates jitter from repeated identical or near-identical commands without sacrificing meaningful travel resolution. Reduce it if finer positioning accuracy is required; increase it if servos still buzz between sequence steps.

#### Why This Matters

In a multi-step sequence the same servo index can appear in several consecutive steps (e.g. servo 0 is moved to `SERVO_MAX_US` in step 1 and then to `SERVO_MID_US` in the final latching step). Without deadband logic, if the state machine were to revisit a step due to a timing edge case the servo would receive redundant writes and hunt around its target, drawing current and causing mechanical wear. The deadband makes every step **idempotent** — running it twice has the same effect as running it once.

---

### 4.6 LED Feedback

The onboard LED (pin 13 / `LED_BUILTIN`) provides three distinct visual states.  
All LED transitions are **fully non-blocking** — implemented as a state machine driven by `millis()` with no `delay()` calls. This ensures servo timing and CRSF communication are never interrupted by LED updates.

#### LED Behavior Table

| LED Pattern | Meaning |
|-------------|---------|
| **Solid ON** | Board is powered and firmware is running. No CRSF signal received yet. |
| **Continuous blink** (200 ms ON / 200 ms OFF) | CRSF signal is actively being received from the ELRS receiver. |
| **Double blink** (ON 100ms · OFF 100ms · ON 100ms · OFF 300ms, repeat) | A servo sequence is currently executing (deploy or retract). |

#### Priority

When multiple conditions are true simultaneously, the highest-priority pattern wins:

```
1. Double-blink   (highest) — servo sequence is active
2. Continuous blink          — CRSF signal present, no sequence running
3. Solid ON       (lowest)  — board alive, no signal
```

#### Signal Detection

CRSF frame arrival sets `signalPresent = true`. If no new frame is received within **500 ms** (`CRSF_SIGNAL_TIMEOUT_MS`), the flag clears and the LED reverts to solid ON. This window is long enough to survive the ~6.67 ms CRSF packet interval by a wide margin, but short enough to detect a genuine signal loss within half a second.

#### Timing Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `LED_SIGNAL_ON_MS` | `200` | Continuous blink: ON duration |
| `LED_SIGNAL_OFF_MS` | `200` | Continuous blink: OFF duration |
| `LED_SERVO_PULSE_MS` | `100` | Double-blink: each pulse ON/OFF duration |
| `LED_SERVO_GAP_MS` | `300` | Double-blink: longer gap after second pulse |
| `CRSF_SIGNAL_TIMEOUT_MS` | `500` | ms without a CRSF frame → signal considered lost |

#### Implementation

The LED is driven by `updateLed()`, called on every `loop()` iteration:

```cpp
// Called every loop() — zero blocking time
void updateLed() {
  LedMode desired = servoActive ? LED_MODE_SERVO
                  : signalPresent ? LED_MODE_SIGNAL
                  : LED_MODE_SOLID;

  if (desired != ledMode) { /* reset phase, apply first output */ }

  // Advance phase within current pattern based on millis() elapsed
  // ...
}
```

CRSF frames update the signal-present flag:

```cpp
if (crsf.hasNewFrame()) {
    lastFrameMs   = millis();
    signalPresent = true;
    // ... read channels ...
}

// Timeout check (every loop):
if (signalPresent && (millis() - lastFrameMs) > CRSF_SIGNAL_TIMEOUT_MS) {
    signalPresent = false;
}
```

---

### 4.5 Sequence Steps

Each step in a sequence specifies:

```cpp
struct SequenceStep {
  uint8_t  servoIndex;   // Which servo to move (0-indexed into SERVO_PINS[])
  uint16_t targetUs;     // Target pulse width in microseconds
  uint16_t holdMs;       // Time to wait before executing next step
};
```

**Zero hold time** (`holdMs = 0`) causes two steps to fire simultaneously — useful for triggering multiple servos at the same instant.

Default deploy sequence (customize in `TeensyServoControl.ino`):

```
Step 1:  Servo 0 → 2000 µs (full extend)  | hold 300 ms
Step 2:  Servo 1 → 2000 µs               | hold 300 ms
Step 3:  Servo 2 → 2000 µs               | hold   0 ms  ← fires simultaneously with step 4
Step 4:  Servo 3 → 2000 µs               | hold 300 ms
Step 5:  Servo 4 → 2000 µs               | hold 300 ms
Step 6:  Servo 5 → 2000 µs               | hold 300 ms
Step 7:  Servo 6 → 2000 µs               | hold   0 ms  ← fires simultaneously with step 8
Step 8:  Servo 7 → 2000 µs               | hold 300 ms
Step 9:  Servo 8 → 2000 µs               | hold 500 ms
Step 10: Servo 0 → 1500 µs (latch/mid)   | hold 200 ms
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
TeensyServoControl v3.0 — ready
Servos on pins: 3, 4, 5, 6, 9, 10, 20, 21, 22
Trigger channel: CH5
LED pin: 13
LED modes: solid=powered, blink=signal, double-blink=servo active
CH5: 172  Switch: OFF
CH5: 172  Switch: OFF
…
```

**Expected LED behavior:**
- **On first power-up (no receiver connected):** LED ON solid.
- **After connecting ELRS receiver (bound and outputting):** LED starts continuous blinking (200 ms / 200 ms).
- **When you flip the trigger switch:** LED switches to double-blink pattern for the duration of the servo sequence, then reverts to continuous blink.

---

## 6. Configuration Options

All user-configurable options are at the top of `TeensyServoControl.ino`:

| Constant | Default | Description |
|----------|---------|-------------|
| `CRSF_SERIAL` | `Serial1` | UART connected to ELRS receiver |
| `CRSF_BAUD` | `420000` | CRSF baud rate (do not change) |
| `TRIGGER_CHANNEL` | `5` | RC channel number that triggers the sequence (1–16) |
| `SWITCH_THRESHOLD` | `1200` | Channel value above this = switch ON (range 172–1811) |
| `SERVO_PINS[]` | `{3, 4, 5, 6, 9, 10, 20, 21, 22}` | Teensy pin numbers for each of the 9 servos |
| `SERVO_MIN_US` | `1000` | Servo minimum pulse width (µs) |
| `SERVO_MAX_US` | `2000` | Servo maximum pulse width (µs) |
| `SERVO_MID_US` | `1500` | Servo centre/mid pulse width (µs) |
| `SERVO_DEADBAND_US` | `100` | Minimum µs change to issue a new servo write (deadband range) |
| `DEPLOY_SEQUENCE[]` | See code | Steps to run when switch goes ON |
| `RETRACT_SEQUENCE[]` | See code | Steps to run when switch goes OFF |
| `LED_PIN` | `LED_BUILTIN` (pin 13) | Onboard LED pin — do not change for Teensy 3.2 |
| `LED_SIGNAL_ON_MS` | `200` | Continuous blink ON duration (signal present) |
| `LED_SIGNAL_OFF_MS` | `200` | Continuous blink OFF duration (signal present) |
| `LED_SERVO_PULSE_MS` | `100` | Double-blink each pulse duration (servo active) |
| `LED_SERVO_GAP_MS` | `300` | Double-blink gap after second pulse (servo active) |
| `CRSF_SIGNAL_TIMEOUT_MS` | `500` | ms without a CRSF frame before signal is marked lost |

### Changing the trigger channel

Map `TRIGGER_CHANNEL` to any AUX switch on your transmitter. Typical EdgeTX/OpenTX mapping:

| Channel | Label | Common Use |
|---------|-------|-----------|
| 5 | AUX1 | SA / 2-pos switch |
| 6 | AUX2 | SB / 3-pos switch |
| 7 | AUX3 | SC |
| 8 | AUX4 | SD |

### Adjusting the threshold

- For a **2-position switch**: set `SWITCH_THRESHOLD = 1200` (detects high position).
- For a **3-position switch**: set `SWITCH_THRESHOLD = 1600` (only fires at top position).

### Adding / removing servos

Edit `SERVO_PINS[]`:

```cpp
static const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };  // 9 servos
```

`NUM_SERVOS` is computed automatically via `sizeof`. Then update `DEPLOY_SEQUENCE` and
`RETRACT_SEQUENCE` to match your new servo count. Additional hardware PWM-capable pins
on the Teensy 3.2: `23`, `25`, `32`.

---

## 7. Serial Debug Output

With USB connected and Serial Monitor open at **115200 baud**:

```
TeensyServoControl v3.0 — ready
Servos on pins: 3, 4, 5, 6, 9, 10, 20, 21, 22
Trigger channel: CH5
LED pin: 13
LED modes: solid=powered, blink=signal, double-blink=servo active
CH5: 172  Switch: OFF       ← polling every 200 ms (non-blocking)
CH5: 1811  Switch: ON
→ DEPLOYING
→ DEPLOYED
CH5: 172  Switch: OFF
→ RETRACTING
→ IDLE
```

To reduce serial overhead in production, comment out the debug block inside the `hasNewFrame()` branch in `loop()`.

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No channel data / `CH5: 0` | Wrong UART RX pin or baud rate | Check wiring to Pin 0; verify CRSF output mode on receiver |
| Servos don't move | Wrong servo pins or no external power | Confirm `SERVO_PINS[]`, check BEC voltage at servo connector |
| Sequence fires immediately on boot | Receiver not bound / outputting zero | Add a startup delay or check receiver binding |
| Sequence runs twice | Switch debounce | Edge detection is already implemented; check for electrical noise on the switch line |
| Upload fails | Teensy not in bootloader mode | Press the button on the Teensy or check USB connection |
| `crsf_parser.h` not found | File not in same folder as `.ino` | Both files must be inside `TeensyServoControl/` folder |
| Servos jitter or draw excess current | All 9 servos from same BEC | Ensure BEC is rated for the combined stall current (≥9 A recommended) |
| Servo doesn’t move to a new target | Target is within deadband of current position | Adjust `SERVO_DEADBAND_US` downward, or verify the sequence step target differs by more than 100 µs from the prior position |
| LED stays solid, won't blink | No CRSF signal received | Check receiver wiring/binding; LED blinks only when frames arrive |
| LED blinks but double-blink never shows | Switch threshold not crossed | Verify `TRIGGER_CHANNEL` and `SWITCH_THRESHOLD` settings |

---

## 9. License

MIT License — free to use, modify, and distribute.  
© 2026 Jordan Temkin / Project 399

---

## Changelog

### v3.1
- **Added servo deadband** (`SERVO_DEADBAND_US = 100` µs) — `moveServo()` now suppresses `writeMicroseconds()` calls when the target is within the deadband of the current position.
- Added `servoPositionUs[]` array to track the last-written position for each servo.
- `resetServos()` initialises `servoPositionUs[]` to `SERVO_MIN_US` on startup.
- Updated README with [Servo Deadband](#44-servo-deadband) section, updated Configuration Options table, and new Troubleshooting entry.
- Bumped version string to `v3.1`.

### v3.0
- **Revised LED feedback** — three distinct non-blocking LED states:
  - **Solid ON** — board powered, no CRSF signal (same as before)
  - **Continuous blink** (200 ms / 200 ms) — CRSF signal actively being received *(new)*
  - **Double blink** (ON 100ms / OFF 100ms / ON 100ms / OFF 300ms, looping) — servo sequence executing *(new)*
- LED is now driven by a **non-blocking state machine** (`updateLed()`) — no `delay()` calls; servo timing and CRSF parsing are fully unaffected
- Added `signalPresent` flag and `CRSF_SIGNAL_TIMEOUT_MS` timeout for signal-loss detection
- Added `servoActive` flag set/cleared by the sequence state machine
- Removed blocking `blinkLed()` helper (replaced by `updateLed()`)
- Replaced individual `LED_BLINK_MS` constant with four granular timing constants (`LED_SIGNAL_ON_MS`, `LED_SIGNAL_OFF_MS`, `LED_SERVO_PULSE_MS`, `LED_SERVO_GAP_MS`)
- Bumped version string to `v3.0`

### v2.1
- Added **onboard LED feedback** (pin 13 / `LED_BUILTIN`)
  - LED turns ON solid at power-up (board-alive indicator)
  - LED blinks briefly (`LED_BLINK_MS` = 75 ms) on every servo activation step
- Added `blinkLed()` helper; `LED_PIN` and `LED_BLINK_MS` are configurable constants
- `stepTimer` re-sampled with `millis()` after each blink to preserve hold-time accuracy
- Startup serial output now prints LED pin and blink duration
- Bumped version string to `v2.1`

### v2.0
- Expanded from 4 to **9 servos**
- Updated `SERVO_PINS[]` to use all 9 hardware PWM pins: `3, 4, 5, 6, 9, 10, 20, 21, 22`
- Extended `DEPLOY_SEQUENCE` and `RETRACT_SEQUENCE` to cover all 9 servo indices
- Updated wiring diagram and pin assignment table in README
- Bumped version string to `v2.0` in startup message

### v1.0
- Initial release — 4-servo sequencer

---

*Built with Teensyduino · ExpressLRS · Project 399*
