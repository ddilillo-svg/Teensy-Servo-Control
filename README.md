# Teensy Servo Control

> **Teensy 3.2 · ExpressLRS (CRSF) input · 9-Servo sequencing · Switch-triggered · CH6 Safety Interlock**

A compact, reliable firmware for the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) that reads RC channel data from an **ExpressLRS (ELRS)** receiver over the CRSF serial protocol and drives **9 servos** through a multi-step, timed sequence — triggered by a designated switch channel.

A **hardware safety interlock on Channel 6** must be armed before any servo movement can be triggered.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Required](#2-hardware-required)
3. [Wiring / Pin Connections](#3-wiring--pin-connections)
4. [Code Explanation](#4-code-explanation)
   - [Libraries Used](#41-libraries-used)
   - [CRSF Parsing](#42-crsf-parsing)
   - [Servo Control Logic](#43-servo-control-logic)
   - [Sequence Steps](#44-sequence-steps)
   - [LED Feedback](#45-led-feedback)
5. [Safety Channel (CH6 Interlock)](#5-safety-channel-ch6-interlock)
6. [Setup & Upload Instructions](#6-setup--upload-instructions)
7. [Configuration Options](#7-configuration-options)
8. [Serial Debug Output](#8-serial-debug-output)
9. [Troubleshooting](#9-troubleshooting)
10. [License](#10-license)

---

## 1. Project Overview

This project was built for Project 399 to remotely actuate a mechanical servo-driven mechanism via a standard RC transmitter running ExpressLRS.

**What it does:**

- Receives CRSF frames from an ELRS receiver at 420 000 baud over UART1.
- Monitors a **safety channel (CH6 / AUX2)**. Servo movement is only permitted when CH6 is in the ARMED position (above `SAFETY_THRESHOLD`). When CH6 is SAFE, the trigger channel is completely ignored.
- Monitors a **trigger channel (CH5 / AUX1)** — active only when the safety is ARMED.
- When the trigger switch crosses a threshold (ON): executes a **deploy sequence** — moving each of the 9 servos to their target positions in a defined order with configurable hold times between steps.
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
    uint16_t ch6 = crsf.getChannel(6);  // safety channel
}
```

Channel values are in the CRSF native range **172–1811** (midpoint ≈ 992). Use `CrsfParser::toServoUs(val)` to map directly to 1000–2000 µs for servo control.

### 4.3 Servo Control Logic

Servos are driven by the standard Teensyduino `Servo` library using `writeMicroseconds()` for precise, sub-millisecond resolution.

The firmware uses a **non-blocking state machine** with four states:

| State | Description |
|-------|-------------|
| `STATE_IDLE` | Waiting for switch ON (rising edge) — only reachable when system is ARMED |
| `STATE_DEPLOYING` | Executing deploy sequence step-by-step |
| `STATE_DEPLOYED` | Sequence complete — waiting for switch OFF |
| `STATE_RETRACTING` | Executing retract sequence step-by-step |

**Edge detection** — the trigger fires on a _rising_ edge (OFF→ON) for deploy and _falling_ edge (ON→OFF) for retract, preventing re-triggering while the switch is held.

**Non-blocking timing** — `millis()` is used instead of `delay()`, so the CRSF parser continues to receive bytes even during sequence execution.

**Safety interlock** — before any edge is detected in `STATE_IDLE`, the firmware checks `systemArmed`. If the safety channel (CH6) is in the SAFE position, `switchOn` is forced to `false` and `lastSwitch` is cleared, so no rising edge can be generated regardless of CH5's physical position.

### 4.5 LED Feedback

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

### 4.4 Sequence Steps

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

## 5. Safety Channel (CH6 Interlock)

### Overview

Channel 6 (AUX2) acts as a **hardware safety interlock**. The system has two states:

| CH6 Value | State | Effect |
|-----------|-------|--------|
| ≤ `SAFETY_THRESHOLD` (default 1500) | **SAFE** | CH5 is completely ignored. No servo movement can be triggered. |
| > `SAFETY_THRESHOLD` (default 1500) | **ARMED** | CH5 operates normally — trigger switch controls the deploy/retract sequence. |

### Wiring the Safety Switch

Assign a dedicated switch to **AUX2 (CH6)** on your RC transmitter. A **2-position switch** (e.g. SA or SF on EdgeTX/OpenTX) is recommended for clear SAFE/ARMED distinction.

Typical transmitter output:
- **Down / OFF position** → ~1000 µs → **SAFE** (servos locked)
- **Up / ON position** → ~2000 µs → **ARMED** (servos active)

### Arming Sequence

For safe operation, always follow this arming order:

1. **Power on** the Teensy and receiver.
2. **Verify** CH6 is in the SAFE position (down). The system boots SAFE regardless.
3. **Verify** CH5 is in the OFF/down position.
4. **Arm** — flip CH6 to the ARMED position. Serial monitor prints `Safety: ARMED`.
5. **Operate** — CH5 now controls the servo sequence normally.

### Disarming

- Flip CH6 back to SAFE at any time. The system immediately blocks new sequences.
- If a sequence is currently running when you disarm, it **completes naturally** (the mechanism is allowed to finish its movement for safety). The system will not accept a new trigger until re-armed.

### Configuring the Threshold

Edit `SAFETY_THRESHOLD` in `TeensyServoControl.ino`:

```cpp
// In the CONFIGURATION block at the top of the sketch:
#define SAFETY_CHANNEL   6      // CRSF channel for safety switch (1-indexed)
#define SAFETY_THRESHOLD 1500   // channel value that must be exceeded to ARM
```

| Switch Type | Recommended `SAFETY_THRESHOLD` |
|-------------|-------------------------------|
| 2-position switch | `1200` (arms at top position ~2000 µs) |
| 3-position switch (arm only at full-up) | `1600` (ignores mid position ~1500 µs) |

CRSF channel range is **172–1811** (midpoint ≈ 992). The default threshold of **1500** safely arms only when the switch is close to its full-up position (~2000 µs) and disarms when in any lower position.

### Serial Debug Output for Safety

When the safety state changes, a log line is printed:

```
CH6: 1811  Safety: ARMED
CH6: 172   Safety: SAFE
```

---

## 6. Setup & Upload Instructions

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
TeensyServoControl v4.0 — ready
Servos on pins: 3, 4, 5, 6, 9, 10, 20, 21, 22
Trigger channel: CH5
Safety channel:  CH6  threshold: 1500
LED pin: 13
LED modes: solid=powered, blink=signal, double-blink=servo active
Safety: SAFE (waiting for CH6 ARM)
```

Once the receiver is bound and outputting:

```
CH6: 172   Safety: SAFE          ← CH6 in down/safe position
CH6: 1811  Safety: ARMED         ← CH6 flipped to arm
CH5: 172   Switch: OFF
CH5: 1811  Switch: ON
→ DEPLOYING
→ DEPLOYED
CH5: 172   Switch: OFF
→ RETRACTING
→ IDLE
```

**Expected LED behavior:**
- **On first power-up (no receiver connected):** LED ON solid.
- **After connecting ELRS receiver (bound and outputting):** LED starts continuous blinking (200 ms / 200 ms).
- **When you flip the trigger switch (CH5, with CH6 armed):** LED switches to double-blink pattern for the duration of the servo sequence, then reverts to continuous blink.

---

## 7. Configuration Options

All user-configurable options are at the top of `TeensyServoControl.ino`:

| Constant | Default | Description |
|----------|---------|-------------|
| `CRSF_SERIAL` | `Serial1` | UART connected to ELRS receiver |
| `CRSF_BAUD` | `420000` | CRSF baud rate (do not change) |
| `TRIGGER_CHANNEL` | `5` | RC channel that triggers the sequence (1–16). Default: CH5 / AUX1 |
| `SWITCH_THRESHOLD` | `1200` | Channel value above this = trigger switch ON (range 172–1811) |
| `SAFETY_CHANNEL` | `6` | RC channel for the safety interlock (1–16). Default: CH6 / AUX2 |
| `SAFETY_THRESHOLD` | `1500` | Channel value above this = system ARMED (range 172–1811) |
| `SERVO_PINS[]` | `{3, 4, 5, 6, 9, 10, 20, 21, 22}` | Teensy pin numbers for each of the 9 servos |
| `SERVO_MIN_US` | `1000` | Servo minimum pulse width (µs) |
| `SERVO_MAX_US` | `2000` | Servo maximum pulse width (µs) |
| `SERVO_MID_US` | `1500` | Servo centre/mid pulse width (µs) |
| `SERVO_DEADBAND_US` | `10` | Ignore commanded position changes smaller than this (µs); prevents jitter from tiny CRSF value fluctuations |
| `DEPLOY_SEQUENCE[]` | See code | Steps to run when switch goes ON |
| `RETRACT_SEQUENCE[]` | See code | Steps to run when switch goes OFF |
| `LED_PIN` | `LED_BUILTIN` (pin 13) | Onboard LED pin — do not change for Teensy 3.2 |
| `LED_SIGNAL_ON_MS` | `200` | Continuous blink ON duration (signal present) |
| `LED_SIGNAL_OFF_MS` | `200` | Continuous blink OFF duration (signal present) |
| `LED_SERVO_PULSE_MS` | `100` | Double-blink each pulse duration (servo active) |
| `LED_SERVO_GAP_MS` | `300` | Double-blink gap after second pulse (servo active) |
| `CRSF_SIGNAL_TIMEOUT_MS` | `500` | ms without a CRSF frame before signal is marked lost |

### Default Channel Mapping

| CRSF Channel | Function | Transmitter Switch | Active When |
|-------------|----------|-------------------|------------|
| **CH5 (AUX1)** | Trigger switch — starts/stops servo sequence | SA (2-pos) or any AUX switch | > 1200 |
| **CH6 (AUX2)** | Safety interlock — must be ARMED first | SB / SF (dedicated safety switch) | > 1500 |

### Changing the trigger channel

Map `TRIGGER_CHANNEL` to any AUX switch on your transmitter. Typical EdgeTX/OpenTX mapping:

| Channel | Label | Common Use |
|---------|-------|-----------|
| 5 | AUX1 | SA / 2-pos switch |
| 6 | AUX2 | SB / 3-pos switch |
| 7 | AUX3 | SC |
| 8 | AUX4 | SD |

> **Note:** `TRIGGER_CHANNEL` and `SAFETY_CHANNEL` must be different channels. Do not assign the same channel to both functions.

### Adjusting the threshold

- For a **2-position switch**: set `SWITCH_THRESHOLD = 1200` (detects high position).
- For a **3-position switch**: set `SWITCH_THRESHOLD = 1600` (only fires at top position).
- For the safety channel with a **3-position switch**: set `SAFETY_THRESHOLD = 1600` to require full-up to arm.

### Adding / removing servos

Edit `SERVO_PINS[]`:

```cpp
static const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };  // 9 servos
```

`NUM_SERVOS` is computed automatically via `sizeof`. Then update `DEPLOY_SEQUENCE` and
`RETRACT_SEQUENCE` to match your new servo count. Additional hardware PWM-capable pins
on the Teensy 3.2: `23`, `25`, `32`.

---

## 8. Serial Debug Output

With USB connected and Serial Monitor open at **115200 baud**:

```
TeensyServoControl v4.0 — ready
Servos on pins: 3, 4, 5, 6, 9, 10, 20, 21, 22
Trigger channel: CH5
Safety channel:  CH6  threshold: 1500
LED pin: 13
LED modes: solid=powered, blink=signal, double-blink=servo active
Safety: SAFE (waiting for CH6 ARM)
CH6: 172   Safety: SAFE          ← receiver bound, CH6 still down
CH6: 1811  Safety: ARMED         ← user flipped safety switch
CH5: 172   Switch: OFF           ← polling every 200 ms (non-blocking)
CH5: 1811  Switch: ON
→ DEPLOYING
→ DEPLOYED
CH5: 172   Switch: OFF
→ RETRACTING
→ IDLE
CH6: 172   Safety: SAFE          ← safety flipped back; no new sequences
```

To reduce serial overhead in production, comment out the debug blocks inside the `hasNewFrame()` branch in `loop()`.

---

## 9. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No channel data / `CH5: 0` | Wrong UART RX pin or baud rate | Check wiring to Pin 0; verify CRSF output mode on receiver |
| Servos don't move | Wrong servo pins or no external power | Confirm `SERVO_PINS[]`, check BEC voltage at servo connector |
| Servos don't move even with CH5 ON | CH6 is in SAFE position | Flip CH6 to ARMED position; confirm `Safety: ARMED` in Serial Monitor |
| Sequence fires immediately on boot | Receiver not bound / outputting zero | Add a startup delay or check receiver binding |
| Sequence runs twice | Switch debounce | Edge detection is already implemented; check for electrical noise on the switch line |
| Upload fails | Teensy not in bootloader mode | Press the button on the Teensy or check USB connection |
| `crsf_parser.h` not found | File not in same folder as `.ino` | Both files must be inside `TeensyServoControl/` folder |
| Servos jitter or draw excess current | All 9 servos from same BEC | Ensure BEC is rated for the combined stall current (≥9 A recommended) |
| LED stays solid, won't blink | No CRSF signal received | Check receiver wiring/binding; LED blinks only when frames arrive |
| LED blinks but double-blink never shows | Switch threshold not crossed, or safety SAFE | Verify `TRIGGER_CHANNEL`, `SWITCH_THRESHOLD`, and that `SAFETY_CHANNEL` is ARMED |
| Safety state never shows ARMED | `SAFETY_THRESHOLD` too high | Lower `SAFETY_THRESHOLD` or verify the transmitter switch output value via Serial Monitor |

---

## 10. License

MIT License — free to use, modify, and distribute.  
© 2026 Jordan Temkin / Project 399

---

## Changelog

### v4.0
- **Added CH6 safety interlock** — a dedicated hardware safety channel (default: CH6 / AUX2)
  - New constants `SAFETY_CHANNEL` (default `6`) and `SAFETY_THRESHOLD` (default `1500`)
  - New global `systemArmed` flag, updated every CRSF frame from CH6 value
  - When `systemArmed == false`: `switchOn` and `lastSwitch` are forced to `false`, preventing any rising-edge detection in `STATE_IDLE`
  - In-flight sequences (already running when safety is disarmed) complete naturally
  - Serial monitor prints `Safety: ARMED` / `Safety: SAFE` on every state change
- Updated startup message to print safety channel and threshold
- Bumped version string to `v4.0`
- Updated README with full [Safety Channel (CH6 Interlock)](#5-safety-channel-ch6-interlock) section, channel mapping table, arming sequence, and troubleshooting entries

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