# Teensy Servo Control

> **Teensy 3.2 · ExpressLRS (CRSF) input · Multi-servo sequencing · Switch-triggered**

A compact, reliable firmware for the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) that reads RC channel data from an **ExpressLRS (ELRS)** receiver over the CRSF serial protocol and drives up to four (configurable) servos through a multi-step, timed sequence — triggered by a designated switch channel.

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
- When the switch crosses a threshold (ON): executes a **deploy sequence** — moving each servo to its target position in a defined order with configurable hold times between steps.
- When the switch returns low (OFF): executes a **retract sequence** in reverse.
- State machine prevents mid-sequence interruption and ensures clean transitions.

**Why Teensy 3.2?**

- Hardware UART capable of 420 000 baud (required for CRSF).
- Plenty of PWM-capable pins for multiple servos.
- 72 MHz ARM Cortex-M4 — more than enough headroom for real-time control.
- Small form factor, 3.3 V I/O compatible with ELRS receivers.

---

## 2. Hardware Required

| Component | Notes |
|-----------|-------|
| Teensy 3.2 | PJRC — `teensy32` board in Teensyduino |
| ELRS receiver | e.g. BetaFPV Nano, Happymodel EP1/EP2, BETAFPV SuperD |
| Servos (×1–4) | Standard PWM servos, 1000–2000 µs pulse range |
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
│  Servo 0 Signal ─────────────────► Pin 3  (PWM)          │
│  Servo 1 Signal ─────────────────► Pin 4  (PWM)          │
│  Servo 2 Signal ─────────────────► Pin 5  (PWM)          │
│  Servo 3 Signal ─────────────────► Pin 6  (PWM)          │
│                                                           │
│  Servo Power (all) ──────────────── External 5V BEC      │
│  Servo GND (all) ────────────────── Shared GND           │
└─────────────────────────────────────────────────────────┘
```

> **⚠️ Power note:** Servos draw significant current (0.5–1 A each under load).  
> Always power them from a dedicated 5 V BEC — not the Teensy's onboard regulators.

> **⚠️ Receiver voltage:** Check your specific ELRS receiver datasheet.  
> Most nano receivers accept 3.3 V–5 V. When powering from the BEC (5 V), use the receiver's dedicated VCC pad, not Teensy's 3.3 V output.

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

The firmware uses a **non-blocking state machine** with four states:

| State | Description |
|-------|-------------|
| `STATE_IDLE` | Waiting for switch ON (rising edge) |
| `STATE_DEPLOYING` | Executing deploy sequence step-by-step |
| `STATE_DEPLOYED` | Sequence complete — waiting for switch OFF |
| `STATE_RETRACTING` | Executing retract sequence step-by-step |

**Edge detection** — the trigger fires on a _rising_ edge (OFF→ON) for deploy and _falling_ edge (ON→OFF) for retract, preventing re-triggering while the switch is held.

**Non-blocking timing** — `millis()` is used instead of `delay()`, so the CRSF parser continues to receive bytes even during sequence execution.

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
Step 1: Servo 0 → 2000 µs (full extend)  | hold 300 ms
Step 2: Servo 1 → 2000 µs               | hold 300 ms
Step 3: Servo 2 → 2000 µs               | hold   0 ms  ← fires simultaneously with step 4
Step 4: Servo 3 → 2000 µs               | hold 500 ms
Step 5: Servo 0 → 1500 µs (latch/mid)   | hold 200 ms
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
TeensyServoControl v1.0 — ready
Servos on pins: 3, 4, 5, 6
Trigger channel: CH5
CH5: 172  Switch: OFF
CH5: 172  Switch: OFF
…
```

---

## 6. Configuration Options

All user-configurable options are at the top of `TeensyServoControl.ino`:

| Constant | Default | Description |
|----------|---------|-------------|
| `CRSF_SERIAL` | `Serial1` | UART connected to ELRS receiver |
| `CRSF_BAUD` | `420000` | CRSF baud rate (do not change) |
| `TRIGGER_CHANNEL` | `5` | RC channel number that triggers the sequence (1–16) |
| `SWITCH_THRESHOLD` | `1200` | Channel value above this = switch ON (range 172–1811) |
| `SERVO_PINS[]` | `{3, 4, 5, 6}` | Teensy pin numbers for each servo |
| `SERVO_MIN_US` | `1000` | Servo minimum pulse width (µs) |
| `SERVO_MAX_US` | `2000` | Servo maximum pulse width (µs) |
| `SERVO_MID_US` | `1500` | Servo centre/mid pulse width (µs) |
| `DEPLOY_SEQUENCE[]` | See code | Steps to run when switch goes ON |
| `RETRACT_SEQUENCE[]` | See code | Steps to run when switch goes OFF |

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
static const uint8_t SERVO_PINS[] = { 3, 4 };   // Two servos only
```

`NUM_SERVOS` is computed automatically via `sizeof`.

Then update `DEPLOY_SEQUENCE` and `RETRACT_SEQUENCE` to match your new servo count.

---

## 7. Serial Debug Output

With USB connected and Serial Monitor open at **115200 baud**:

```
TeensyServoControl v1.0 — ready
Servos on pins: 3, 4, 5, 6
Trigger channel: CH5
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

---

## 9. License

MIT License — free to use, modify, and distribute.  
© 2026 Jordan Temkin / Project 399

---

*Built with Teensyduino · ExpressLRS · Project 399*
