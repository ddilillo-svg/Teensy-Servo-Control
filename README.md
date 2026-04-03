# Teensy 3.2 — 9-Servo CRSF Sequencer

Control 9 servos in sequence, one per switch toggle, using an ExpressLRS (ELRS) receiver over the CRSF serial protocol.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Teensy 3.2 |
| Receiver | Any ELRS receiver with CRSF UART output |
| Servos | 9× standard RC servo (5V, 50 Hz PWM) |

### Wiring

#### ELRS Receiver → Teensy 3.2

```
Receiver TX  ──►  Teensy pin 0  (RX1)   ← CRSF data in
Receiver RX  ──►  Teensy pin 1  (TX1)   ← telemetry out (optional)
Receiver GND ──►  Teensy GND
Receiver 5V  ──►  Teensy VIN / external 5V
```

> ⚠️ Many ELRS receivers run on **5 V** but output 3.3 V logic — this is
> compatible with Teensy 3.2 (which also runs at 3.3 V). Double-check your
> specific receiver's datasheet.

#### Servos → Teensy 3.2

| Servo # | Teensy Pin | Notes |
|---------|-----------|-------|
| 1 | 3 | Hardware PWM (FTM2) |
| 2 | 4 | Hardware PWM (FTM1) |
| 3 | 5 | Hardware PWM (FTM0) |
| 4 | 6 | Hardware PWM (FTM0) |
| 5 | 9 | Hardware PWM (FTM0) |
| 6 | 10 | Hardware PWM (FTM0) |
| 7 | 20 | Hardware PWM (FTM0) |
| 8 | 21 | Hardware PWM (FTM0) |
| 9 | 22 | Hardware PWM (FTM0) |

> Power the servos from a dedicated **5 V BEC / regulator** — do not power 9
> servos from the Teensy's 3.3 V or the USB 5 V rail, which cannot handle
> the combined stall current.

---

## Software Dependencies

Install both libraries before compiling:

| Library | Source | How to install |
|---------|--------|---------------|
| **CrsfSerial** (CapnBry) | https://github.com/CapnBry/CRServoF | Arduino Library Manager → search *CrsfSerial* |
| **Servo** | Bundled with Teensyduino | Already installed with Teensyduino |

---

## Configuration

All user-tunable values are at the top of `teensy_servo_crsf.ino` inside the
**CONFIGURATION SECTION**.

### Switch channel

```cpp
#define SWITCH_CHANNEL  5   // which CRSF channel carries the trigger switch
```

Change to whichever channel your transmitter's switch is assigned to (1–16).

### Threshold

```cpp
#define SWITCH_MID  992   // values above this = switch HIGH
```

CRSF raw range is **172** (min) → **992** (center) → **1811** (max).
Adjust if your switch sits at a non-standard position.

### Servo pins

```cpp
const uint8_t SERVO_PINS[NUM_SERVOS] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };
```

Change any pin to any free Teensy 3.2 digital pin.

### Servo positions

```cpp
const uint16_t SERVO_START_US[NUM_SERVOS] = { 1000, 1000, ... };  // rest position
const uint16_t SERVO_END_US[NUM_SERVOS]   = { 2000, 2000, ... };  // triggered position
```

Values are in **microseconds**. Standard range: `1000` (full CCW/down) –
`1500` (center) – `2000` (full CW/up). Tune per-servo to match your
mechanical setup.

---

## Behavior

1. On power-up all servos move to their `SERVO_START_US` positions.
2. Each **rising edge** of the configured switch channel (LOW → HIGH) fires
   the next servo in sequence.
3. Sequence order: Servo 1 → Servo 2 → … → Servo 9 → Servo 1 → …
4. Triggered servos move to their `SERVO_END_US` position and **hold there**.
5. Switching back (HIGH → LOW) does **not** move any servo — only the next
   rising edge does.
6. Debug output streams over USB serial at 115 200 baud (open the Serial
   Monitor in the Arduino IDE).

---

## Building

### Arduino IDE

1. Install **Teensyduino** from https://www.pjrc.com/teensy/teensyduino.html
2. Install **CrsfSerial** via Library Manager
3. Set **Board → Teensy 3.2**
4. Open `teensy_servo_crsf/teensy_servo_crsf.ino` and click Upload

### PlatformIO

Add to `platformio.ini`:

```ini
[env:teensy31]
platform  = teensy
board     = teensy31
framework = arduino
lib_deps  =
    CapnBry/CrsfSerial
```

*(Teensy 3.2 uses the same `teensy31` board target in PlatformIO)*

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| No CRSF data received | Verify RX1/TX1 wiring; confirm receiver outputs CRSF (not SBUS/PPM) |
| Servos jitter | Separate servo power supply — do not power from USB |
| Wrong channel triggers | Change `SWITCH_CHANNEL` to match your transmitter mapping |
| Sequence skips servos | Switch is bouncing — increase `SWITCH_MID` deadband or add software debounce |

---

## License

MIT — free to use and modify.

## Author

Jordan Temkin \<399project@gmail.com\>
