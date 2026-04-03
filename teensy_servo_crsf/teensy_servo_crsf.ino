/**
 * teensy_servo_crsf.ino
 *
 * Teensy 3.2 — 9-Servo Sequencer driven by an ELRS (CRSF) receiver
 *
 * Hardware:
 *   - Teensy 3.2
 *   - ELRS receiver wired to:
 *       TX1 (Teensy pin 1) → RX on receiver (telemetry back-channel, optional)
 *       RX1 (Teensy pin 0) → TX on receiver  ← primary data line
 *   - 9 servos on SERVO_PINS[] below
 *
 * Dependencies (install via Arduino Library Manager or PlatformIO):
 *   - CrsfSerial  by CapnBry  (search "CrsfSerial" in Library Manager)
 *   - Servo       — bundled with Teensyduino / Arduino IDE
 *
 * Configuration:
 *   Edit the CONFIGURATION SECTION below to set:
 *     SWITCH_CHANNEL   – which CRSF channel carries your switch (1-16)
 *     SWITCH_MID       – mid-point threshold between low/high (CRSF range ≈ 172-1811)
 *     SERVO_PINS[]     – output pin for each of the 9 servos
 *     SERVO_START_US[] – servo position (µs) when closed/default (start & reset)
 *     SERVO_END_US[]   – servo position (µs) when open/triggered
 *
 * Behavior — Strict Sequential Mode:
 *   Flip 1  → Servo 1 moves to its END (open) position
 *   Flip 2  → Servo 2 moves to its END (open) position
 *   ...
 *   Flip 9  → Servo 9 moves to its END (open) position
 *   Flip 10 → ALL 9 servos return to their START (closed/default) position
 *             and the sequence resets to Servo 1.
 *
 *   Only ONE servo moves per switch flip. Switching back HIGH→LOW does NOT
 *   trigger any action; only the next LOW→HIGH rising edge does.
 *
 * Author : Jordan Temkin <399project@gmail.com>
 * Board  : Teensy 3.2 (select in Arduino IDE: Tools → Board → Teensy 3.2)
 */

#include <CrsfSerial.h>   // CapnBry/CRServoF  —  https://github.com/CapnBry/CRServoF
#include <Servo.h>         // Teensyduino / Arduino built-in

// ─────────────────────────────────────────────────────────────────────────────
//  CONFIGURATION SECTION  ← Edit these values to match your setup
// ─────────────────────────────────────────────────────────────────────────────

// Which CRSF RC channel carries the trigger switch? (1-16)
#define SWITCH_CHANNEL  5

// CRSF channel value midpoint.  Values above this = switch HIGH.
// CRSF raw range:  172 (min)  …  992 (center)  …  1811 (max)
#define SWITCH_MID      992

// Number of servos in the sequence
#define NUM_SERVOS      9

// ── Pin assignments ──────────────────────────────────────────────────────────
// Teensy 3.2 hardware PWM-capable pins:
//   3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32
// Servo.h works on any digital output but sounds better on PWM pins.
// Pins 0 and 1 are reserved for CRSF serial (RX1/TX1).
const uint8_t SERVO_PINS[NUM_SERVOS] = {
  3,   // Servo 1
  4,   // Servo 2
  5,   // Servo 3
  6,   // Servo 4
  9,   // Servo 5
  10,  // Servo 6
  20,  // Servo 7
  21,  // Servo 8
  22   // Servo 9
};

// ── Servo positions (microseconds) ──────────────────────────────────────────
// Standard range: 1000 µs (full left/down) … 1500 µs (center) … 2000 µs (full right/up)
// Adjust per servo mechanical travel.

// Closed / default position — where each servo sits on startup and after the
// "all close" reset (10th flip).
const uint16_t SERVO_START_US[NUM_SERVOS] = {
  1000,  // Servo 1 closed
  1000,  // Servo 2 closed
  1000,  // Servo 3 closed
  1000,  // Servo 4 closed
  1000,  // Servo 5 closed
  1000,  // Servo 6 closed
  1000,  // Servo 7 closed
  1000,  // Servo 8 closed
  1000   // Servo 9 closed
};

// Open / triggered position — where each servo moves when it is activated.
const uint16_t SERVO_END_US[NUM_SERVOS] = {
  2000,  // Servo 1 open
  2000,  // Servo 2 open
  2000,  // Servo 3 open
  2000,  // Servo 4 open
  2000,  // Servo 5 open
  2000,  // Servo 6 open
  2000,  // Servo 7 open
  2000,  // Servo 8 open
  2000   // Servo 9 open
};

// ─────────────────────────────────────────────────────────────────────────────
//  END CONFIGURATION — no changes needed below unless you know what you're doing
// ─────────────────────────────────────────────────────────────────────────────

// CRSF runs at 420 000 baud over Serial1 (pins 0 = RX1, 1 = TX1)
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

Servo servos[NUM_SERVOS];

// ── Sequence state ───────────────────────────────────────────────────────────
// flipCount tracks how many LOW→HIGH rising edges have occurred this cycle.
//   flipCount 1-9  → activate servo (flipCount - 1)
//   flipCount 10   → close all servos and reset to 0
uint8_t flipCount = 0;

// Debounce / edge-detection state for the switch channel
bool switchWasHigh = false;

// ── Forward declarations ─────────────────────────────────────────────────────
void onChannelPacket();
void activateServo(uint8_t idx);
void closeAllServos();


// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
  // USB serial — for debug output on the Serial Monitor (115 200 baud)
  Serial.begin(115200);
  Serial.println(F("Teensy 3.2  |  9-Servo CRSF Sequencer  |  Booting..."));

  // Attach all servos and move them to their closed/start positions
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(SERVO_START_US[i]);
    Serial.print(F("  Servo "));
    Serial.print(i + 1);
    Serial.print(F(" → pin "));
    Serial.print(SERVO_PINS[i]);
    Serial.print(F("  closed="));
    Serial.print(SERVO_START_US[i]);
    Serial.println(F(" µs"));
  }

  // Initialise CRSF receiver on Serial1
  crsf.begin();
  crsf.onPacketChannels = &onChannelPacket;   // register callback

  Serial.print(F("Listening for CRSF on Serial1 (pin 0=RX1, pin 1=TX1) at "));
  Serial.print(CRSF_BAUDRATE);
  Serial.println(F(" baud"));
  Serial.print(F("Trigger channel: "));
  Serial.println(SWITCH_CHANNEL);
  Serial.println(F("Ready. Waiting for switch flips..."));
  Serial.println(F("  Flips 1-9 : activate Servo 1 through 9 sequentially"));
  Serial.println(F("  Flip 10   : close all servos and reset sequence"));
}


// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
  // Must be called every iteration — processes incoming CRSF bytes
  crsf.loop();

  // All logic is handled inside the onChannelPacket() callback;
  // add any other non-blocking housekeeping here if needed.
}


// ─────────────────────────────────────────────────────────────────────────────
/**
 * onChannelPacket()
 * Called by CrsfSerial each time a full CRSF channel frame is decoded
 * (typically ~150 Hz with ELRS in 150 Hz mode, or ~50 Hz in 50 Hz mode).
 *
 * Reads the configured switch channel and advances the sequence on every
 * rising edge (LOW → HIGH transition).
 *
 * Sequence rules:
 *   Flip 1  → Servo 1 activates
 *   Flip 2  → Servo 2 activates
 *   ...
 *   Flip 9  → Servo 9 activates
 *   Flip 10 → All servos close, sequence resets to Flip 1
 */
void onChannelPacket()
{
  // getChannel() returns a 16-bit value in the CRSF range 172–1811
  uint16_t switchVal = crsf.getChannel(SWITCH_CHANNEL);
  bool switchIsHigh  = (switchVal > SWITCH_MID);

  // Detect rising edge only (LOW → HIGH)
  if (switchIsHigh && !switchWasHigh) {
    flipCount++;

    if (flipCount <= NUM_SERVOS) {
      // Flips 1-9: activate the corresponding servo (strictly one at a time)
      activateServo(flipCount - 1);   // convert 1-based flip to 0-based index
    } else {
      // Flip 10: close all servos and reset the cycle
      closeAllServos();
      flipCount = 0;
    }
  }

  switchWasHigh = switchIsHigh;
}


// ─────────────────────────────────────────────────────────────────────────────
/**
 * activateServo(idx)
 * Moves a single servo (by 0-based index) to its open/triggered END position.
 * No other servo is touched.
 */
void activateServo(uint8_t idx)
{
  Serial.print(F("Flip "));
  Serial.print(flipCount);
  Serial.print(F(" → Servo "));
  Serial.print(idx + 1);
  Serial.print(F("  (pin "));
  Serial.print(SERVO_PINS[idx]);
  Serial.print(F(")  OPEN → "));
  Serial.print(SERVO_END_US[idx]);
  Serial.println(F(" µs"));

  servos[idx].writeMicroseconds(SERVO_END_US[idx]);
}


// ─────────────────────────────────────────────────────────────────────────────
/**
 * closeAllServos()
 * Returns ALL 9 servos to their closed/default (START) positions.
 * Called on the 10th switch flip. Resets the sequence back to Servo 1.
 */
void closeAllServos()
{
  Serial.println(F("Flip 10 → CLOSE ALL servos — sequence reset"));

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(SERVO_START_US[i]);
    Serial.print(F("  Servo "));
    Serial.print(i + 1);
    Serial.print(F(" CLOSED → "));
    Serial.print(SERVO_START_US[i]);
    Serial.println(F(" µs"));
  }

  Serial.println(F("All servos closed. Ready for next cycle (Flip 1 = Servo 1)."));
}
