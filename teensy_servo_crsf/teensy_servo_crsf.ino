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
 *     SERVO_START_US[] – servo position (µs) BEFORE trigger
 *     SERVO_END_US[]   – servo position (µs) AFTER trigger
 *
 * Behavior:
 *   Every time the switch transitions LOW→HIGH, the next servo in the
 *   sequence (0-8) moves from its START position to its END position.
 *   The index wraps: after servo 8, it resets to servo 0.
 *   Switching back HIGH→LOW does NOT reverse the move; a second toggle
 *   advances to the next servo.
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

// Position each servo sits at on startup / before it's triggered
const uint16_t SERVO_START_US[NUM_SERVOS] = {
  1000,  // Servo 1 start
  1000,  // Servo 2 start
  1000,  // Servo 3 start
  1000,  // Servo 4 start
  1000,  // Servo 5 start
  1000,  // Servo 6 start
  1000,  // Servo 7 start
  1000,  // Servo 8 start
  1000   // Servo 9 start
};

// Position each servo moves TO when triggered
const uint16_t SERVO_END_US[NUM_SERVOS] = {
  2000,  // Servo 1 end
  2000,  // Servo 2 end
  2000,  // Servo 3 end
  2000,  // Servo 4 end
  2000,  // Servo 5 end
  2000,  // Servo 6 end
  2000,  // Servo 7 end
  2000,  // Servo 8 end
  2000   // Servo 9 end
};

// ─────────────────────────────────────────────────────────────────────────────
//  END CONFIGURATION — no changes needed below unless you know what you're doing
// ─────────────────────────────────────────────────────────────────────────────

// CRSF runs at 420 000 baud over Serial1 (pins 0 = RX1, 1 = TX1)
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

Servo servos[NUM_SERVOS];

// Track which servo fires next (0-based index into SERVO_PINS[])
uint8_t  sequenceIndex = 0;

// Debounce / edge-detection state for the switch channel
bool     switchWasHigh = false;

// ── Forward declarations ─────────────────────────────────────────────────────
void onChannelPacket();
void triggerNextServo();


// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
  // USB serial — for debug output on the Serial Monitor (115 200 baud)
  Serial.begin(115200);
  Serial.println(F("Teensy 3.2  |  9-Servo CRSF Sequencer  |  Booting..."));

  // Attach all servos and move them to their start positions
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].writeMicroseconds(SERVO_START_US[i]);
    Serial.print(F("  Servo "));
    Serial.print(i + 1);
    Serial.print(F(" → pin "));
    Serial.print(SERVO_PINS[i]);
    Serial.print(F("  start="));
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
  Serial.println(F("Ready."));
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
 * Reads the configured switch channel and fires triggerNextServo() on
 * every rising edge (LOW → HIGH transition).
 */
void onChannelPacket()
{
  // getChannel() returns a 16-bit value in the CRSF range 172–1811
  uint16_t switchVal = crsf.getChannel(SWITCH_CHANNEL);
  bool switchIsHigh  = (switchVal > SWITCH_MID);

  // Detect rising edge only (LOW → HIGH)
  if (switchIsHigh && !switchWasHigh) {
    triggerNextServo();
  }

  switchWasHigh = switchIsHigh;
}


// ─────────────────────────────────────────────────────────────────────────────
/**
 * triggerNextServo()
 * Advances the sequence index and moves the next servo to its end position.
 * After the last servo (index 8), the index wraps back to 0.
 */
void triggerNextServo()
{
  uint8_t idx = sequenceIndex;

  Serial.print(F("Trigger → Servo "));
  Serial.print(idx + 1);
  Serial.print(F("  (pin "));
  Serial.print(SERVO_PINS[idx]);
  Serial.print(F(")  → "));
  Serial.print(SERVO_END_US[idx]);
  Serial.println(F(" µs"));

  servos[idx].writeMicroseconds(SERVO_END_US[idx]);

  // Advance and wrap the index
  sequenceIndex = (sequenceIndex + 1) % NUM_SERVOS;
}
