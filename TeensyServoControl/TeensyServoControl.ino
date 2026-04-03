/**
 * TeensyServoControl.ino
 * 
 * Teensy 3.2 — CRSF (ExpressLRS) Input → 9-Servo Sequencing Controller
 * 
 * Author:  Jordan Temkin <399project@gmail.com>
 * Project: Project 399
 * 
 * Overview:
 *   Reads CRSF (Crossfire Serial Protocol) data from an ExpressLRS (ELRS) receiver
 *   connected to Teensy 3.2 UART1.  When a designated switch channel crosses a
 *   configurable threshold the firmware runs a multi-step servo sequence, then
 *   reverses it when the switch is released.
 * 
 * Hardware:
 *   - Teensy 3.2
 *   - ELRS receiver (e.g. BetaFPV ELRS Nano / Happymodel EP1/EP2)
 *   - 9 servos wired to SERVO_PINS[]
 * 
 * Wiring:
 *   ELRS RX TX-pin  →  Teensy Pin 0  (Serial1 RX)
 *   ELRS RX RX-pin  →  Teensy Pin 1  (Serial1 TX)  [optional, telemetry]
 *   ELRS RX GND     →  Teensy GND
 *   ELRS RX VCC     →  Teensy 3.3 V or 5 V (check your receiver datasheet)
 *   Servo signal    →  See SERVO_PINS[] below
 *   Servo power     →  External 5 V BEC (do NOT power servos from Teensy 3V3)
 *   Servo GND       →  Shared GND with Teensy
 */

#include <Servo.h>
#include "crsf_parser.h"

// ─────────────────────────────────────────────────────────────────────────────
//  CONFIGURATION  (edit these to match your build)
// ─────────────────────────────────────────────────────────────────────────────

// Serial port connected to the ELRS receiver (UART1 on Teensy 3.2)
#define CRSF_SERIAL      Serial1
#define CRSF_BAUD        420000          // standard CRSF baud rate

// CRSF channel that acts as the trigger switch (1-indexed, 1 = CH1)
// Typical RC mapping: CH5 = AUX1 (first aux switch)
#define TRIGGER_CHANNEL  5

// Switch threshold: channel value above this = "switch ON"
// CRSF channel range is 172–1811, midpoint ≈ 992
#define SWITCH_THRESHOLD 1200

// Servo output pins on Teensy 3.2
// Hardware PWM-capable pins: 3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32
// Pins 0 and 1 are reserved for CRSF serial (RX1/TX1).
// Add or remove pins as needed; NUM_SERVOS is derived automatically.
static const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };
#define NUM_SERVOS  (sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]))

// Servo travel limits (microseconds — safe universal range)
#define SERVO_MIN_US   1000
#define SERVO_MAX_US   2000
#define SERVO_MID_US   1500

// Sequence step definition
struct SequenceStep {
  uint8_t  servoIndex;   // index into SERVO_PINS[]
  uint16_t targetUs;     // target pulse width in µs
  uint16_t holdMs;       // milliseconds to hold before next step
};

// ── DEPLOY SEQUENCE (switch ON) ───────────────────────────────────────────────
// Steps execute top-to-bottom when the trigger switch is activated.
// Customize these steps for your mechanism.
static const SequenceStep DEPLOY_SEQUENCE[] = {
  // Step 1: move servo 0 to full extend
  { 0, SERVO_MAX_US, 300 },
  // Step 2: move servo 1 to full extend
  { 1, SERVO_MAX_US, 300 },
  // Step 3: move servos 2 & 3 simultaneously (fire both at once via back-to-back steps, 0 ms gap)
  { 2, SERVO_MAX_US,   0 },
  { 3, SERVO_MAX_US, 300 },
  // Step 4: move servo 4 to full extend
  { 4, SERVO_MAX_US, 300 },
  // Step 5: move servo 5 to full extend
  { 5, SERVO_MAX_US, 300 },
  // Step 6: move servos 6 & 7 simultaneously
  { 6, SERVO_MAX_US,   0 },
  { 7, SERVO_MAX_US, 300 },
  // Step 7: move servo 8 to full extend
  { 8, SERVO_MAX_US, 500 },
  // Step 8: return servo 0 to mid as a latch
  { 0, SERVO_MID_US, 200 },
};
#define DEPLOY_STEPS  (sizeof(DEPLOY_SEQUENCE) / sizeof(DEPLOY_SEQUENCE[0]))

// ── RETRACT SEQUENCE (switch OFF) ─────────────────────────────────────────────
// Mirror / reverse of deploy, runs when the switch returns low.
static const SequenceStep RETRACT_SEQUENCE[] = {
  { 8, SERVO_MIN_US, 300 },
  { 7, SERVO_MIN_US, 300 },
  { 6, SERVO_MIN_US, 300 },
  { 5, SERVO_MIN_US, 300 },
  { 4, SERVO_MIN_US, 300 },
  { 3, SERVO_MIN_US, 300 },
  { 2, SERVO_MIN_US, 300 },
  { 0, SERVO_MIN_US,   0 },
  { 1, SERVO_MIN_US, 500 },
};
#define RETRACT_STEPS  (sizeof(RETRACT_SEQUENCE) / sizeof(RETRACT_SEQUENCE[0]))

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────────────

Servo    servos[NUM_SERVOS];
CrsfParser crsf;

// State machine
enum State {
  STATE_IDLE,
  STATE_DEPLOYING,
  STATE_DEPLOYED,
  STATE_RETRACTING
};

static State   currentState  = STATE_IDLE;
static uint8_t seqStep       = 0;
static uint32_t stepTimer    = 0;
static bool    lastSwitch    = false;

// ─────────────────────────────────────────────────────────────────────────────
//  HELPERS
// ─────────────────────────────────────────────────────────────────────────────

void attachServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  }
}

void moveServo(uint8_t idx, uint16_t us) {
  if (idx < NUM_SERVOS) {
    servos[idx].writeMicroseconds(us);
  }
}

// Drive all servos to a safe rest position
void resetServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(SERVO_MIN_US);
  }
}

// Run one step of a sequence; returns true when the full sequence is complete.
bool runSequence(const SequenceStep* seq, uint8_t numSteps) {
  if (seqStep >= numSteps) return true;   // already done

  uint32_t now = millis();

  if (seqStep == 0 || (now - stepTimer) >= seq[seqStep - 1].holdMs) {
    // Execute current step
    moveServo(seq[seqStep].servoIndex, seq[seqStep].targetUs);
    stepTimer = now;
    seqStep++;

    // If this step has zero hold time, immediately chain to next
    if (seq[seqStep - 1].holdMs == 0 && seqStep < numSteps) {
      moveServo(seq[seqStep].servoIndex, seq[seqStep].targetUs);
      stepTimer = now;
      seqStep++;
    }
  }

  return (seqStep >= numSteps && (now - stepTimer) >= seq[numSteps - 1].holdMs);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);           // USB debug serial
  CRSF_SERIAL.begin(CRSF_BAUD);  // ELRS receiver

  attachServos();
  resetServos();

  Serial.println(F("TeensyServoControl v2.0 — ready"));
  Serial.print(F("Servos on pins: "));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(SERVO_PINS[i]);
    if (i < NUM_SERVOS - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("Trigger channel: CH"));
  Serial.println(TRIGGER_CHANNEL);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  // ── Feed incoming bytes into the CRSF parser ──────────────────────────────
  while (CRSF_SERIAL.available()) {
    crsf.feed(CRSF_SERIAL.read());
  }

  // ── Read channel value once a new frame is available ──────────────────────
  bool switchOn = false;
  if (crsf.hasNewFrame()) {
    uint16_t chVal = crsf.getChannel(TRIGGER_CHANNEL);
    switchOn = (chVal > SWITCH_THRESHOLD);

    // Debug (comment out in production to reduce latency)
    static uint32_t dbgTimer = 0;
    if (millis() - dbgTimer > 200) {
      dbgTimer = millis();
      Serial.print(F("CH"));
      Serial.print(TRIGGER_CHANNEL);
      Serial.print(F(": "));
      Serial.print(chVal);
      Serial.print(F("  Switch: "));
      Serial.println(switchOn ? F("ON") : F("OFF"));
    }
  }

  // ── State machine ─────────────────────────────────────────────────────────
  switch (currentState) {

    case STATE_IDLE:
      if (switchOn && !lastSwitch) {
        // Rising edge — begin deploy sequence
        seqStep     = 0;
        currentState = STATE_DEPLOYING;
        Serial.println(F("→ DEPLOYING"));
      }
      break;

    case STATE_DEPLOYING:
      if (runSequence(DEPLOY_SEQUENCE, DEPLOY_STEPS)) {
        currentState = STATE_DEPLOYED;
        Serial.println(F("→ DEPLOYED"));
      }
      break;

    case STATE_DEPLOYED:
      if (!switchOn && lastSwitch) {
        // Falling edge — begin retract sequence
        seqStep     = 0;
        currentState = STATE_RETRACTING;
        Serial.println(F("→ RETRACTING"));
      }
      break;

    case STATE_RETRACTING:
      if (runSequence(RETRACT_SEQUENCE, RETRACT_STEPS)) {
        currentState = STATE_IDLE;
        Serial.println(F("→ IDLE"));
      }
      break;
  }

  lastSwitch = switchOn;
}
