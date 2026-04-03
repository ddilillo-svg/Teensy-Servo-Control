/**
 * TeensyServoControl.ino
 *
 * Teensy 3.2 — CRSF (ExpressLRS) Input → 9-Servo Sequential Controller
 *
 * Author:  Jordan Temkin <399project@gmail.com>
 * Project: Project 399
 *
 * Overview:
 *   Reads CRSF (Crossfire Serial Protocol) data from an ExpressLRS (ELRS) receiver
 *   connected to Teensy 3.2 UART1.  Each time the designated switch channel (CH5)
 *   crosses the configurable threshold (rising edge), the firmware advances to the
 *   next servo in the sequence.  Servos move STRICTLY ONE AT A TIME — no servo
 *   activates until the previous switch flip has been acknowledged.
 *
 *   A hardware safety switch on CH6 must be in the ARMED position before any
 *   servo movement can be triggered.  When CH6 is in the SAFE position the
 *   firmware ignores all CH5 input and no sequences can start.
 *
 *   Flip 1  → Servo 1 extends   (index 0)
 *   Flip 2  → Servo 2 extends   (index 1)
 *   Flip 3  → Servo 3 extends   (index 2)
 *   Flip 4  → Servo 4 extends   (index 3)
 *   Flip 5  → Servo 5 extends   (index 4)
 *   Flip 6  → Servo 6 extends   (index 5)
 *   Flip 7  → Servo 7 extends   (index 6)
 *   Flip 8  → Servo 8 extends   (index 7)
 *   Flip 9  → Servo 9 extends   (index 8)
 *   Flip 10 → All servos retract sequentially (9 → 1), counter resets to 0
 *
 *   After the "all close" retract sequence finishes the counter resets to 0 and
 *   the cycle can repeat from Flip 1.
 *
 * LED Feedback (onboard LED — Pin 13 / LED_BUILTIN):
 *   - Stays ON solid after power-up to indicate the board is live.
 *   - Blinks continuously (200 ms ON / 200 ms OFF) while CRSF signal is actively
 *     being received from the ELRS receiver.
 *   - Double-blinks (ON 100 ms / OFF 100 ms / ON 100 ms / OFF 300 ms, looping)
 *     while a servo retract sequence is in progress.
 *
 *   Priority (highest → lowest):
 *     1. Double-blink  — retract sequence is executing
 *     2. Continuous blink — CRSF signal is present
 *     3. Solid ON       — board powered, no signal received
 *
 * Hardware:
 *   - Teensy 3.2
 *   - ELRS receiver (e.g. BetaFPV ELRS Nano / Happymodel EP1/EP2)
 *   - 9 servos wired to SERVO_PINS[]
 *   - Safety switch wired to CH6 on the RC transmitter
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

// Onboard LED pin (Teensy 3.2 uses pin 13 — same as LED_BUILTIN)
#define LED_PIN        LED_BUILTIN   // pin 13 on Teensy 3.2

// LED blink timings (all in milliseconds)
#define LED_SIGNAL_ON_MS     200     // continuous blink: ON duration while receiving signal
#define LED_SIGNAL_OFF_MS    200     // continuous blink: OFF duration while receiving signal
#define LED_SERVO_PULSE_MS   100     // double-blink: each pulse ON/OFF duration
#define LED_SERVO_GAP_MS     300     // double-blink: longer gap after second pulse before repeating

// Timeout to consider CRSF signal "lost" if no new frame arrives within this window
#define CRSF_SIGNAL_TIMEOUT_MS  500  // ms without a frame → signal considered absent

// Serial port connected to the ELRS receiver (UART1 on Teensy 3.2)
#define CRSF_SERIAL      Serial1
#define CRSF_BAUD        420000          // standard CRSF baud rate

// ── TRIGGER CHANNEL (CH5 / AUX1) ─────────────────────────────────────────────
// CRSF channel that advances the servo sequence (1-indexed).
// Typical RC mapping: CH5 = AUX1 (first aux switch).
#define TRIGGER_CHANNEL  5

// Switch threshold: channel value above this = "switch ON"
// CRSF channel range is 172–1811, midpoint ≈ 992
#define SWITCH_THRESHOLD 1200

// ── SAFETY CHANNEL (CH6 / AUX2) ──────────────────────────────────────────────
// CRSF channel that acts as a hardware safety interlock (1-indexed).
// The firmware will not allow any servo movement unless this channel is in the
// ARMED position (above SAFETY_THRESHOLD).
//
//   CH6 ≤ SAFETY_THRESHOLD  →  SAFE  (no servo movement permitted)
//   CH6 > SAFETY_THRESHOLD  →  ARMED (CH5 trigger is active)
//
// Wire a dedicated 2-position or 3-position switch to AUX2 on your transmitter
// and configure it to output ~1000 µs (safe) and ~2000 µs (armed).
// For a 3-position switch set SAFETY_THRESHOLD = 1600 so only the full-up
// position arms the system.
#define SAFETY_CHANNEL   6
#define SAFETY_THRESHOLD 1500        // channel value that must be exceeded to ARM

// Servo output pins on Teensy 3.2
// Hardware PWM-capable pins: 3, 4, 5, 6, 9, 10, 20, 21, 22, 23, 25, 32
// Pins 0 and 1 are reserved for CRSF serial (RX1/TX1).
static const uint8_t SERVO_PINS[] = { 3, 4, 5, 6, 9, 10, 20, 21, 22 };
#define NUM_SERVOS  (sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]))

// Servo travel limits (microseconds — safe universal range)
#define SERVO_MIN_US   1000
#define SERVO_MAX_US   2000

// How long (ms) to hold between each servo step during the all-close retract.
#define RETRACT_HOLD_MS  300

// Total number of switch flips before the "all close" retract fires.
// Must equal NUM_SERVOS (one flip per servo).
#define FLIPS_BEFORE_CLOSE  NUM_SERVOS   // = 9

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBALS
// ─────────────────────────────────────────────────────────────────────────────

Servo      servos[NUM_SERVOS];
CrsfParser crsf;

// How many servos have been extended so far (0–NUM_SERVOS).
// On the 10th flip this triggers the all-close retract sequence.
static uint8_t  flipCount     = 0;

// State machine for the "all close" retract sequence
enum RetractState {
  RETRACT_IDLE,      // not running
  RETRACT_RUNNING,   // moving servos one at a time
  RETRACT_DONE       // last servo moved, waiting for hold to expire
};

static RetractState retractState  = RETRACT_IDLE;
static uint8_t      retractStep   = 0;    // which servo is next to retract (0-indexed into reverse order)
static uint32_t     retractTimer  = 0;    // millis() when current step started

// Safety interlock
// systemArmed: true when CH6 > SAFETY_THRESHOLD.
// When false, CH5 input is completely ignored — no sequences can start.
static bool         systemArmed   = false;

// Edge detection for the trigger switch
static bool lastSwitch = false;

// ── LED state machine ─────────────────────────────────────────────────────────
enum LedMode { LED_MODE_SOLID, LED_MODE_SIGNAL, LED_MODE_SERVO };

static LedMode  ledMode      = LED_MODE_SOLID;
static uint8_t  ledPhase     = 0;
static uint32_t ledTimer     = 0;
static bool     ledState     = true;

// CRSF signal-present tracking
static bool     signalPresent = false;
static uint32_t lastFrameMs   = 0;

// Whether the retract sequence is active (used by LED logic)
static bool     servoActive   = false;

// ─────────────────────────────────────────────────────────────────────────────
//  LED HELPERS  (fully non-blocking)
// ─────────────────────────────────────────────────────────────────────────────

void updateLed() {
  uint32_t now = millis();

  LedMode desired;
  if (servoActive)        desired = LED_MODE_SERVO;
  else if (signalPresent) desired = LED_MODE_SIGNAL;
  else                    desired = LED_MODE_SOLID;

  if (desired != ledMode) {
    ledMode  = desired;
    ledPhase = 0;
    ledTimer = now;
    ledState = true;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  switch (ledMode) {
    case LED_MODE_SOLID:
      break;

    case LED_MODE_SIGNAL: {
      uint32_t duration = (ledPhase == 0) ? LED_SIGNAL_ON_MS : LED_SIGNAL_OFF_MS;
      if ((now - ledTimer) >= duration) {
        ledPhase = (ledPhase + 1) % 2;
        ledState = (ledPhase == 0);
        ledTimer = now;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
    }

    case LED_MODE_SERVO: {
      uint32_t duration;
      switch (ledPhase) {
        case 0:  duration = LED_SERVO_PULSE_MS; break;
        case 1:  duration = LED_SERVO_PULSE_MS; break;
        case 2:  duration = LED_SERVO_PULSE_MS; break;
        case 3:  duration = LED_SERVO_GAP_MS;   break;
        default: duration = LED_SERVO_PULSE_MS; break;
      }
      if ((now - ledTimer) >= duration) {
        ledPhase = (ledPhase + 1) % 4;
        ledState = (ledPhase == 0 || ledPhase == 2);
        ledTimer = now;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SERVO HELPERS
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

// Drive all servos to rest position
void resetServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(SERVO_MIN_US);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ALL-CLOSE RETRACT SEQUENCE (non-blocking, called every loop)
//  Retracts servos one at a time: servo (NUM_SERVOS-1) → servo 0
//  Returns true when the full retract is finished.
// ─────────────────────────────────────────────────────────────────────────────
bool runRetractSequence() {
  uint32_t now = millis();

  switch (retractState) {
    case RETRACT_IDLE:
      return false;

    case RETRACT_RUNNING:
      if ((now - retractTimer) >= RETRACT_HOLD_MS) {
        if (retractStep < NUM_SERVOS) {
          // Retract from highest index down to 0
          uint8_t idx = (NUM_SERVOS - 1) - retractStep;
          moveServo(idx, SERVO_MIN_US);
          Serial.print(F("  Retract servo "));
          Serial.println(idx + 1);   // 1-indexed for human readability
          retractTimer = now;
          retractStep++;

          if (retractStep >= NUM_SERVOS) {
            retractState = RETRACT_DONE;
          }
        }
      }
      return false;

    case RETRACT_DONE:
      if ((now - retractTimer) >= RETRACT_HOLD_MS) {
        retractState = RETRACT_IDLE;
        retractStep  = 0;
        return true;
      }
      return false;
  }

  return false; // unreachable
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  ledMode  = LED_MODE_SOLID;
  ledTimer = millis();

  Serial.begin(115200);
  CRSF_SERIAL.begin(CRSF_BAUD);

  attachServos();
  resetServos();

  Serial.println(F("TeensyServoControl v4.1 — ready"));
  Serial.print(F("Servos on pins: "));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(SERVO_PINS[i]);
    if (i < NUM_SERVOS - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("Trigger channel: CH"));
  Serial.println(TRIGGER_CHANNEL);
  Serial.print(F("Safety channel:  CH"));
  Serial.print(SAFETY_CHANNEL);
  Serial.print(F("  threshold: "));
  Serial.println(SAFETY_THRESHOLD);
  Serial.print(F("Servos per cycle: "));
  Serial.println(NUM_SERVOS);
  Serial.println(F("Mode: strictly sequential — one servo per switch flip"));
  Serial.println(F("Flip 1-9: extend servo 1-9 individually"));
  Serial.println(F("Flip 10 : all servos retract sequentially (9→1)"));
  Serial.println(F("LED pin: 13"));
  Serial.println(F("LED modes: solid=powered, blink=signal, double-blink=retract active"));
  Serial.println(F("Safety: SAFE (waiting for CH6 ARM)"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  uint32_t now = millis();

  // ── Feed CRSF bytes ───────────────────────────────────────────────────────
  while (CRSF_SERIAL.available()) {
    crsf.feed(CRSF_SERIAL.read());
  }

  // ── Read new CRSF frame ───────────────────────────────────────────────────
  bool switchOn = false;
  if (crsf.hasNewFrame()) {
    lastFrameMs   = now;
    signalPresent = true;

    // ── CH6 SAFETY INTERLOCK ────────────────────────────────────────────────
    uint16_t safetyVal = crsf.getChannel(SAFETY_CHANNEL);
    bool prevArmed     = systemArmed;
    systemArmed        = (safetyVal > SAFETY_THRESHOLD);

    if (systemArmed != prevArmed) {
      Serial.print(F("CH"));
      Serial.print(SAFETY_CHANNEL);
      Serial.print(F(": "));
      Serial.print(safetyVal);
      Serial.println(systemArmed ? F("  Safety: ARMED") : F("  Safety: SAFE"));
    }

    // ── CH5 TRIGGER — only when armed ─────────────────────────────────────
    if (systemArmed) {
      uint16_t chVal = crsf.getChannel(TRIGGER_CHANNEL);
      switchOn = (chVal > SWITCH_THRESHOLD);

      static uint32_t dbgTimer = 0;
      if (now - dbgTimer > 200) {
        dbgTimer = now;
        Serial.print(F("CH"));
        Serial.print(TRIGGER_CHANNEL);
        Serial.print(F(": "));
        Serial.print(chVal);
        Serial.print(F("  Switch: "));
        Serial.print(switchOn ? F("ON") : F("OFF"));
        Serial.print(F("  FlipCount: "));
        Serial.println(flipCount);
      }
    }
  }

  // ── Signal timeout ────────────────────────────────────────────────────────
  if (signalPresent && (now - lastFrameMs) > CRSF_SIGNAL_TIMEOUT_MS) {
    signalPresent = false;
  }

  // ── Safety interlock: suppress trigger and clear edge latch when not armed ─
  if (!systemArmed) {
    switchOn   = false;
    lastSwitch = false;
  }

  // ── Retract sequence tick (runs while active) ─────────────────────────────
  if (retractState != RETRACT_IDLE) {
    servoActive = true;
    if (runRetractSequence()) {
      servoActive = false;
      flipCount   = 0;
      Serial.println(F("→ ALL CLOSED — cycle reset to 0"));
    }
  } else {
    servoActive = false;
  }

  // ── Rising-edge detection — process each flip ─────────────────────────────
  bool risingEdge = (switchOn && !lastSwitch);

  if (risingEdge) {
    if (retractState != RETRACT_IDLE) {
      // Ignore switch input while retract sequence is running
      Serial.println(F("  (flip ignored — retract in progress)"));

    } else if (flipCount < FLIPS_BEFORE_CLOSE) {
      // ── Flips 1-9: extend the next servo individually ─────────────────────
      uint8_t idx = flipCount;   // 0-indexed
      moveServo(idx, SERVO_MAX_US);
      flipCount++;

      Serial.print(F("→ Flip "));
      Serial.print(flipCount);
      Serial.print(F(": Servo "));
      Serial.print(idx + 1);   // 1-indexed for human readability
      Serial.println(F(" extended"));

    } else {
      // ── Flip 10: start all-close retract sequence ─────────────────────────
      Serial.println(F("→ Flip 10: starting all-close retract sequence"));
      retractStep   = 0;
      retractState  = RETRACT_RUNNING;
      retractTimer  = now - RETRACT_HOLD_MS;  // fire first servo on next tick
      servoActive   = true;
    }
  }

  lastSwitch = switchOn;

  // ── Update LED ────────────────────────────────────────────────────────────
  updateLed();
}
