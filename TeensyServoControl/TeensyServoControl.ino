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
 *   Servo Deadband:
 *   Each servo's last-written position is tracked in servoPositionUs[].  Before
 *   issuing a new writeMicroseconds() command the firmware checks whether the
 *   target is within SERVO_DEADBAND_US of the current position.  If so the write
 *   is skipped, preventing continuous micro-adjustments and reducing jitter.
 * 
 * LED Feedback (onboard LED — Pin 13 / LED_BUILTIN):
 *   - Stays ON solid after power-up to indicate the board is live.
 *   - Blinks continuously (200 ms ON / 200 ms OFF) while CRSF signal is actively
 *     being received from the ELRS receiver.
 *   - Double-blinks (ON 100ms / OFF 100ms / ON 100ms / OFF 300ms, looping) when a
 *     servo command is active (deploy or retract sequence in progress).
 *
 *   Priority (highest → lowest):
 *     1. Double-blink  — servo sequence is executing
 *     2. Continuous blink — CRSF signal is present but no sequence running
 *     3. Solid ON       — board powered, no signal received
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

// Servo deadband (microseconds)
// If the target position is within this many µs of the current position,
// the write() command is suppressed to prevent jitter and micro-adjustments.
#define SERVO_DEADBAND_US  100

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

Servo      servos[NUM_SERVOS];
CrsfParser crsf;

// Tracks the last position written to each servo (µs).
// Initialised to SERVO_MIN_US (the startup reset position).
static uint16_t servoPositionUs[NUM_SERVOS];

// State machine
enum State {
  STATE_IDLE,
  STATE_DEPLOYING,
  STATE_DEPLOYED,
  STATE_RETRACTING
};

static State    currentState  = STATE_IDLE;
static uint8_t  seqStep       = 0;
static uint32_t stepTimer     = 0;
static bool     lastSwitch    = false;

// ── LED state machine ─────────────────────────────────────────────────────────
//
//  LED_MODE_SOLID         — board alive, no CRSF signal
//  LED_MODE_SIGNAL        — continuous blink: CRSF signal present
//  LED_MODE_SERVO         — double-blink: servo sequence active
//
//  Double-blink waveform (repeating):
//    [ON LED_SERVO_PULSE_MS] [OFF LED_SERVO_PULSE_MS]
//    [ON LED_SERVO_PULSE_MS] [OFF LED_SERVO_GAP_MS]
//    ... repeat ...
//
//  ledPhase tracks position within each pattern:
//    Continuous blink: 0 = ON phase, 1 = OFF phase
//    Double-blink:     0 = pulse1 ON, 1 = pulse1 OFF,
//                      2 = pulse2 ON, 3 = gap OFF

enum LedMode { LED_MODE_SOLID, LED_MODE_SIGNAL, LED_MODE_SERVO };

static LedMode  ledMode      = LED_MODE_SOLID;
static uint8_t  ledPhase     = 0;
static uint32_t ledTimer     = 0;
static bool     ledState     = true;   // current physical LED output

// CRSF signal-present tracking
static bool     signalPresent   = false;
static uint32_t lastFrameMs     = 0;    // millis() when last CRSF frame decoded

// Whether a servo sequence is currently running
static bool     servoActive     = false;

// ─────────────────────────────────────────────────────────────────────────────
//  LED HELPERS  (fully non-blocking)
// ─────────────────────────────────────────────────────────────────────────────

// Call once per loop() to update the LED output.
// No delay() — advances through phases based on millis().
void updateLed() {
  uint32_t now = millis();

  // ── Determine desired mode (priority: servo > signal > solid) ────────────
  LedMode desired;
  if (servoActive)        desired = LED_MODE_SERVO;
  else if (signalPresent) desired = LED_MODE_SIGNAL;
  else                    desired = LED_MODE_SOLID;

  // ── On mode change: restart phase from beginning ─────────────────────────
  if (desired != ledMode) {
    ledMode  = desired;
    ledPhase = 0;
    ledTimer = now;

    // Immediately apply first-phase output
    if (ledMode == LED_MODE_SOLID) {
      ledState = true;
    } else {
      ledState = true;   // all patterns start with LED ON
    }
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    return;
  }

  // ── Advance phase within current mode ────────────────────────────────────
  switch (ledMode) {

    case LED_MODE_SOLID:
      // Nothing to do — stays HIGH
      break;

    case LED_MODE_SIGNAL: {
      // phase 0 = ON  (LED_SIGNAL_ON_MS)
      // phase 1 = OFF (LED_SIGNAL_OFF_MS)
      uint32_t duration = (ledPhase == 0) ? LED_SIGNAL_ON_MS : LED_SIGNAL_OFF_MS;
      if ((now - ledTimer) >= duration) {
        ledPhase = (ledPhase + 1) % 2;
        ledState = (ledPhase == 0);          // phase 0 → ON, phase 1 → OFF
        ledTimer = now;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
    }

    case LED_MODE_SERVO: {
      // phase 0 = pulse1 ON   (LED_SERVO_PULSE_MS)
      // phase 1 = pulse1 OFF  (LED_SERVO_PULSE_MS)
      // phase 2 = pulse2 ON   (LED_SERVO_PULSE_MS)
      // phase 3 = gap   OFF   (LED_SERVO_GAP_MS)
      uint32_t duration;
      switch (ledPhase) {
        case 0: duration = LED_SERVO_PULSE_MS; break;
        case 1: duration = LED_SERVO_PULSE_MS; break;
        case 2: duration = LED_SERVO_PULSE_MS; break;
        case 3: duration = LED_SERVO_GAP_MS;   break;
        default: duration = LED_SERVO_PULSE_MS; break;
      }
      if ((now - ledTimer) >= duration) {
        ledPhase = (ledPhase + 1) % 4;
        // phases 0, 2 → ON;  phases 1, 3 → OFF
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

// Move a servo to the target position (µs).
// If the target is within SERVO_DEADBAND_US of the current position the command
// is suppressed, preventing jitter from repeated tiny adjustments.
void moveServo(uint8_t idx, uint16_t us) {
  if (idx >= NUM_SERVOS) return;

  uint16_t current = servoPositionUs[idx];
  uint16_t delta   = (us > current) ? (us - current) : (current - us);

  if (delta <= SERVO_DEADBAND_US) {
    // Within deadband — skip the write to avoid servo jitter
    return;
  }

  servos[idx].writeMicroseconds(us);
  servoPositionUs[idx] = us;
}

// Drive all servos to a safe rest position
void resetServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].writeMicroseconds(SERVO_MIN_US);
    servoPositionUs[i] = SERVO_MIN_US;
  }
}

// Run one step of a sequence; returns true when the full sequence is complete.
// The LED is handled entirely by updateLed() — no delay() calls here.
bool runSequence(const SequenceStep* seq, uint8_t numSteps) {
  if (seqStep >= numSteps) return true;   // already done

  uint32_t now = millis();

  if (seqStep == 0 || (now - stepTimer) >= seq[seqStep - 1].holdMs) {
    // Execute current step
    moveServo(seq[seqStep].servoIndex, seq[seqStep].targetUs);
    stepTimer = millis();
    seqStep++;

    // If this step has zero hold time, immediately chain to next
    if (seq[seqStep - 1].holdMs == 0 && seqStep < numSteps) {
      moveServo(seq[seqStep].servoIndex, seq[seqStep].targetUs);
      stepTimer = millis();
      seqStep++;
    }
  }

  return (seqStep >= numSteps && (millis() - stepTimer) >= seq[numSteps - 1].holdMs);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  // Onboard LED — turn ON immediately at power-up (solid ON = board alive)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  ledMode  = LED_MODE_SOLID;
  ledTimer = millis();

  Serial.begin(115200);           // USB debug serial
  CRSF_SERIAL.begin(CRSF_BAUD);  // ELRS receiver

  attachServos();
  resetServos();

  Serial.println(F("TeensyServoControl v3.1 — ready"));
  Serial.print(F("Servos on pins: "));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(SERVO_PINS[i]);
    if (i < NUM_SERVOS - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("Trigger channel: CH"));
  Serial.println(TRIGGER_CHANNEL);
  Serial.println(F("LED pin: 13"));
  Serial.println(F("LED modes: solid=powered, blink=signal, double-blink=servo active"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  uint32_t now = millis();

  // ── Feed incoming bytes into the CRSF parser ──────────────────────────────
  while (CRSF_SERIAL.available()) {
    crsf.feed(CRSF_SERIAL.read());
  }

  // ── Read channel value once a new frame is available ──────────────────────
  bool switchOn = false;
  if (crsf.hasNewFrame()) {
    lastFrameMs  = now;                      // mark signal as present
    signalPresent = true;

    uint16_t chVal = crsf.getChannel(TRIGGER_CHANNEL);
    switchOn = (chVal > SWITCH_THRESHOLD);

    // Debug (comment out in production to reduce latency)
    static uint32_t dbgTimer = 0;
    if (now - dbgTimer > 200) {
      dbgTimer = now;
      Serial.print(F("CH"));
      Serial.print(TRIGGER_CHANNEL);
      Serial.print(F(": "));
      Serial.print(chVal);
      Serial.print(F("  Switch: "));
      Serial.println(switchOn ? F("ON") : F("OFF"));
    }
  }

  // ── Signal timeout: mark absent if no frame within CRSF_SIGNAL_TIMEOUT_MS ──
  if (signalPresent && (now - lastFrameMs) > CRSF_SIGNAL_TIMEOUT_MS) {
    signalPresent = false;
  }

  // ── State machine ─────────────────────────────────────────────────────────
  switch (currentState) {

    case STATE_IDLE:
      servoActive = false;
      if (switchOn && !lastSwitch) {
        // Rising edge — begin deploy sequence
        seqStep      = 0;
        currentState = STATE_DEPLOYING;
        servoActive  = true;
        Serial.println(F("→ DEPLOYING"));
      }
      break;

    case STATE_DEPLOYING:
      servoActive = true;
      if (runSequence(DEPLOY_SEQUENCE, DEPLOY_STEPS)) {
        currentState = STATE_DEPLOYED;
        servoActive  = false;
        Serial.println(F("→ DEPLOYED"));
      }
      break;

    case STATE_DEPLOYED:
      servoActive = false;
      if (!switchOn && lastSwitch) {
        // Falling edge — begin retract sequence
        seqStep      = 0;
        currentState = STATE_RETRACTING;
        servoActive  = true;
        Serial.println(F("→ RETRACTING"));
      }
      break;

    case STATE_RETRACTING:
      servoActive = true;
      if (runSequence(RETRACT_SEQUENCE, RETRACT_STEPS)) {
        currentState = STATE_IDLE;
        servoActive  = false;
        Serial.println(F("→ IDLE"));
      }
      break;
  }

  lastSwitch = switchOn;

  // ── Update LED (non-blocking, runs every loop iteration) ──────────────────
  updateLed();
}
