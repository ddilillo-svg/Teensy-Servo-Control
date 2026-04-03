/**
 * crsf_parser.h
 * 
 * Lightweight CRSF (Crossfire Serial Protocol) frame parser for Teensy / Arduino.
 * 
 * Parses RC_CHANNELS_PACKED frames (0x16) and exposes 16 channels in the
 * standard CRSF range: 172 – 1811 (midpoint ≈ 992).
 * 
 * Usage:
 *   CrsfParser crsf;
 *   // In loop():
 *   while (Serial1.available()) crsf.feed(Serial1.read());
 *   if (crsf.hasNewFrame()) {
 *       uint16_t ch1 = crsf.getChannel(1);   // 1-indexed
 *   }
 * 
 * CRSF frame structure:
 *   [SYNC 0xC8] [LEN] [TYPE] [PAYLOAD…] [CRC8]
 * 
 * RC_CHANNELS_PACKED (type 0x16, payload = 22 bytes):
 *   16 channels × 11 bits, little-endian packed.
 */

#pragma once
#include <Arduino.h>

// ─── Protocol constants ───────────────────────────────────────────────────────
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_RC_CH    0x16
#define CRSF_MAX_FRAME_LEN      64
#define CRSF_RC_PAYLOAD_BYTES   22
#define CRSF_NUM_CHANNELS       16

// Channel value range
#define CRSF_CH_MIN             172
#define CRSF_CH_MID             992
#define CRSF_CH_MAX            1811

class CrsfParser {
public:
  CrsfParser() { reset(); }

  // Feed one byte from the UART stream.
  void feed(uint8_t b) {
    switch (_state) {

      case S_SYNC:
        if (b == CRSF_SYNC_BYTE) {
          _buf[0] = b;
          _state  = S_LEN;
        }
        break;

      case S_LEN:
        _frameLen = b;   // includes TYPE + PAYLOAD + CRC (not SYNC, not LEN itself)
        if (_frameLen < 2 || _frameLen > CRSF_MAX_FRAME_LEN - 2) {
          reset(); break;
        }
        _buf[1] = b;
        _pos    = 2;
        _state  = S_DATA;
        break;

      case S_DATA:
        _buf[_pos++] = b;
        if (_pos == (uint8_t)(_frameLen + 2)) {
          // Full frame received
          if (validateCrc()) {
            parseFrame();
          }
          reset();
        }
        break;
    }
  }

  // Returns true (once) when a fresh RC channels frame has been parsed.
  bool hasNewFrame() {
    if (_newFrame) { _newFrame = false; return true; }
    return false;
  }

  // Get a channel value (1-indexed). Returns CRSF_CH_MID if out of range.
  uint16_t getChannel(uint8_t ch) const {
    if (ch < 1 || ch > CRSF_NUM_CHANNELS) return CRSF_CH_MID;
    return _channels[ch - 1];
  }

  // Map CRSF channel value to a servo pulse width in µs.
  // CRSF 172–1811  →  1000–2000 µs
  static uint16_t toServoUs(uint16_t crsfVal) {
    // Clamp
    crsfVal = constrain(crsfVal, CRSF_CH_MIN, CRSF_CH_MAX);
    return map(crsfVal, CRSF_CH_MIN, CRSF_CH_MAX, 1000, 2000);
  }

private:
  enum State : uint8_t { S_SYNC, S_LEN, S_DATA };

  uint8_t  _buf[CRSF_MAX_FRAME_LEN];
  uint8_t  _pos       = 0;
  uint8_t  _frameLen  = 0;
  State    _state     = S_SYNC;
  bool     _newFrame  = false;
  uint16_t _channels[CRSF_NUM_CHANNELS] = {};

  void reset() {
    _state = S_SYNC;
    _pos   = 0;
  }

  // CRC-8/DVB-S2 (polynomial 0xD5)
  static uint8_t crc8dvb(const uint8_t* buf, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
      crc ^= *buf++;
      for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
      }
    }
    return crc;
  }

  bool validateCrc() {
    // CRC covers TYPE + PAYLOAD (buf[2] … buf[frameLen]) 
    uint8_t calcCrc = crc8dvb(&_buf[2], _frameLen - 1);
    uint8_t rxCrc   = _buf[_frameLen + 1];
    return calcCrc == rxCrc;
  }

  void parseFrame() {
    uint8_t type = _buf[2];
    if (type != CRSF_FRAMETYPE_RC_CH) return;

    // Payload starts at buf[3], length 22 bytes → 16 × 11-bit channels
    const uint8_t* p = &_buf[3];

    _channels[ 0] = ((uint16_t)p[ 0]       | ((uint16_t)p[ 1] << 8)) & 0x07FF;
    _channels[ 1] = ((uint16_t)p[ 1] >> 3  | ((uint16_t)p[ 2] << 5)) & 0x07FF;
    _channels[ 2] = ((uint16_t)p[ 2] >> 6  | ((uint16_t)p[ 3] << 2) | ((uint16_t)p[ 4] << 10)) & 0x07FF;
    _channels[ 3] = ((uint16_t)p[ 4] >> 1  | ((uint16_t)p[ 5] << 7)) & 0x07FF;
    _channels[ 4] = ((uint16_t)p[ 5] >> 4  | ((uint16_t)p[ 6] << 4)) & 0x07FF;
    _channels[ 5] = ((uint16_t)p[ 6] >> 7  | ((uint16_t)p[ 7] << 1) | ((uint16_t)p[ 8] << 9))  & 0x07FF;
    _channels[ 6] = ((uint16_t)p[ 8] >> 2  | ((uint16_t)p[ 9] << 6)) & 0x07FF;
    _channels[ 7] = ((uint16_t)p[ 9] >> 5  | ((uint16_t)p[10] << 3)) & 0x07FF;
    _channels[ 8] = ((uint16_t)p[11]       | ((uint16_t)p[12] << 8)) & 0x07FF;
    _channels[ 9] = ((uint16_t)p[12] >> 3  | ((uint16_t)p[13] << 5)) & 0x07FF;
    _channels[10] = ((uint16_t)p[13] >> 6  | ((uint16_t)p[14] << 2) | ((uint16_t)p[15] << 10)) & 0x07FF;
    _channels[11] = ((uint16_t)p[15] >> 1  | ((uint16_t)p[16] << 7)) & 0x07FF;
    _channels[12] = ((uint16_t)p[16] >> 4  | ((uint16_t)p[17] << 4)) & 0x07FF;
    _channels[13] = ((uint16_t)p[17] >> 7  | ((uint16_t)p[18] << 1) | ((uint16_t)p[19] << 9))  & 0x07FF;
    _channels[14] = ((uint16_t)p[19] >> 2  | ((uint16_t)p[20] << 6)) & 0x07FF;
    _channels[15] = ((uint16_t)p[20] >> 5  | ((uint16_t)p[21] << 3)) & 0x07FF;

    _newFrame = true;
  }
};
