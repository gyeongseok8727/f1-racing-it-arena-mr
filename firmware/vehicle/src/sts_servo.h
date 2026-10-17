// Feetech STS/SMS protocol (ST3215): write goal position only.
#pragma once
#include <Arduino.h>

class StsServo {
 public:
  void begin(HardwareSerial& port, uint32_t baud, int rx, int tx) {
    port_ = &port;
    port_->begin(baud, SERIAL_8N1, rx, tx);
  }

  // position 0..4095, speed in steps/s (0 = max)
  void write_position(uint8_t id, int position, int speed = 0) {
    if (!port_) return;
    position = constrain(position, 0, 4095);
    // WRITE(0x03) at GOAL_POSITION 0x2A: pos L/H, time L/H, speed L/H
    uint8_t p[] = {0xFF, 0xFF, id, 9, 0x03, 0x2A,
                   (uint8_t)(position & 0xFF), (uint8_t)(position >> 8),
                   0, 0,
                   (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8), 0};
    uint8_t sum = 0;
    for (size_t i = 2; i < sizeof(p) - 1; ++i) sum += p[i];
    p[sizeof(p) - 1] = ~sum;
    port_->write(p, sizeof(p));
  }

 private:
  HardwareSerial* port_ = nullptr;
};
