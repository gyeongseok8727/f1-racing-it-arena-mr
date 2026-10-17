// Minimal gyro-z reader. Bias is averaged at boot while the car sits still.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "config.h"

class Imu {
 public:
  bool begin() {
#if IMU_MODEL == IMU_NONE
    return false;
#else
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);
#if IMU_MODEL == IMU_MPU6050
    addr_ = 0x68;
    if (!write(0x6B, 0x00)) return false;  // wake
    write(0x1B, 0x08);                      // +-500 dps
    write(0x1A, 0x03);                      // DLPF ~44 Hz
    scale_ = (500.0f / 32768.0f) * DEG_TO_RAD;
    reg_gz_ = 0x47;
    big_endian_ = true;
#elif IMU_MODEL == IMU_LSM6DSOX
    addr_ = 0x6A;
    if (!write(0x11, 0x84)) return false;  // CTRL2_G: 1.66 kHz, 500 dps
    scale_ = (17.5f / 1000.0f) * DEG_TO_RAD;
    reg_gz_ = 0x26;
    big_endian_ = false;
#endif
    delay(100);
    float sum = 0;
    int n = 0;
    for (int i = 0; i < 200; ++i, delay(5)) {
      float g;
      if (raw(g)) { sum += g; ++n; }
    }
    if (n < 100) return false;
    bias_ = sum / n;
    ok_ = true;
    return true;
#endif
  }

  // rad/s, bias removed. false on bus error.
  bool gyro_z(float& out) {
    if (!ok_) return false;
    float g;
    if (!raw(g)) return false;
    out = g - bias_;
    return true;
  }

  bool ok() const { return ok_; }

 private:
  uint8_t addr_ = 0, reg_gz_ = 0;
  bool big_endian_ = true, ok_ = false;
  float scale_ = 0, bias_ = 0;

  bool write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool raw(float& out) {
    Wire.beginTransmission(addr_);
    Wire.write(reg_gz_);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr_, 2) != 2) return false;
    uint8_t a = Wire.read(), b = Wire.read();
    int16_t v = big_endian_ ? (int16_t)((a << 8) | b) : (int16_t)((b << 8) | a);
    out = v * scale_;
    return true;
  }
};
