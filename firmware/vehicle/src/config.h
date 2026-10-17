// Vehicle ESP32 configuration. Everything marked TODO(HW) must be checked on the real car.
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------- pins
// Avoid strapping pins 0, 2, 5, 12, 15 for anything that must be LOW at boot.
// WARNING(HW): GPIO14 emits a PWM burst during boot on classic ESP32 (like 1, 3, 5, 15).
// With K1 here the motor relay can blip on at power-up. Prefer GPIO 13 or 23 when wiring.
constexpr int PIN_K1 = 14;             // motor power MOSFET (IRLZ44N), external pull-down
constexpr int PIN_MOTOR_L_PWM = 25;    // TODO(HW) PM28 speed input (via level shifter to 5 V)
constexpr int PIN_MOTOR_R_PWM = 26;    // TODO(HW)
constexpr int PIN_MOTOR_L_DIR = 27;    // TODO(HW)
constexpr int PIN_MOTOR_R_DIR = 33;    // TODO(HW)
constexpr int PIN_FG_L = 34;           // TODO(HW) input-only pin, FG open collector -> pull-up
constexpr int PIN_FG_R = 35;           // TODO(HW)
constexpr int PIN_SERVO_TX = 17;       // ST3215 bus via half-duplex adapter (Serial2)
constexpr int PIN_SERVO_RX = 16;
constexpr int PIN_STOP_BUTTON = 32;    // on-car stop button to GND (INPUT_PULLUP)
constexpr int PIN_BATT_ADC = 36;       // TODO(HW) divider
constexpr int PIN_LED = 4;
constexpr int PIN_IMU_SDA = 21;
constexpr int PIN_IMU_SCL = 22;

// ---------------------------------------------------------------- motors (PM28-BL3657B 1/5.2)
// PLAN.md: the driver stops when its PWM input sits at 5 V, so a reset/hang (pin floating,
// pulled up) stops the motor. Duty is therefore inverted: 100 % high = stop.
constexpr bool PWM_STOP_HIGH = true;   // TODO(HW) confirm with the driver datasheet
constexpr uint32_t PWM_FREQ_HZ = 20000;  // TODO(HW) driver's accepted PWM range
constexpr int PWM_BITS = 10;
constexpr bool DIR_FORWARD_LEVEL_L = HIGH;  // TODO(HW) left/right mirrored mounting
constexpr bool DIR_FORWARD_LEVEL_R = LOW;   // TODO(HW)

// ---------------------------------------------------------------- wheels / FG
constexpr float WHEEL_DIAMETER_M = 0.055f;  // TODO(HW) effective rolling diameter (TPU squashes)
constexpr float FG_PULSES_PER_WHEEL_REV = 6.0f * 5.2f;  // TODO(HW) FG/motor rev x gear; measure
constexpr float V_FULL_DUTY_MPS = 3.4f;     // TODO(HW) wheel speed at 100 % duty on 22 V (feedforward)
constexpr float TRACK_WIDTH_M = 0.13f;      // rear wheel spacing
constexpr float WHEELBASE_M = 0.145f;

// speed PI (per wheel), on top of feedforward
constexpr float KP = 0.15f;  // duty per m/s error
constexpr float KI = 0.8f;
constexpr float I_LIMIT = 0.3f;
constexpr float MAX_ACCEL_MPS2 = 4.0f;  // command slew

// ---------------------------------------------------------------- steering (ST3215 bus servo)
constexpr uint8_t SERVO_ID = 1;
constexpr int SERVO_CENTER_TICKS = 2048;       // TODO(HW) straight-ahead position
constexpr float SERVO_TICKS_PER_RAD = 651.9f;  // 4096/(2*pi) x linkage ratio. TODO(HW) map
constexpr float STEER_MAX_RAD = 0.52f;         // ~30 deg. Need >= 25.8 deg for R = 0.3 m
constexpr int SERVO_DIR = 1;                   // TODO(HW) +1 if +ticks steers left
constexpr uint32_t SERVO_BAUD = 1000000;

// ---------------------------------------------------------------- safety
constexpr uint32_t JETSON_TIMEOUT_MS = 150;
constexpr uint32_t PERMIT_TIMEOUT_MS = 200;
constexpr uint32_t STALL_MS = 400;
constexpr float STALL_MIN_CMD_MPS = 0.25f;
constexpr float A_LAT_LIMIT = 0.7f * 9.81f;  // v <= sqrt(a_lat * L / tan|delta|)

// ---------------------------------------------------------------- IMU (gyro z only is used)
// TODO(HW) set to the purchased part. IMU_NONE -> Jetson derives yaw rate from wheel speeds.
#define IMU_NONE 0
#define IMU_MPU6050 1   // also MPU6500/9250 register-compatible, addr 0x68
#define IMU_LSM6DSOX 2  // addr 0x6A
#ifndef IMU_MODEL
#define IMU_MODEL IMU_NONE
#endif

// ---------------------------------------------------------------- misc
constexpr uint32_t UART_BAUD = 921600;
constexpr float BATT_DIVIDER = 11.0f;  // TODO(HW) (R1+R2)/R2
constexpr uint32_t CONTROL_HZ = 100;
