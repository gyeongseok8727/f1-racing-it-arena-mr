// MR vehicle ESP32: wheel speed control + e-diff + steering + ESP-NOW e-stop (PLAN.md §5-6).
//
// Tasks
//   core 1  control  100 Hz  FG -> wheel PI -> PWM, servo, STATE to Jetson, safety checks
//   core 1  comm             UART RX -> latest Cmd (CRC checked)
//   core 0  estop    100 Hz  ESP-NOW permit state machine -> K1
// Both control and estop are on the task watchdog; a hang resets the chip, and during reset
// the PWM pins float high (= stop) and K1 is pulled down (= motor power off).
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <driver/pcnt.h>

#include "config.h"
#include "imu.h"
#include "protocol.h"
#include "sts_servo.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "firmware/common/secrets.h missing: using secrets.h.example values (NOT for the race)"
#include "secrets.h.example"
#endif

// ------------------------------------------------------------------ shared state
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static proto::Cmd g_cmd = {};
static volatile uint32_t g_cmd_ms = 0;
static volatile bool g_have_cmd = false;

static volatile bool g_armed = false;
static volatile uint8_t g_faults_latched = 0;  // cleared on the next ARM
static volatile uint32_t g_crc_err_ms = 0;
static bool g_wdt_boot = false;

// ESP-NOW receive: the callback only copies the packet and the time (PLAN.md §5)
static proto::Permit g_rx = {};
static volatile uint32_t g_rx_ms = 0;
static volatile bool g_rx_new = false;

static Imu g_imu;
static StsServo g_servo;

// ------------------------------------------------------------------ helpers
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void disarm(uint8_t fault) {
  portENTER_CRITICAL(&g_mux);
  g_armed = false;
  g_faults_latched |= fault;
  portEXIT_CRITICAL(&g_mux);
  digitalWrite(PIN_K1, LOW);
}

static void motor_write(int pin_pwm, int pin_dir, bool fwd_level, int ch, float u) {
  // u in [-1, 1]; 0 = stop
  u = clampf(u, -1.0f, 1.0f);
  digitalWrite(pin_dir, u >= 0 ? fwd_level : !fwd_level);
  float mag = fabsf(u);
  float duty = PWM_STOP_HIGH ? (1.0f - mag) : mag;
  ledcWrite(ch, (uint32_t)(duty * ((1 << PWM_BITS) - 1) + 0.5f));
}

static void motors_stop() {
  motor_write(PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR, DIR_FORWARD_LEVEL_L, 0, 0);
  motor_write(PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR, DIR_FORWARD_LEVEL_R, 1, 0);
}

static void pcnt_setup(pcnt_unit_t unit, int pin) {
  pcnt_config_t c = {};
  c.pulse_gpio_num = pin;
  c.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  c.channel = PCNT_CHANNEL_0;
  c.unit = unit;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DIS;
  c.lctrl_mode = PCNT_MODE_KEEP;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = 32767;
  c.counter_l_lim = 0;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(unit, 1000);  // ignore glitches < 12.5 us
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

static int16_t pcnt_take(pcnt_unit_t unit) {
  int16_t v = 0;
  pcnt_get_counter_value(unit, &v);
  pcnt_counter_clear(unit);
  return v;
}

// ------------------------------------------------------------------ wheel controller
struct Wheel {
  float target = 0, meas = 0, integ = 0;
  uint32_t last_pulse_ms = 0;

  float update(float tgt, int16_t pulses, float dt, uint32_t now) {
    target = tgt;
    float m = pulses * (PI * WHEEL_DIAMETER_M / FG_PULSES_PER_WHEEL_REV) / dt;  // speed magnitude
    if (tgt < 0) m = -m;  // FG has no direction: take the commanded one
    meas += 0.3f * (m - meas);
    if (pulses > 0) last_pulse_ms = now;
    float err = tgt - meas;
    integ = clampf(integ + KI * err * dt, -I_LIMIT, I_LIMIT);
    float u = tgt / V_FULL_DUTY_MPS + KP * err + integ;
    if (fabsf(tgt) < 0.02f) {
      integ = 0;
      u = 0;
    }
    return u;
  }
  void reset() { target = meas = integ = 0; }
};

// ------------------------------------------------------------------ ESP-NOW
static void on_espnow_recv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(proto::Permit) || memcmp(mac, MR_HANDHELD_MAC, 6) != 0) return;
  portENTER_CRITICAL_ISR(&g_mux);
  memcpy(&g_rx, data, sizeof(g_rx));
  g_rx_ms = millis();
  g_rx_new = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

static bool espnow_setup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // never join the venue AP
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(MR_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_set_pmk(MR_PMK);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MR_HANDHELD_MAC, 6);
  peer.channel = MR_ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  memcpy(peer.lmk, MR_LMK, 16);
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  esp_now_register_recv_cb(on_espnow_recv);
  return true;
}

// ------------------------------------------------------------------ tasks
static void estop_task(void*) {
  esp_task_wdt_add(nullptr);
  uint32_t session = 0;      // currently armed session
  uint32_t burned[8] = {0};  // recently ended sessions (random ids): never re-arm on these
  uint8_t burned_n = 0;
  uint32_t last_permit_ms = 0;

  auto is_burned = [&](uint32_t s) {
    for (uint8_t i = 0; i < 8; ++i)
      if (burned[i] == s && s != 0) return true;
    return false;
  };
  auto burn = [&](uint32_t s) {
    if (s == 0) return;
    burned[burned_n % 8] = s;
    ++burned_n;
  };

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    esp_task_wdt_reset();
    proto::Permit p;
    bool fresh = false;
    uint32_t rx_ms;
    portENTER_CRITICAL(&g_mux);
    if (g_rx_new) {
      p = g_rx;
      g_rx_new = false;
      fresh = true;
    }
    rx_ms = g_rx_ms;
    portEXIT_CRITICAL(&g_mux);
    uint32_t now = millis();

    if (fresh && proto::permit_valid(p, MR_TEAM_ID) && !is_burned(p.session)) {
      if (p.type == proto::P_STOP) {
        if (g_armed) disarm(proto::F_ESTOP_STOP);
        burn(p.session);
        burn(session);
        session = 0;
      } else if (p.type == proto::P_ARM && !g_armed && p.session != 0 &&
                 digitalRead(PIN_STOP_BUTTON) == HIGH) {
        session = p.session;
        last_permit_ms = rx_ms;
        portENTER_CRITICAL(&g_mux);
        g_faults_latched = 0;
        g_armed = true;
        portEXIT_CRITICAL(&g_mux);
        digitalWrite(PIN_K1, HIGH);
      } else if ((p.type == proto::P_PERMIT || p.type == proto::P_ARM) && g_armed && p.session == session) {
        last_permit_ms = rx_ms;
      }
    }

    if (g_armed) {
      if (now - last_permit_ms > PERMIT_TIMEOUT_MS) {
        disarm(proto::F_ESTOP_TIMEOUT);
        burn(session);
        session = 0;
      } else if (digitalRead(PIN_STOP_BUTTON) == LOW) {
        disarm(proto::F_ESTOP_STOP);
        burn(session);
        session = 0;
      }
    } else {
      digitalWrite(PIN_K1, LOW);
    }
    digitalWrite(PIN_LED, g_armed ? HIGH : ((now / 500) & 1));
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

static void comm_task(void*) {
  proto::Parser parser;
  uint32_t last_err = 0;
  for (;;) {
    while (Serial.available()) {
      if (parser.push((uint8_t)Serial.read()) && parser.type() == proto::T_CMD &&
          parser.len() == sizeof(proto::Cmd)) {
        portENTER_CRITICAL(&g_mux);
        memcpy(&g_cmd, parser.payload(), sizeof(g_cmd));
        g_cmd_ms = millis();
        g_have_cmd = true;
        portEXIT_CRITICAL(&g_mux);
      }
    }
    if (parser.crc_errors != last_err) {
      last_err = parser.crc_errors;
      g_crc_err_ms = millis();
    }
    vTaskDelay(1);
  }
}

static void control_task(void*) {
  esp_task_wdt_add(nullptr);
  Wheel wl, wr;
  float v_slewed = 0;
  uint16_t seq = 0;
  uint8_t out[32];
  const float dt = 1.0f / CONTROL_HZ;
  uint32_t servo_tick = 0;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    esp_task_wdt_reset();
    uint32_t now = millis();
    proto::Cmd cmd;
    uint32_t cmd_ms;
    bool have;
    portENTER_CRITICAL(&g_mux);
    cmd = g_cmd;
    cmd_ms = g_cmd_ms;
    have = g_have_cmd;
    portEXIT_CRITICAL(&g_mux);

    uint8_t faults = 0;
    bool cmd_ok = have && (now - cmd_ms) <= JETSON_TIMEOUT_MS && (cmd.flags & proto::FLAG_DRIVE);
    if (have && (now - cmd_ms) > JETSON_TIMEOUT_MS) faults |= proto::F_JETSON_TIMEOUT;

    float steer = clampf(cmd.steer_mrad / 1000.0f, -STEER_MAX_RAD, STEER_MAX_RAD);
    float v = cmd_ok ? cmd.speed_mm_s / 1000.0f : 0.0f;
    // steering-angle speed cap (same rule as the Jetson side)
    float t = fabsf(tanf(steer));
    if (t > 1e-3f) {
      float cap = sqrtf(A_LAT_LIMIT * WHEELBASE_M / t);
      v = clampf(v, -cap, cap);
    }
    if (!g_armed) v = 0;
    float dv = clampf(v - v_slewed, -MAX_ACCEL_MPS2 * dt, MAX_ACCEL_MPS2 * dt);
    v_slewed += dv;

    // e-diff
    float vl = v_slewed, vr = v_slewed;
    if (cmd.flags & proto::FLAG_EDIFF) {
      float g = cmd.ediff_gain_pct / 100.0f;
      float half = g * (tanf(steer) / WHEELBASE_M) * TRACK_WIDTH_M / 2.0f;
      vl = v_slewed * (1.0f - half);
      vr = v_slewed * (1.0f + half);
    }

    int16_t pl = pcnt_take(PCNT_UNIT_0), pr = pcnt_take(PCNT_UNIT_1);
    if (g_armed) {
      float ul = wl.update(vl, pl, dt, now);
      float ur = wr.update(vr, pr, dt, now);
      motor_write(PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR, DIR_FORWARD_LEVEL_L, 0, ul);
      motor_write(PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR, DIR_FORWARD_LEVEL_R, 1, ur);
      // stall: commanded to move but no FG pulses (no spare motors in the budget)
      if (fabsf(vl) > STALL_MIN_CMD_MPS && now - wl.last_pulse_ms > STALL_MS) disarm(proto::F_STALL_LEFT);
      if (fabsf(vr) > STALL_MIN_CMD_MPS && now - wr.last_pulse_ms > STALL_MS) disarm(proto::F_STALL_RIGHT);
    } else {
      motors_stop();
      wl.update(0, pl, dt, now);
      wr.update(0, pr, dt, now);
      wl.integ = wr.integ = 0;
      wl.last_pulse_ms = wr.last_pulse_ms = now;
      v_slewed = 0;
    }

    if (++servo_tick >= 2) {  // 50 Hz
      servo_tick = 0;
      int pos = SERVO_CENTER_TICKS + SERVO_DIR * (int)lroundf(steer * SERVO_TICKS_PER_RAD);
      g_servo.write_position(SERVO_ID, pos);
    }

    float gz = 0;
    if (!g_imu.gyro_z(gz)) faults |= proto::F_IMU;
    if (now - g_crc_err_ms < 1000 && g_crc_err_ms) faults |= proto::F_CRC_ERRORS;
    if (g_wdt_boot) faults |= proto::F_WATCHDOG_BOOT;

    proto::State st = {};
    st.seq = seq++;
    st.t_ms = now;
    st.v_left_mm_s = (int16_t)lroundf(wl.meas * 1000);
    st.v_right_mm_s = (int16_t)lroundf(wr.meas * 1000);
    st.gyro_z_mrad_s = (int16_t)lroundf(gz * 1000);
    st.steer_mrad = (int16_t)lroundf(steer * 1000);
    st.arm = g_armed ? 1 : 0;
    st.faults = faults | g_faults_latched;
    st.batt_mv = (uint16_t)(analogReadMilliVolts(PIN_BATT_ADC) * BATT_DIVIDER);
    st.cmd_seq = cmd.seq;
    size_t n = proto::encode(proto::T_STATE, &st, sizeof(st), out);
    Serial.write(out, n);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / CONTROL_HZ));
  }
}

// ------------------------------------------------------------------ setup
void setup() {
  // safe state first
  pinMode(PIN_K1, OUTPUT);
  digitalWrite(PIN_K1, LOW);
  ledcSetup(0, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(1, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_MOTOR_L_PWM, 0);
  ledcAttachPin(PIN_MOTOR_R_PWM, 1);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  motors_stop();
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FG_L, INPUT);
  pinMode(PIN_FG_R, INPUT);

  esp_reset_reason_t rr = esp_reset_reason();
  g_wdt_boot = (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_WDT);

  Serial.setRxBufferSize(1024);
  Serial.begin(UART_BAUD);
  g_servo.begin(Serial2, SERVO_BAUD, PIN_SERVO_RX, PIN_SERVO_TX);
  pcnt_setup(PCNT_UNIT_0, PIN_FG_L);
  pcnt_setup(PCNT_UNIT_1, PIN_FG_R);
  g_imu.begin();  // car must be still for ~1 s here (gyro bias)

  bool espnow_ok = espnow_setup();
  // boot log is plain text before binary frames start; the Jetson parser skips it
  Serial.printf("\nMR vehicle fw | MAC: %s | espnow %s | imu %s | reset %d\n",
                WiFi.macAddress().c_str(), espnow_ok ? "ok" : "FAIL", g_imu.ok() ? "ok" : "none", (int)rr);

  esp_task_wdt_init(1, true);  // 1 s, panic -> reset
  xTaskCreatePinnedToCore(estop_task, "estop", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(comm_task, "comm", 4096, nullptr, 3, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
