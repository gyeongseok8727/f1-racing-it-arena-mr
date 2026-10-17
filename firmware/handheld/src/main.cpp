// MR handheld e-stop transmitter (PLAN.md §6, docs/PROTOCOL.md).
//
// ARM button  -> new random session, ARM x3, then PERMIT every 50 ms
// STOP button -> STOP x5, stop sending (car disarms immediately)
// E-stop (b-contact in the battery line) -> board dies -> no PERMIT -> car disarms in 200 ms
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>

#include "protocol.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "firmware/common/secrets.h missing: using secrets.h.example values (NOT for the race)"
#include "secrets.h.example"
#endif

constexpr int PIN_ARM = 18;   // TODO(HW) to GND
constexpr int PIN_STOP = 19;  // TODO(HW) to GND
constexpr int PIN_LED = 2;
constexpr uint32_t PERIOD_MS = 50;

static uint32_t g_session = 0;
static uint32_t g_seq = 0;
static volatile uint32_t g_ok = 0, g_fail = 0;

static void on_sent(const uint8_t*, esp_now_send_status_t s) {
  if (s == ESP_NOW_SEND_SUCCESS) ++g_ok; else ++g_fail;
}

static void send(uint8_t type) {
  proto::Permit p = {};
  p.magic = proto::ESPNOW_MAGIC;
  p.team = MR_TEAM_ID;
  p.type = type;
  p.session = g_session;
  p.seq = ++g_seq;
  proto::permit_seal(p);
  esp_now_send(MR_VEHICLE_MAC, reinterpret_cast<uint8_t*>(&p), sizeof(p));
}

void setup() {
  pinMode(PIN_ARM, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(MR_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("espnow init FAIL");
  }
  esp_now_set_pmk(MR_PMK);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MR_VEHICLE_MAC, 6);
  peer.channel = MR_ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  memcpy(peer.lmk, MR_LMK, 16);
  esp_now_add_peer(&peer);
  esp_now_register_send_cb(on_sent);
  Serial.printf("MR handheld | MAC: %s\n", WiFi.macAddress().c_str());
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();

  if (digitalRead(PIN_STOP) == LOW) {
    if (g_session) {
      for (int i = 0; i < 5; ++i) { send(proto::P_STOP); delay(5); }
      g_session = 0;
    }
  } else if (digitalRead(PIN_ARM) == LOW && g_session == 0) {
    delay(30);  // debounce
    if (digitalRead(PIN_ARM) == LOW) {
      do { g_session = esp_random(); } while (g_session == 0);
      for (int i = 0; i < 3; ++i) { send(proto::P_ARM); delay(5); }
      while (digitalRead(PIN_ARM) == LOW) delay(5);  // wait for release
    }
  }

  if (g_session && now - last >= PERIOD_MS) {
    last = now;
    send(proto::P_PERMIT);
  }
  // LED: solid = permits flowing and acked, blink = disarmed
  digitalWrite(PIN_LED, g_session ? (g_fail == 0 || g_ok > g_fail) : ((now / 250) & 1));
  delay(2);
}
