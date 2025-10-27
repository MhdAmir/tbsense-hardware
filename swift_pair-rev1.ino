// Raspberry Pi Pico 2 W — Soil + VOC + BLE NUS (safe notify)
// Core: Earle Philhower (arduino-pico)
// Tools → IP/Bluetooth Stack: pico_w_bluetooth

#include <Arduino.h>
#include <BTstackLib.h>
#include "btstack.h"   // att_server_notify, att_server_request_can_send_now_event, HCI_CON_HANDLE_INVALID

// ---------- Pins ----------
const int VOC_PIN   = A0;     // GP26 / ADC0
const int SOIL_PIN  = A1;     // GP27 / ADC1
const int LED_PIN   = LED_BUILTIN;

// ---------- Soil calib ----------
int RAW_DRY = 3200;           // ganti setelah kalibrasi
int RAW_WET = 1400;           // ganti setelah kalibrasi
const int AVG_SAMPLES = 10;
const int VREF_mV     = 3300;
const int WET_THRESHOLD_PERCENT = 50;

// ---------- VOC (kasar) ----------
const float V0 = 0.40f;       // tegangan udara bersih (kalibrasi)
float k_sensitivity = 50.0f;  // faktor kalibrasi

// ---------- NUS UUIDs ----------
static const char* UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* TX_CHAR_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // Notify/Read
static const char* RX_CHAR_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Write/WNR

// ---------- BLE state ----------
static uint16_t tx_char_handle = 0;    // value handle (TX)
static uint16_t rx_char_handle = 0;    // value handle (RX)
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static bool notify_enabled = false;

static char last_line[128];
static bool need_send_now = false;     // gate notif

// ---------- Utils ----------
static inline int readRawAveraged(int pin, int samples) {
  long acc = 0;
  for (int i = 0; i < samples; ++i) {
    acc += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(acc / samples);
}

static inline int rawToPercent(int raw, int wetRaw, int dryRaw) {
  if (wetRaw == dryRaw) return 0;
  long p = (wetRaw < dryRaw)
           ? -(long)(raw - dryRaw) * 100L / (wetRaw - dryRaw)
           :  (long)(raw - wetRaw) * 100L / (dryRaw - wetRaw);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (int)p;
}

// ---------- BLE callbacks ----------
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  if (status == BLE_STATUS_OK) {
    connection_handle = device->getHandle();
    digitalWrite(LED_PIN, HIGH);
    if (Serial) Serial.println("[BLE] Connected");
  }
}

// signature TANPA BLEStatus
void deviceDisconnectedCallback(BLEDevice *device) {
  (void)device;
  connection_handle = HCI_CON_HANDLE_INVALID;
  notify_enabled = false;
  need_send_now = false;
  digitalWrite(LED_PIN, LOW);
  if (Serial) Serial.println("[BLE] Disconnected");
}

// GATT Read (opsional)
uint16_t gattReadCallback(uint16_t value_handle, uint8_t *buffer, uint16_t buffer_size) {
  if (value_handle == tx_char_handle) {
    size_t n = strnlen(last_line, sizeof(last_line));
    if (buffer && buffer_size) {
      size_t m = (n < buffer_size) ? n : buffer_size;
      memcpy(buffer, last_line, m);
      return (uint16_t)m;
    }
    return (uint16_t)n;
  }
  return 0;
}

// CCCD & RX write
int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  const uint16_t tx_cccd = tx_char_handle + 1;   // CCCD = value_handle + 1
  if (value_handle == tx_cccd && size >= 2) {
    uint16_t cfg = buffer[0] | (buffer[1] << 8);
    notify_enabled = (cfg & 0x0001);
    if (Serial) Serial.printf("[BLE] Notify %s\n", notify_enabled ? "ENABLED" : "DISABLED");
    // Minta slot kirim pertama kali saat enable
    if (notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID) {
      need_send_now = true;
      att_server_request_can_send_now_event(connection_handle);
    }
    return 0;
  }
  if (value_handle == rx_char_handle) {
    String cmd; cmd.reserve(size);
    for (uint16_t i=0;i<size;i++) cmd += (char)buffer[i];
    cmd.trim();
    if (Serial) { Serial.print("[RX] "); Serial.println(cmd); }
    if (cmd.equalsIgnoreCase("LED=ON"))  digitalWrite(LED_PIN, HIGH);
    if (cmd.equalsIgnoreCase("LED=OFF")) digitalWrite(LED_PIN, LOW);
    return 0;
  }
  return 0;
}

// Low-level HCI handler untuk CAN_SEND_NOW
static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  (void)channel; (void)size;
  if (packet_type != HCI_EVENT_PACKET) return;
  uint8_t event = hci_event_packet_get_type(packet);
  if (event == ATT_EVENT_CAN_SEND_NOW && need_send_now && notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID) {
    // Coba kirim; kalau masih gagal, minta slot lagi
    int err = att_server_notify(connection_handle, tx_char_handle, (uint8_t*)last_line, (uint16_t)strlen(last_line));
    if (err) {
      att_server_request_can_send_now_event(connection_handle);
    } else {
      need_send_now = false;
    }
  }
}

static btstack_packet_callback_registration_t hci_cb;

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogReadResolution(12);

  if (Serial) Serial.println("\n== TBSense BLE Soil+VOC (NUS, safe) ==");

  // Register callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);

  // GATT Service + Chars
  BTstack.addGATTService(new UUID(UART_SERVICE_UUID));
  tx_char_handle = BTstack.addGATTCharacteristicDynamic(new UUID(TX_CHAR_UUID),
                      ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 0);
  rx_char_handle = BTstack.addGATTCharacteristicDynamic(new UUID(RX_CHAR_UUID),
                      ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 0);

  // HCI event handler for CAN_SEND_NOW
  hci_cb.callback = &hciPacketHandler;
  hci_add_event_handler(&hci_cb);

  // Power on & advertise
  BTstack.setup("TBSense");
  BTstack.startAdvertising();

  if (Serial) Serial.println("[BLE] Advertising as 'TBSense' (NUS)");
}

void loop() {
  BTstack.loop();

  static uint32_t last = 0;
  if (millis() - last < 300) return;
  last = millis();

  // --- VOC (A0) ---
  (void)analogRead(VOC_PIN);             // buang 1 sampel setelah switch channel
  delayMicroseconds(10);
  int vocRaw = readRawAveraged(VOC_PIN, AVG_SAMPLES);
  float vocVoltage = 3.30f * (vocRaw / 4095.0f);
  float voc_ppm = k_sensitivity * (vocVoltage - V0);
  if (voc_ppm < 0) voc_ppm = 0;

  if (Serial) {
    Serial.print("VOC raw="); Serial.print(vocRaw);
    Serial.print(" V=");      Serial.print(vocVoltage, 3);
    Serial.print(" ppm=");    Serial.println(voc_ppm, 1);
  }

  // --- Soil (A1) ---
  (void)analogRead(SOIL_PIN);            // buang 1 sampel setelah switch channel
  delayMicroseconds(10);
  const int raw = readRawAveraged(SOIL_PIN, AVG_SAMPLES);
  const int mv  = (int)((long)VREF_mV * raw / 4095L);
  const int pct = rawToPercent(raw, RAW_WET, RAW_DRY);
  const bool basah = (pct >= WET_THRESHOLD_PERCENT);

  digitalWrite(LED_PIN, basah ? HIGH : LOW);

  snprintf(last_line, sizeof(last_line),
           "%lu,RAW=%d,mV=%d,Moist=%d,%s,VOC=%.1fppm",
           (unsigned long)millis(), raw, mv, pct, basah ? "BASAH" : "KERING", voc_ppm);

  if (Serial) Serial.println(last_line);

  // Kirim aman via CAN_SEND_NOW gate
  if (notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID) {
    int err = att_server_notify(connection_handle, tx_char_handle,
                                (uint8_t*)last_line, (uint16_t)strlen(last_line));
    if (err) {
      need_send_now = true;
      att_server_request_can_send_now_event(connection_handle);
    } else {
      need_send_now = false;
    }
  }
}
