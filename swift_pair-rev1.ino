// Raspberry Pi Pico 2 W — Soil + VOC + BLE NUS (safe notify)
// Core: Earle Philhower (arduino-pico)
// Tools → IP/Bluetooth Stack: pico_w_bluetooth

#include <Arduino.h>
#include <BTstackLib.h>
#include "btstack.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Hardware Pins
namespace Pin {
  const int VOC   = A0;        // GP26 / ADC0
  const int SOIL  = A1;        // GP27 / ADC1
  const int LED   = LED_BUILTIN;
}

// Soil Sensor Calibration
namespace SoilConfig {
  const int RAW_DRY = 3200;    // Update after calibration
  const int RAW_WET = 1400;    // Update after calibration
  const int AVG_SAMPLES = 10;
  const int VREF_mV = 3300;
  const int WET_THRESHOLD_PERCENT = 50;
}

// VOC Sensor Calibration
namespace VOCConfig {
  const float V0 = 0.40f;      // Clean air voltage (calibration baseline)
  const float K_SENSITIVITY = 50.0f;  // Calibration factor
}

// BLE Timing
namespace Timing {
  const uint32_t UPDATE_INTERVAL_MS = 300;
  const uint16_t ADC_SETTLE_US = 10;
  const uint16_t ADC_SAMPLE_DELAY_US = 200;
}

// NUS (Nordic UART Service) UUIDs
namespace NUS {
  const char* SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
  const char* TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // Notify/Read
  const char* RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // Write/WNR
}

// ============================================================================
// BLE STATE
// ============================================================================

struct BLEState {
  uint16_t tx_char_handle = 0;
  uint16_t rx_char_handle = 0;
  hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
  bool notify_enabled = false;
  bool need_send_now = false;
  char last_message[128] = {0};
};

static BLEState ble;
static btstack_packet_callback_registration_t hci_cb;

// ============================================================================
// SENSOR READING
// ============================================================================

class SensorReader {
public:
  static int readRawAveraged(int pin, int samples) {
    long accumulator = 0;
    for (int i = 0; i < samples; ++i) {
      accumulator += analogRead(pin);
      delayMicroseconds(Timing::ADC_SAMPLE_DELAY_US);
    }
    return (int)(accumulator / samples);
  }

  static int mapToPercent(int raw, int wetRaw, int dryRaw) {
    if (wetRaw == dryRaw) return 0;
    
    long percent = (wetRaw < dryRaw)
      ? -(long)(raw - dryRaw) * 100L / (wetRaw - dryRaw)
      :  (long)(raw - wetRaw) * 100L / (dryRaw - wetRaw);
    
    return constrain(percent, 0, 100);
  }
};

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

struct VOCReading {
  int raw;
  float voltage;
  float ppm;

  void read() {
    analogRead(Pin::VOC);  // Discard first sample after channel switch
    delayMicroseconds(Timing::ADC_SETTLE_US);
    
    raw = SensorReader::readRawAveraged(Pin::VOC, SoilConfig::AVG_SAMPLES);
    voltage = 3.30f * (raw / 4095.0f);
    ppm = VOCConfig::K_SENSITIVITY * (voltage - VOCConfig::V0);
    if (ppm < 0) ppm = 0;
  }

  void print() const {
    if (!Serial) return;
    Serial.print("VOC raw="); Serial.print(raw);
    Serial.print(" V=");      Serial.print(voltage, 3);
    Serial.print(" ppm=");    Serial.println(ppm, 1);
  }
};

struct SoilReading {
  int raw;
  int millivolts;
  int percent;
  bool wet;

  void read() {
    analogRead(Pin::SOIL);  // Discard first sample after channel switch
    delayMicroseconds(Timing::ADC_SETTLE_US);
    
    raw = SensorReader::readRawAveraged(Pin::SOIL, SoilConfig::AVG_SAMPLES);
    millivolts = (int)((long)SoilConfig::VREF_mV * raw / 4095L);
    percent = SensorReader::mapToPercent(raw, SoilConfig::RAW_WET, SoilConfig::RAW_DRY);
    wet = (percent >= SoilConfig::WET_THRESHOLD_PERCENT);
  }
};

// ============================================================================
// BLE NOTIFICATION HANDLER
// ============================================================================

class BLENotifier {
public:
  static void send(const char* message) {
    strncpy(ble.last_message, message, sizeof(ble.last_message) - 1);
    ble.last_message[sizeof(ble.last_message) - 1] = '\0';

    if (!ble.notify_enabled || ble.connection_handle == HCI_CON_HANDLE_INVALID) {
      return;
    }

    int err = att_server_notify(
      ble.connection_handle, 
      ble.tx_char_handle,
      (uint8_t*)ble.last_message, 
      (uint16_t)strlen(ble.last_message)
    );

    if (err) {
      ble.need_send_now = true;
      att_server_request_can_send_now_event(ble.connection_handle);
    } else {
      ble.need_send_now = false;
    }
  }
};

// ============================================================================
// BLE CALLBACKS
// ============================================================================

void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  if (status == BLE_STATUS_OK) {
    ble.connection_handle = device->getHandle();
    digitalWrite(Pin::LED, HIGH);
    if (Serial) Serial.println("[BLE] Connected");
  }
}

void deviceDisconnectedCallback(BLEDevice *device) {
  (void)device;
  ble.connection_handle = HCI_CON_HANDLE_INVALID;
  ble.notify_enabled = false;
  ble.need_send_now = false;
  digitalWrite(Pin::LED, LOW);
  if (Serial) Serial.println("[BLE] Disconnected");
}

uint16_t gattReadCallback(uint16_t value_handle, uint8_t *buffer, uint16_t buffer_size) {
  if (value_handle == ble.tx_char_handle) {
    size_t length = strnlen(ble.last_message, sizeof(ble.last_message));
    if (buffer && buffer_size) {
      size_t copy_size = min(length, (size_t)buffer_size);
      memcpy(buffer, ble.last_message, copy_size);
      return (uint16_t)copy_size;
    }
    return (uint16_t)length;
  }
  return 0;
}

int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  const uint16_t tx_cccd = ble.tx_char_handle + 1;
  
  // Handle CCCD (notification enable/disable)
  if (value_handle == tx_cccd && size >= 2) {
    uint16_t config = buffer[0] | (buffer[1] << 8);
    ble.notify_enabled = (config & 0x0001);
    
    if (Serial) {
      Serial.printf("[BLE] Notify %s\n", ble.notify_enabled ? "ENABLED" : "DISABLED");
    }
    
    if (ble.notify_enabled && ble.connection_handle != HCI_CON_HANDLE_INVALID) {
      ble.need_send_now = true;
      att_server_request_can_send_now_event(ble.connection_handle);
    }
    return 0;
  }
  
  // Handle RX characteristic (commands)
  if (value_handle == ble.rx_char_handle) {
    String command;
    command.reserve(size);
    for (uint16_t i = 0; i < size; i++) {
      command += (char)buffer[i];
    }
    command.trim();
    
    if (Serial) {
      Serial.print("[RX] ");
      Serial.println(command);
    }
    
    // Process commands
    if (command.equalsIgnoreCase("LED=ON")) {
      digitalWrite(Pin::LED, HIGH);
    } else if (command.equalsIgnoreCase("LED=OFF")) {
      digitalWrite(Pin::LED, LOW);
    }
    
    return 0;
  }
  
  return 0;
}

static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;
  
  if (packet_type != HCI_EVENT_PACKET) return;
  
  uint8_t event = hci_event_packet_get_type(packet);
  if (event == ATT_EVENT_CAN_SEND_NOW && 
      ble.need_send_now && 
      ble.notify_enabled && 
      ble.connection_handle != HCI_CON_HANDLE_INVALID) {
    
    int err = att_server_notify(
      ble.connection_handle, 
      ble.tx_char_handle,
      (uint8_t*)ble.last_message, 
      (uint16_t)strlen(ble.last_message)
    );
    
    if (err) {
      att_server_request_can_send_now_event(ble.connection_handle);
    } else {
      ble.need_send_now = false;
    }
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void setupHardware() {
  pinMode(Pin::LED, OUTPUT);
  digitalWrite(Pin::LED, LOW);
  analogReadResolution(12);
}

void setupBLE() {
  // Register callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);

  // Create GATT service and characteristics
  BTstack.addGATTService(new UUID(NUS::SERVICE_UUID));
  
  ble.tx_char_handle = BTstack.addGATTCharacteristicDynamic(
    new UUID(NUS::TX_CHAR_UUID),
    ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY, 
    0
  );
  
  ble.rx_char_handle = BTstack.addGATTCharacteristicDynamic(
    new UUID(NUS::RX_CHAR_UUID),
    ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, 
    0
  );

  // Register HCI event handler
  hci_cb.callback = &hciPacketHandler;
  hci_add_event_handler(&hci_cb);

  // Start BLE
  BTstack.setup("TBSense");
  BTstack.startAdvertising();
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  
  if (Serial) {
    Serial.println("\n== TBSense BLE Soil+VOC (NUS, safe) ==");
  }

  setupHardware();
  setupBLE();

  if (Serial) {
    Serial.println("[BLE] Advertising as 'TBSense' (NUS)");
  }
}

void loop() {
  BTstack.loop();

  static uint32_t last_update = 0;
  if (millis() - last_update < Timing::UPDATE_INTERVAL_MS) {
    return;
  }
  last_update = millis();

  // Read sensors
  VOCReading voc;
  voc.read();
  voc.print();

  SoilReading soil;
  soil.read();

  // Update LED based on soil moisture
  digitalWrite(Pin::LED, soil.wet ? HIGH : LOW);

  // Format and send data
  char message[128];
  snprintf(message, sizeof(message),
    "%lu,RAW=%d,mV=%d,Moist=%d,%s,VOC=%.1fppm",
    (unsigned long)millis(), 
    soil.raw, 
    soil.millivolts, 
    soil.percent,
    soil.wet ? "BASAH" : "KERING", 
    voc.ppm
  );

  if (Serial) {
    Serial.println(message);
  }

  BLENotifier::send(message);
}