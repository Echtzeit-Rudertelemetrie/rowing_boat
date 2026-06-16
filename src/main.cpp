#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

static constexpr uint8_t ESPNOW_CHANNEL = 11;

static void printMacAddress(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void printU16Array(const uint8_t *data, int offsetBytes, int count) {
  for (int i = 0; i < count; ++i) {
    uint16_t value = 0;
    memcpy(&value, data + offsetBytes + i * sizeof(uint16_t), sizeof(value));
    Serial.printf("%u", value);
    if (i + 1 < count) {
      Serial.print(", ");
    }
  }
}

static void dumpRawHex(const uint8_t *data, int len) {
  for (int i = 0; i < len; ++i) {
    Serial.printf("%02X", data[i]);
    if (i + 1 < len) {
      Serial.print(" ");
    }
  }
}

static void handlePacket(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("RX von ");
  printMacAddress(mac);
  Serial.printf(" | len=%d | ", len);

  if (len < static_cast<int>(sizeof(uint32_t))) {
    Serial.println("FEHLER: Paket zu kurz");
    return;
  }

  uint32_t header = 0;
  memcpy(&header, incomingData, sizeof(header));

  const uint8_t  espId = static_cast<uint8_t>((header >> 29) & 0x07u);
  const uint32_t seq   = header & 0x1FFFFFFFu;

  Serial.printf("espId=%u | seq=%lu", espId, static_cast<unsigned long>(seq));

  const int payloadBytes = len - static_cast<int>(sizeof(uint32_t));

  if (payloadBytes <= 0 || (payloadBytes % 4) != 0) {
    Serial.println(" | FEHLER: Unerwartete Paketlaenge");
    Serial.print("Raw: ");
    dumpRawHex(incomingData, len);
    Serial.println();
    return;
  }

  const int valueCount = payloadBytes / 4; // force[] + angle[] je uint16_t => 4 Bytes pro Index

  Serial.printf(" | Werte pro Array=%d\n", valueCount);

  Serial.print("  force_values : [");
  printU16Array(incomingData, 4, valueCount);
  Serial.println("]");

  Serial.print("  angle_values : [");
  printU16Array(incomingData, 4 + valueCount * 2, valueCount);
  Serial.println("]");
  Serial.println();
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (recv_info == nullptr || recv_info->src_addr == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }
  handlePacket(recv_info->src_addr, incomingData, len);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }
  handlePacket(mac, incomingData, len);
}
#endif

static void initEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("FEHLER: esp_wifi_set_channel() fehlgeschlagen");
  }

  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
    Serial.println("FEHLER: esp_wifi_set_ps(WIFI_PS_NONE) fehlgeschlagen");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("FEHLER: esp_now_init() fehlgeschlagen");
    ESP.restart();
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("FEHLER: esp_now_register_recv_cb() fehlgeschlagen");
    ESP.restart();
  }

  Serial.println("ESP-NOW Empfaenger initialisiert.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("Starte ESP32 ESP-NOW Broadcast-Receiver...");
  initEspNowReceiver();
}

void loop() {
  delay(1000);
}