#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

// WICHTIG: Diese Werte müssen exakt denen aus dem Sender entsprechen!
static constexpr uint8_t ESPNOW_CHANNEL = 11;
static constexpr uint8_t PACKET_VALUES = 10; // Bitte mit deinem AppTypes.h Wert abgleichen!

// Das exakt gleiche Struct wie beim Sender definieren
typedef struct __attribute__((packed)) {
  uint32_t espIdAndSequenceNum;
  uint16_t force_values[PACKET_VALUES];
  uint16_t angle_values[PACKET_VALUES];
} MeasurementPack;

static void printMacAddress(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void handlePacket(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.print("RX von ");
  printMacAddress(mac);
  Serial.printf(" | len=%d\n", len);

  // Prüfen, ob die Länge genau der Größe unseres Structs entspricht
  if (len != sizeof(MeasurementPack)) {
    Serial.printf("FEHLER: Falsche Paketlänge. Erwartet: %u, Erhalten: %d\n", sizeof(MeasurementPack), len);
    return;
  }

  // Daten elegant in das Struct casten
  const MeasurementPack* pkt = reinterpret_cast<const MeasurementPack*>(incomingData);

  // Werte auslesen
  const uint8_t  espId = static_cast<uint8_t>((pkt->espIdAndSequenceNum >> 29) & 0x07u);
  const uint32_t seq   = pkt->espIdAndSequenceNum & 0x1FFFFFFFu;

  Serial.printf("=== Empfangenes Paket ===\n");
  Serial.printf("ESP-ID: %u | Sequenz: %lu\n", espId, static_cast<unsigned long>(seq));

  // Force Arrays ausgeben
  Serial.print("force_values: [");
  for(int i = 0; i < PACKET_VALUES; i++) {
    Serial.print(pkt->force_values[i]);
    if(i < PACKET_VALUES - 1) Serial.print(", ");
  }
  Serial.println("]");

  // Angle Arrays ausgeben
  Serial.print("angle_values: [");
  for(int i = 0; i < PACKET_VALUES; i++) {
    Serial.print(pkt->angle_values[i]);
    if(i < PACKET_VALUES - 1) Serial.print(", ");
  }
  Serial.println("]\n");
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (recv_info == nullptr || recv_info->src_addr == nullptr || incomingData == nullptr || len <= 0) return;
  handlePacket(recv_info->src_addr, incomingData, len);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) return;
  handlePacket(mac, incomingData, len);
}
#endif

static void initEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false); // Sehr gut für ESP-NOW Empfang

  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("FEHLER: Kanal setzen fehlgeschlagen");
  }

  // WICHTIG: Wenn du im Sender WIFI_PROTOCOL_11B nutzt, musst du es HIER auch setzen!
  // Ich empfehle aber, es auf BEIDEN Seiten wegzulassen. Falls nötig einkommentieren:
  // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  if (esp_now_init() != ESP_OK) {
    Serial.println("FEHLER: esp_now_init() fehlgeschlagen");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW Empfänger initialisiert.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starte Empfänger...");
  initEspNowReceiver();
}

void loop() {
  delay(1000);
}