#include "DataSender.h"

constexpr std::uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

std::uint16_t forceBuffer[PACKET_VALUES]{};
std::uint16_t angleBuffer[PACKET_VALUES]{};
std::uint8_t bufferIndex = 0;
std::uint32_t packetSeq = 0;

std::uint8_t hubMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool peerAdded = false;

DataSender::DataSender(MeasurementData& data)
: data_(&data) {
}

void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    (void)status;
}

void DataSender::espnow_init_sender(std::uint8_t /*board_id*/, const std::uint8_t* peerMac) {
    //s_board_id_sender = board_id; -> brauch ich das?
    WiFi.mode(WIFI_STA); //-> Stationary Funkteilnehmer und nicht access point
    WiFi.disconnect(); //Kein Wlan an

    esp_wifi_set_max_tx_power(78); //Setzt die maximale Sendeleistung 
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE); //Channel auf channel 11 setzen
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); //Verbindungsprotokoll setzen (nicht ändern)

    if (esp_now_init() != ESP_OK) { //initialisiert den ESP-Stack
        Serial.println("FEHLER: esp_now_init");
        ESP.restart();
    }
    esp_now_register_send_cb(onDataSent); //Nach jedem Sendeversuch ruft das System automatisch onDataSent auf (macht nichts -> weglassen?)

    esp_now_peer_info_t peer = {}; // Erstellt eine Peer-Konfigurationsstruktur. 
    //Ein „Peer“ ist bei ESP-NOW der Gegenknoten, den man ansprechen will.
    memcpy(peer.peer_addr, peerMac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA; //passt das?

    if (esp_now_add_peer(&peer) == ESP_OK) { //fügt dem Empfänger als Peer hinzu
        peerAdded = true;
        memcpy(hubMac, peerMac, 6);
    } else {
        Serial.println("FEHLER: esp_now_add_peer");
    }
}

  std::uint16_t quantize(float value, float minValue, float maxValue, std::uint8_t bits) {
    const std::uint32_t maxInt = (1UL << bits) - 1UL;

    if (value <= minValue) {
        return 0;
    }
    if (value >= maxValue) {
        return static_cast<std::uint16_t>(maxInt);
    }

    const float normalized = (value - minValue) / (maxValue - minValue);
    return static_cast<std::uint16_t>(normalized * maxInt + 0.5f);
  }

esp_err_t espnow_send(const MeasurementPack* pkt) { //sendet paket an alle (weil s_hub_mac als broadcast definiert ist)
    if (!peerAdded)
    {
      return ESP_FAIL;
    }
    return esp_now_send(hubMac, reinterpret_cast<const std::uint8_t*>(pkt), sizeof(MeasurementPack));
}

void DataSender::sendData() {
    if (data_ == nullptr) {
        return;
    }

    forceBuffer[bufferIndex] = quantize(
        data_->forceSensor,
        FORCE_MIN_N,
        FORCE_MAX_N,
        static_cast<std::uint8_t>(sizeof(forceBuffer[0]) * CHAR_BIT)
    );

    angleBuffer[bufferIndex] = quantize(
        data_->degreeSensor,
        ANGLE_MIN_DEG,
        ANGLE_MAX_DEG,
        static_cast<std::uint8_t>(sizeof(angleBuffer[0]) * CHAR_BIT)
    );

    ++bufferIndex;

    if (bufferIndex < PACKET_VALUES) {
        return;
    }

    ++packetSeq;

    if ((ESP_ID & ~0x07u) != 0) {
        // Fehlerbehandlung: ESP_ID ist außerhalb des 3-Bit-Bereichs
    }

    if ((packetSeq & ~0x1FFFFFFFu) != 0) {
        // Fehlerbehandlung: Sequenznummer ist außerhalb des 29-Bit-Bereichs
    }

    MeasurementPack pkt{};
    pkt.espIdAndSeqenceNum = (static_cast<std::uint32_t>(ESP_ID) << 29) | packetSeq;

    memcpy(pkt.force_values, forceBuffer, sizeof(forceBuffer));
    memcpy(pkt.angle_values, angleBuffer, sizeof(angleBuffer));

    // Nur senden, wenn ESP-NOW initialisiert und ein Peer hinzugefuegt wurde.
    // Sonst stuerzt esp_now_send() auf dem uninitialisierten Stack ab (LoadProhibited).
    if (peerAdded) {
        for (std::uint8_t retry = 0; retry < PACKET_RETRIES; ++retry) {
            esp_now_send(hubMac, reinterpret_cast<const std::uint8_t*>(&pkt), sizeof(pkt));
        }
    }

    bufferIndex = 0;

    Serial.print("Sending Data");
}