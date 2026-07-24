#include "DataSender.h"

// Debug-Vollausgabe pro Paket (~900 Zeichen) laeuft im selben Task wie der
// 100-Hz-EKF: liest kein Host den USB-CDC-Puffer, blockiert Serial die
// Sample-Schleife und die Event-Queue (~210 ms Kapazitaet) laeuft ueber.
// Deshalb standardmaessig aus. Einschalten ohne Code-Aenderung: env
// "xiao_s3_debug" in platformio.ini bauen (setzt -D DATASENDER_DEBUG_DUMP=1).
#ifndef DATASENDER_DEBUG_DUMP
#define DATASENDER_DEBUG_DUMP 0
#endif

#ifndef DATASENDER_TELEPLOT
#define DATASENDER_TELEPLOT 0
#endif

constexpr std::uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

std::uint16_t forceBuffer[PACKET_VALUES]{};
std::uint16_t angleBuffer[PACKET_VALUES]{};
std::uint8_t bufferIndex = 0;
std::uint32_t packetSeq = 0;

#if DATASENDER_DEBUG_DUMP
// Nur fuer den Serial-Dump: haelt die physikalischen Werte (N / Grad) VOR
// der Quantisierung fest, damit der Dump menschenlesbar ist und nicht die
// rohen uint16-Codes aus forceBuffer/angleBuffer zeigt. Das gesendete
// Paket (pkt) bleibt davon unberuehrt -> gleiche Daten wie ohne Debug-Dump.
float forceFloatBuffer[PACKET_VALUES]{};
float angleFloatBuffer[PACKET_VALUES]{};
#endif

std::uint8_t hubMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool peerAdded = false;

DataSender::DataSender(MeasurementData &data)
    : data_(&data)
{
}

// static: lib/ESPNOW (von test_sender/test_receiver genutzt) definiert ein
// gleichnamiges onDataSent — ohne static gibt das einen Linker-Konflikt,
// sobald beide Libs im selben Build landen.
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
#if DATASENDER_DEBUG_DUMP
    // Nur im Debug-Modus: der Callback laeuft im WiFi-Task (Core 0) — Serial
    // von dort konkurriert mit dem Sample-Task um den CDC-Puffer.
    Serial.print("ESP-NOW Sende-Status: ");
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Erfolgreich gesendet (ACK erhalten oder Broadcast rausgegangen)");
    }
    else
    {
        Serial.println("FEHLER: Zustellung fehlgeschlagen (Empfänger nicht erreichbar/falscher Kanal)");
    }
#else
    (void)mac;
    (void)status;
#endif
}

#if DATASENDER_DEBUG_DUMP
// static: gleicher Namenskonflikt mit lib/ESPNOW wie bei onDataSent.
// Nur vom Debug-Dump genutzt — ohne den waere es eine unused-function-Warnung.
static uint8_t *espnow_get_local_mac()
{
    static uint8_t mac[6]; //-> durch static bleibt adresse auch im nachhinein noch gültig (nachem function fertig)
    WiFi.macAddress(mac);
    return mac;
}
#endif // DATASENDER_DEBUG_DUMP

void DataSender::espnow_init_sender()
{
    // s_board_id_sender = board_id; -> brauch ich das?
    WiFi.mode(WIFI_STA); //-> Stationary Funkteilnehmer und nicht access point
    WiFi.disconnect();   // Kein Wlan an

    esp_wifi_set_max_tx_power(78); // Setzt die maximale Sendeleistung
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE); // Channel auf channel 11 setzen
    esp_wifi_set_promiscuous(false);
    // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B); //Verbindungsprotokoll setzen (nicht ändern)

    if (esp_now_init() != ESP_OK)
    { // initialisiert den ESP-Stack
        Serial.println("FEHLER: esp_now_init");
        ESP.restart();
    }
    esp_now_register_send_cb(onDataSent); // Nach jedem Sendeversuch ruft das System automatisch onDataSent auf (macht nichts -> weglassen?)

    esp_now_peer_info_t peer = {}; // Erstellt eine Peer-Konfigurationsstruktur.
    // Ein „Peer“ ist bei ESP-NOW der Gegenknoten, den man ansprechen will.
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA; // passt das?

    if (esp_now_add_peer(&peer) == ESP_OK)
    { // fügt dem Empfänger als Peer hinzu
        peerAdded = true;
        memcpy(hubMac, BROADCAST_MAC, 6);
    }
    else
    {
        Serial.println("FEHLER: esp_now_add_peer");
    }
}

// invSpan = vorberechneter Kehrwert von (maxValue - minValue), siehe
// DataSender.h: Multiplikation statt Division im 100-Hz-Pfad.
std::uint16_t quantize(float value, float minValue, float maxValue, float invSpan, std::uint8_t bits)
{
    const std::uint32_t maxInt = (1UL << bits) - 1UL;

    if (value <= minValue)
    {
        return 0;
    }
    if (value >= maxValue)
    {
        return static_cast<std::uint16_t>(maxInt);
    }

    const float normalized = (value - minValue) * invSpan;
    return static_cast<std::uint16_t>(normalized * maxInt + 0.5f);
}

//     std::uint16_t quantizeAngle(float value, std::uint8_t bits) {
//     const std::uint32_t maxInt = (1UL << bits) - 1UL;

//     return static_cast<std::uint16_t>(value * maxInt + 0.5f);
//   }

// Hier gab es frueher einen espnow_send(MeasurementPack*)-Wrapper — entfernt,
// weil toter Code: sendData() ruft esp_now_send() direkt auf, der Wrapper
// wurde nirgends aufgerufen.
void DataSender::sendData()
{
    if (data_ == nullptr)
    {
        return;
    }

#if DATASENDER_DEBUG_DUMP
    forceFloatBuffer[bufferIndex] = data_->forceSensor;
    angleFloatBuffer[bufferIndex] = data_->degreeSensor;
#endif

    forceBuffer[bufferIndex] = quantize(
        data_->forceSensor,
        FORCE_MIN_N,
        FORCE_MAX_N,
        FORCE_INV_SPAN,
        static_cast<std::uint8_t>(sizeof(forceBuffer[0]) * CHAR_BIT));

    angleBuffer[bufferIndex] = quantize(
        data_->degreeSensor,
        ANGLE_MIN_DEG,
        ANGLE_MAX_DEG,
        ANGLE_INV_SPAN,
        static_cast<std::uint8_t>(sizeof(angleBuffer[0]) * CHAR_BIT));

    ++bufferIndex;

    if (bufferIndex < PACKET_VALUES)
    {
        return;
    }

    ++packetSeq;

    if ((ESP_ID & ~IDSEQ_ID_MASK) != 0)
    {
        // Fehlerbehandlung: ESP_ID ist außerhalb des 4-Bit-Bereichs
    }

    if ((packetSeq & ~IDSEQ_SEQ_MASK) != 0)
    {
        // Fehlerbehandlung: Sequenznummer ist außerhalb des 28-Bit-Bereichs
    }

    MeasurementPack pkt{};
    pkt.espIdAndSeqenceNum = packIdSeq(ESP_ID, packetSeq);

    memcpy(pkt.force_values, forceBuffer, sizeof(forceBuffer));
    memcpy(pkt.angle_values, angleBuffer, sizeof(angleBuffer));

    // Nur senden, wenn ESP-NOW initialisiert und ein Peer hinzugefuegt wurde.
    // Sonst stuerzt esp_now_send() auf dem uninitialisierten Stack ab (LoadProhibited).
    if (peerAdded)
    {
        for (std::uint8_t retry = 0; retry < PACKET_RETRIES; ++retry)
        {
            esp_now_send(hubMac, reinterpret_cast<const std::uint8_t *>(&pkt), sizeof(pkt));
        }
    }
    else
    {
        Serial.println("Peer was not added");
    }

#if DATASENDER_TELEPLOT
    // Optional Teleplot output. Keep disabled in production: USB-CDC writes can
    // block the 100-Hz task when no monitor consumes them.
    for (int i = 0; i < PACKET_VALUES; ++i)
    {
        const float deg =
            angleBuffer[i] * ((ANGLE_MAX_DEG - ANGLE_MIN_DEG) / 65535.0f) +
            ANGLE_MIN_DEG;
        Serial.printf(">winkel_q:%.2f\n", deg);
    }
#endif

#if DATASENDER_DEBUG_DUMP
    // peerAdded=false: esp_now_send() lief oben gar nicht -> Paketinhalt ist
    // trotzdem interessant (zeigt, was gesendet WORDEN WAERE), muss aber klar
    // als nicht abgeschickt markiert sein statt wie unten "Sending Data".
    Serial.println(peerAdded ? "Sending Data" : "NOT sent (no peer) - packet content below anyway:");
    Serial.println("=== Measurement Pack ===");

    // 1. Ausgabe der Force-Werte (physikalisch, N -- nicht die gesendeten
    // quantisierten Codes, siehe forceFloatBuffer oben)
    Serial.print("Force Values [N]: [");
    for (int i = 0; i < PACKET_VALUES; i++)
    {
        Serial.print(forceFloatBuffer[i], 2);
        if (i < PACKET_VALUES - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println("]");

    // Serial.println(angleFloatBuffer[0], 2);

    Serial.print("Angle Values [deg]: [");
    for (int i = 0; i < PACKET_VALUES; i++)
    {
        Serial.print(angleFloatBuffer[i], 2);
        if (i < PACKET_VALUES - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println("]");

    // 3. Ausgabe der ID und Sequenznummer
    Serial.print("Raw ID & Seq: ");
    Serial.println(pkt.espIdAndSeqenceNum);

    // 4 Bit ID | 28 Bit Sequenz (siehe AppTypes.h). Die Sequenz vorher auf
    // 16 Bit zu maskieren hat sie ab 65536 falsch angezeigt.
    uint8_t espId = idFromIdSeq(pkt.espIdAndSeqenceNum);
    uint32_t seqNum = seqFromIdSeq(pkt.espIdAndSeqenceNum);

    Serial.print(" -> Extracted ESP-ID: ");
    Serial.println(espId);
    Serial.print(" -> Extracted Sequence: ");
    Serial.println(seqNum);

    Serial.println("========================");
    Serial.println("Mac Adress");
    uint8_t *mac = espnow_get_local_mac();

    Serial.printf("MAC-Adresse: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Serial.println();
    Serial.println();
#endif // DATASENDER_DEBUG_DUMP

    bufferIndex = 0;
}
