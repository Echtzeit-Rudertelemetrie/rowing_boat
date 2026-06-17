#include "espnow.h"

static uint8_t s_hub_mac[6]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool    s_peer_added       = false;
static uint8_t s_board_id_sender  = 0;

static constexpr uint8_t MAX_BOARD_ID = 255;
static volatile uint32_t s_total_raw_packets[MAX_BOARD_ID + 1] = {0};
static volatile uint32_t s_logical_packets[MAX_BOARD_ID + 1]   = {0};
static volatile uint32_t s_last_seq[MAX_BOARD_ID + 1]          = {0};
static volatile uint32_t s_max_seen_seq[MAX_BOARD_ID + 1]      = {0};
static volatile bool     s_first_seen[MAX_BOARD_ID + 1]        = {false};

void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    (void)status;
}

void onDataRecv(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (len != sizeof(RowingPacket)) return;

    RowingPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    uint8_t id = pkt.board_id;

    s_total_raw_packets[id]++;

    if (pkt.seq > s_max_seen_seq[id]) {
        s_max_seen_seq[id] = pkt.seq;
    }

    if (s_first_seen[id] && pkt.seq == s_last_seq[id]) {
        return;
    }

    s_last_seq[id] = pkt.seq;
    s_first_seen[id] = true;
    s_logical_packets[id]++;
}

uint8_t* espnow_get_local_mac() {
    static uint8_t mac[6];
    WiFi.macAddress(mac);
    return mac;
}

void espnow_set_hub_mac(const uint8_t* mac) {
    memcpy(s_hub_mac, mac, 6);
}

bool espnow_is_peer_added() {
    return s_peer_added;
}

void espnow_init_receiver() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    esp_wifi_set_max_tx_power(78);

    if (esp_now_init() != ESP_OK) {
        ESP.restart();
    }
    esp_now_register_recv_cb(onDataRecv);
}

void espnow_init_sender(uint8_t board_id, const uint8_t* peer_mac) {
    s_board_id_sender = board_id;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_max_tx_power(78);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FEHLER: esp_now_init");
        ESP.restart();
    }
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) == ESP_OK) {
        s_peer_added = true;
        memcpy(s_hub_mac, peer_mac, 6);
    } else {
        Serial.println("FEHLER: esp_now_add_peer");
    }
}

esp_err_t espnow_send(const RowingPacket* pkt) {
    if (!s_peer_added) return ESP_FAIL;
    return esp_now_send(s_hub_mac, (const uint8_t*)pkt, sizeof(RowingPacket));
}

void espnow_print_stats() {
    for (uint8_t id = 1; id < MAX_BOARD_ID; id++) {
        if (!s_first_seen[id]) continue;

        uint32_t exp_logical  = s_max_seen_seq[id] + 1;
        uint32_t recv_logical = s_logical_packets[id];
        uint32_t lost_logical = exp_logical - recv_logical;
        float logical_success = 100.0f * recv_logical / exp_logical;

        uint32_t exp_raw  = exp_logical * PACKET_RETRIES;
        uint32_t recv_raw = s_total_raw_packets[id];
        uint32_t lost_raw = exp_raw - recv_raw;
        float raw_success = 100.0f * recv_raw / exp_raw;

        Serial.printf("Board %u: Logische Pakete: %u / %u (%.1f%%) | "
                      "Funkpakete: %u / %u (%.1f%%) | "
                      "Verlust logisch: %u | Verlust Funk: %u\n",
                      id, recv_logical, exp_logical, logical_success,
                      recv_raw, exp_raw, raw_success,
                      lost_logical, lost_raw);

        s_total_raw_packets[id] = 0;
        s_logical_packets[id]   = 0;
        s_last_seq[id]          = 0;
        s_max_seen_seq[id]      = 0;
        s_first_seen[id]        = false;
    }
}