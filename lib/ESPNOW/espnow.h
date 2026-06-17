#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define PACKET_VALUES  32
#define PACKET_RETRIES 3
#define ESPNOW_CHANNEL 11

#define BOARD_ID_SENDER   1
#define BOARD_ID_RECEIVER 0

#define ESPNOW_MAX_BOARD_ID 255

typedef struct __attribute__((packed)) {
    uint8_t   board_id;
    uint32_t  seq;
    uint16_t  force[PACKET_VALUES];
    uint16_t  angle[PACKET_VALUES];
} RowingPacket;

void      espnow_init_sender(uint8_t board_id, const uint8_t* peer_mac);
void      espnow_init_receiver();
uint8_t*  espnow_get_local_mac();
void      espnow_set_hub_mac(const uint8_t* mac);
esp_err_t espnow_send(const RowingPacket* pkt);
bool      espnow_is_peer_added();
void      espnow_print_stats();

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
void onDataSent(const uint8_t* mac, esp_now_send_status_t status);