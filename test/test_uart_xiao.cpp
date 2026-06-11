// ============================================================================
// UART-Bruecke ESP32 <-> XIAO nRF52840  (XIAO-Seite)
// ----------------------------------------------------------------------------
// Board : Seeed XIAO nRF52840 (Sense) (env: test_uart_xiao)
//         Arduino-Board "Seeed nRF52 Boards" (non-mbed) -> Adafruit_TinyUSB
//         noetig, sonst kompiliert/funktioniert Serial1 nicht.
// UART  : Serial1 (Hardware-UART)
//   D6 = TX -> ESP32 GPIO16 (RX2)
//   D7 = RX <- ESP32 GPIO17 (TX2)
//   GND <-> ESP32 GND
//
// Test: jede Sekunde "Hello World" -> Serial1 senden und pruefen, ob von der
//       Gegenseite etwas ankommt. Ausgabe EINE Statuszeile pro Sekunde:
//         >>> UART OK  <<<   wenn kuerzlich empfangen
//         !!! UART FAIL !!!  wenn nichts ankommt
// ============================================================================
#include <Adafruit_TinyUSB.h>   // MUSS vor Serial.begin() stehen (non-mbed BSP)
#include <Arduino.h>

constexpr uint32_t UART_BAUD      = 115200;
constexpr uint32_t SEND_PERIOD_MS = 1000;
constexpr uint32_t RX_TIMEOUT_MS  = 2500;   // so lange gilt der Link als "OK"

void setup() {
    Serial.begin(UART_BAUD);                 // USB-Konsole
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) { /* auf USB warten, max 3s */ }

    Serial1.begin(UART_BAUD);                // UART zum ESP32 (D6=TX, D7=RX)

    Serial.println();
    Serial.println("=== XIAO bereit (UART1-Test) ===");
    Serial.printf("Serial1: TX=D6 -> ESP GPIO16,  RX=D7 <- ESP GPIO17  @ %u 8N1\n", UART_BAUD);
    Serial.println("Statuszeile 1x/Sekunde: >>> UART OK <<< oder !!! UART FAIL !!!\n");
}

void loop() {
    static uint32_t lastSend  = 0;
    static uint32_t txSeq     = 0;
    static uint32_t rxCount   = 0;
    static uint32_t lastRxMs  = 0;
    static bool     everRx    = false;
    static String   rxLine;
    static String   lastMsg;

    uint32_t now = millis();

    // Empfang einsammeln (zeilenweise)
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            lastMsg  = rxLine;
            rxLine   = "";
            rxCount++;
            lastRxMs = now;
            everRx   = true;
        } else if (c != '\r') {
            rxLine += c;
        }
    }

    // 1x pro Sekunde senden + Status ausgeben
    if (now - lastSend >= SEND_PERIOD_MS) {
        lastSend = now;
        Serial1.printf("Hello World from XIAO #%u\n", txSeq);

        bool linkOk = everRx && (now - lastRxMs <= RX_TIMEOUT_MS);
        if (linkOk) {
            Serial.printf(">>> UART OK  <<<  TX #%u | RX=%u | letzte: \"%s\" (vor %u ms)\n",
                          txSeq, rxCount, lastMsg.c_str(), now - lastRxMs);
        } else if (everRx) {
            Serial.printf("!!! UART FAIL !!! TX #%u | RX=%u | seit %u ms nichts empfangen\n",
                          txSeq, rxCount, now - lastRxMs);
        } else {
            Serial.printf("!!! UART FAIL !!! TX #%u | RX=0  | noch NIE empfangen -> Verkabelung pruefen\n",
                          txSeq);
        }
        txSeq++;
    }
}
