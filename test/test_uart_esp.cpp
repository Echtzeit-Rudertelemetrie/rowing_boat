// ============================================================================
// UART-Bruecke ESP32 <-> XIAO nRF52840  (ESP32-Seite)
// ----------------------------------------------------------------------------
// Board : ESP32-DevKitC V2 / HW-394 (env: test_uart_esp -> nodemcu-32s)
// UART  : HardwareSerial UART2 (UART0 ist fuer USB belegt!)
//   RX2 = GPIO16 (D16)  <- XIAO D6 (TX)
//   TX2 = GPIO17 (D17)  -> XIAO D7 (RX)
//   GND <-> XIAO GND
//
// Test: jede Sekunde "Hello World" -> UART2 senden und pruefen, ob von der
//       Gegenseite etwas ankommt. Ausgabe EINE Statuszeile pro Sekunde:
//         >>> UART OK  <<<   wenn kuerzlich empfangen
//         !!! UART FAIL !!!  wenn nichts ankommt
// ============================================================================
#include <Arduino.h>

constexpr int      UART2_RX_PIN   = 16;     // RX2 <- XIAO TX (D6)
constexpr int      UART2_TX_PIN   = 17;     // TX2 -> XIAO RX (D7)
constexpr uint32_t UART_BAUD      = 115200;
constexpr uint32_t SEND_PERIOD_MS = 1000;
constexpr uint32_t RX_TIMEOUT_MS  = 2500;   // so lange gilt der Link als "OK"

void setup() {
    Serial.begin(UART_BAUD);                                  // USB-Konsole
    delay(300);
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);

    Serial.println();
    Serial.println("=== ESP bereit (UART2-Test) ===");
    Serial.printf("UART2: RX=GPIO%d  TX=GPIO%d  @ %u 8N1\n",
                  UART2_RX_PIN, UART2_TX_PIN, UART_BAUD);
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
    while (Serial2.available()) {
        char c = (char)Serial2.read();
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
        Serial2.printf("Hello World from ESP #%u\n", txSeq);

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
