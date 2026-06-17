#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("Idle - waiting for commands");
}

void loop() {
    delay(1000);
}