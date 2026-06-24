#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    delay(100);
    Serial.println("alive");


}

void loop() {
    Serial.println("tick");
    delay(1000);
}