#include <Arduino.h>
#include "gyro.h"
#include "dms.h"

void setup() {
    Serial.begin(115200);

    setupGyro();
    calibrateGyro();
    dms_init();
    delay(100);
}

void loop() {
    // TODO: replace with actual gyro update call once gyro.h exposes one
    dms_update();
}