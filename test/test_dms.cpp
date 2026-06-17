#include <Arduino.h>
#include "dms.h"

void setup() {
    Serial.begin(115200);
    dms_init();
}

void loop() {
    dms_update();
}
