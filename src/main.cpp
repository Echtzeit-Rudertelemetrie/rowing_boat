#include <Arduino.h>
#include "MainApp.h"
#include "Eval_loop.h"

MainApp app;

void setup() {
  app.begin();
}

void loop() {
    uint32_t t0 = micros();

    eval_loop();

    uint32_t t1 = micros();

    Serial.printf("eval loop dauerte %u us\n", t1 - t0);
}