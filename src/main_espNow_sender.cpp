#include <Arduino.h>
#include "EspNow_sender_App.h"

EspNow_sender_App app;

void setup() {
  app.begin();
}

void loop() {
  app.run();
}