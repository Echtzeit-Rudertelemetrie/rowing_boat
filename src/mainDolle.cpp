#include <Arduino.h>
#include "DollenApp.h"

DollenApp app;

void setup() {
  app.begin();
}

void loop() {
  app.run();
}