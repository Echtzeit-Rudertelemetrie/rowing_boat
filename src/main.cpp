#include <Arduino.h>
#include "MainApp.h"

MainApp app;

void setup() {
  app.begin();
}

void loop() {
  app.run();
}