#include <Arduino.h>

#define RGB_BRIGHTNESS 10

#ifdef RGB_BUILTIN
#undef RGB_BUILTIN
#endif
#define RGB_BUILTIN 21

void setup() {
}

void loop() {
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS);
  delay(1000);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  delay(1000);
}