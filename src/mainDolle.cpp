#include <Arduino.h>
#include "EspNow_sender_App.h"

// Entry-Point der Dollen-Einheit (envs xiao_s3 / xiao_s3_matlab).
// Die App-Klasse hiess in sensor_miniesp_code DollenApp und wurde in main
// nach EspNow_sender_App umbenannt — die begin()/run()-API ist unveraendert.
EspNow_sender_App app;

void setup() {
  app.begin();
}

void loop() {
  app.run();
}
