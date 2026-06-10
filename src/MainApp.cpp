#include "MainApp.h"
#include "Eval_loop.h"

MainApp::MainApp()
: latest_{}
, eventQueue_{}
, sensor_{}
, sender_(latest_)
, timer_(eventQueue_) {
}

void MainApp::begin() {
  Serial.begin(115200);
  delay(200);

  sensor_.begin();

  // Die Queue sollte groß genug sein, damit kurze Lastspitzen nicht sofort Events verlieren.
  if (!eventQueue_.begin(64)) {
    Serial.println("EventQueue konnte nicht erstellt werden.");
    while (true) {
      delay(1000);
    }
  }

  if (!timer_.begin(200)) {
    Serial.println("Timer konnte nicht gestartet werden.");
    while (true) {
      delay(1000);
    }
  }

  eval_setup();

  Serial.println("System gestartet.");
}

void MainApp::run() {
  for (;;) {
    EventType event;

    // Blockiert, bis ein Event da ist.
    // Dadurch wird nicht aktiv gespammt oder in einer Busy-Wait-Schleife rotiert.
    if (eventQueue_.pop(event, portMAX_DELAY)) {
      handleEvent(event);
    }
  }
}

/*void MainApp::handleEvent(EventType event) {
  switch (event) {
    case EventType::ReadSensor1:
        unsigned long ms = millis();

        Serial.printf("Uptime: %02lu\n",
                    ms);
      latest_.sensor1 = sensor_.ReadSensor1();
      break;

    case EventType::ReadSensor2:
      latest_.sensor2 = sensor_.ReadSensor2();
      break;

    case EventType::SendData:
      sender_.sendData();
        unsigned long ims = millis();

        Serial.printf("Uptime: %02lu\n",
                    ims);
      break;
  }
}*/

void MainApp::handleEvent(EventType event) {
    switch (event) {
        case EventType::ReadSensor1: {
            unsigned long ms = millis();

            Serial.printf("Uptime: %lu\n", ms);

            latest_.sensor1 = sensor_.ReadSensor1();
            eval_loop();
            break;
        }

        case EventType::ReadSensor2: {
            latest_.sensor2 = sensor_.ReadSensor2();
            break;
        }

        case EventType::SendData: {
            sender_.sendData();

            unsigned long ms = millis();

            Serial.printf("Uptime: %lu\n", ms);

            break;
        }

        default:
            break;
    }
}