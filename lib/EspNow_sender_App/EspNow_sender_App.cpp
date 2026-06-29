#include "EspNow_sender_App.h"

EspNow_sender_App::EspNow_sender_App()
: latest_{}
, eventQueue_{}
, sensor_{}
, sender_(latest_)
, timer_(eventQueue_) {
}

void EspNow_sender_App::begin() {
  Serial.begin(115200);
  delay(200);

  Serial.printf("Begin (reset_reason=%d)\n", (int)esp_reset_reason());

  // Sensor-Hardware wird hier initialisiert (NICHT mehr im Konstruktor),
  // damit Serial bereits laeuft und fehlende Hardware den Boot nicht blockiert.
  sensor_.begin();

  sender_.espnow_init_sender();

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

  Serial.println("System gestartet.");
}

void EspNow_sender_App::run() {
  for (;;) {
    EventType event;

    // Blockiert, bis ein Event da ist.
    // Dadurch wird nicht aktiv gespammt oder in einer Busy-Wait-Schleife rotiert.
    if (eventQueue_.pop(event, portMAX_DELAY)) {
      handleEvent(event);
    }
  }
}

void EspNow_sender_App::handleEvent(EventType event) {
  switch (event) {
    case EventType::ReadSensor1: {
      //unsigned long ms = millis();

      //Serial.printf("Uptime: %02lu\n", ms);
      latest_.forceSensor = sensor_.ReadForce();
      break;
    }

    case EventType::ReadSensor2:
      latest_.degreeSensor = sensor_.ReadAngle();
      Serial.printf(">Winkel:%.2lf\n", latest_.degreeSensor);
      break;

    case EventType::SendData: {
      //sender_.sendData();

      //unsigned long ims = millis();

      //Serial.printf("Uptime: %02lu\n", ims);
      break;
    }
  }
}

/*void DollenApp::handleEvent(EventType event) {
    switch (event) {
        case EventType::ReadSensor1: {
            latest_.forceSensor = sensor_.ReadForce();
            break;
        }

        case EventType::ReadSensor2: {
            latest_.degreeSensor = sensor_.ReadAngle();
            break;
        }

        case EventType::SendData: {
            sender_.sendData();
            break;
        }

        default:
            break;
    } 
} */