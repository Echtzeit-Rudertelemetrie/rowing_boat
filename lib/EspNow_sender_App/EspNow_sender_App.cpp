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

  // 200 Hz Abtastung (2026-07-29, vorher 100 Hz). 1 Tick = 1 EKF-Schritt, siehe
  // AngleReader::sampleAndCalculateAngle. Gyro und Accel liefern bei einem ODR
  // von 225 Hz tatsaechlich neue Werte, und der Schlag erreicht 276 deg/s.
  //
  // Das Magnetometer zieht NICHT mit: der AK09916 ist in diesem Aufbau mit
  // real 7-10 Hz frisch. Das ist unkritisch, weil das Staleness-Gate ihn
  // ohnehin nur bewertet, wenn ein frischer Wert vorliegt.
  //
  // Gesendet wird weiterhin mit 100 Hz -- handleEvent dezimiert SendData um
  // zwei. Das 8-Werte-Paket deckt damit unveraendert 80 ms ab und geht etwa
  // 12,5-mal pro Sekunde raus.
  //
  // Im Auge behalten: Der EKF-Schritt enthaelt eine 6x6-Inversion in float und
  // laeuft jetzt doppelt so oft. ANGLEREADER_PROFILING gibt queue_max aus --
  // laeuft die Event-Queue voll, ist die CPU das Limit und nicht der Sensor.
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
      const uint32_t depth = eventQueue_.messagesWaiting();
      if (depth > maxQueueDepth_) maxQueueDepth_ = depth;
#if ANGLEREADER_PROFILING
      const uint32_t nowMs = millis();
      if (static_cast<uint32_t>(nowMs - lastTimingReportMs_) >= 5000U) {
        lastTimingReportMs_ = nowMs;
        const AngleDiagnostics& d = sensor_.angleDiagnostics();
        Serial.printf(
          "ANGLE_TIMING,dt_min=%.6f,dt_mean=%.6f,dt_max=%.6f,"
          "invalid_dt=%lu,queue_now=%lu,queue_max=%lu,dropped=%lu,"
          "stationary=%d,accel_valid=%d,mag_valid=%d,clip_count=%lu\n",
          d.dtMin, d.dtMean, d.dtMax,
          static_cast<unsigned long>(d.invalidDtCount),
          static_cast<unsigned long>(depth),
          static_cast<unsigned long>(maxQueueDepth_),
          static_cast<unsigned long>(eventQueue_.droppedEvents()),
          d.stationary ? 1 : 0, d.accelValid ? 1 : 0, d.magValid ? 1 : 0,
          static_cast<unsigned long>(d.gyroClipCount));
      }
#endif
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
      break;

    case EventType::SendData: {
      // Der EKF laeuft mit 200 Hz, ins Paket darf aber nur jedes zweite Sample:
      // die App rechnet mit fest verdrahteten 10 ms je Sample
      // (BluetoothPacket.sampleIntervalMs in bluetooth_packet_decode_util.dart).
      // Ohne diese Dezimierung liefe ihre Zeitachse doppelt so schnell, und die
      // Paketrate wuerde sich ebenfalls verdoppeln.
      sendDivider_ ^= 1;
      if (sendDivider_ != 0) {
        break;
      }
      sender_.sendData();
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
