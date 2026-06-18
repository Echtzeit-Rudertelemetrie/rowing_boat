#pragma once

#include <Arduino.h>
#include "AppTypes.h"
#include "EventQueue.h"
#include "Sensor.h"
#include "DataSender.h"
#include "TimerManager.h"

class EspNow_sender_App {
public:
  EspNow_sender_App();
  void begin();
  void run();

private:
  void handleEvent(EventType event);

  MeasurementData latest_;
  EventQueue eventQueue_;
  Sensor sensor_;
  DataSender sender_;
  TimerManager timer_;
};