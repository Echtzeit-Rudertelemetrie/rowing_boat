#pragma once

#include <Arduino.h>
#include "AppTypes.h"

class DataSender {
public:
  explicit DataSender(MeasurementData& data);

  void sendData();

private:
  MeasurementData* data_;
};