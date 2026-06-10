#include "DataSender.h"

DataSender::DataSender(MeasurementData& data)
: data_(&data) {
}

void DataSender::sendData() {
  if (data_ == nullptr) {
    return;
  }

  data_->frame++;

  // Kurz und kompakt halten, sonst wird Serial bei 200 Hz schnell zum Bottleneck.
  Serial.printf("%lu;%.3f;%.3f\n",
                static_cast<unsigned long>(data_->frame),
                data_->sensor1,
                data_->sensor2);
}