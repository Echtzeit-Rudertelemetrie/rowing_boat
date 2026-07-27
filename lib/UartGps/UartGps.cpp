#include "UartGps.h"

void UartGps::begin() {
  Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
}

void UartGps::update() {
  while (Serial2.available() > 0) {
    gps_.encode(static_cast<char>(Serial2.read()));
  }
}

GpsData UartGps::data() {
  GpsData result{};
  result.valid = gps_.location.isValid();
  if (result.valid) {
    result.lat_e6 = static_cast<int32_t>(gps_.location.lat() * 1000000.0);
    result.lon_e6 = static_cast<int32_t>(gps_.location.lng() * 1000000.0);
  }
  if (gps_.speed.isValid()) {
    result.speed_cms = static_cast<uint16_t>(
      constrain(static_cast<long>(gps_.speed.mps() * 100.0 + 0.5), 0L, 65535L));
  }
  if (gps_.course.isValid()) {
    result.course_cdeg = static_cast<uint16_t>(
      constrain(static_cast<long>(gps_.course.deg() * 100.0 + 0.5), 0L, 35999L));
  }
  if (gps_.satellites.isValid()) {
    result.satellites = static_cast<uint8_t>(
      min(gps_.satellites.value(), static_cast<uint32_t>(255)));
  }
  return result;
}

uint32_t UartGps::charsProcessed() const {
  return gps_.charsProcessed();
}

uint32_t UartGps::passedChecksums() const {
  return gps_.passedChecksum();
}

uint32_t UartGps::failedChecksums() const {
  return gps_.failedChecksum();
}

uint32_t UartGps::locationAgeMs() const {
  return gps_.location.isValid() ? gps_.location.age() : UINT32_MAX;
}
