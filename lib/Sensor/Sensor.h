#pragma once

#include <Arduino.h>
#include "AngleReader.h"
#include "ForceReader.h"

class Sensor {
public:
  void begin();
  float ReadForce();
  float ReadAngle();
  const AngleDiagnostics& angleDiagnostics() const;
  void zeroAngle();

private:
    AngleReader angleReader;
    ForceReader forceReader;
};
