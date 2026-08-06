#pragma once

#include <Arduino.h>

#include <cstdint>

// Jede Einheit identifiziert sich ueber ihre eigene Board-MAC statt ueber fest
// einkompilierte Konstanten. Damit laeuft auf allen Einheiten derselbe Build,
// und niemand muss vor dem Flashen Werte tauschen.
//
// Zwei Dinge haengen an der MAC:
//   1. die Dollen-ID fuer ESP-NOW (frueher ESP_ID, dann OarlockIdentity),
//   2. die Magnetometer-Kalibrierung (frueher AngleReaderConfig::MAG_A/MAG_B
//      fuer die Dolle und BoatImuConfig::MAG_CALIBRATION fuer den Hub).
//
// Beides ist einbauspezifisch. Eine falsche ID belegt die Nummer einer anderen
// Dolle, eine falsche Kalibrierung setzt dem EKF einen falschen absoluten
// Anker. Gemessen auf 14:c1:9f:c6:fa:98 mit der Kalibrierung von c5:c8:50: der
// Magnetometervektor drehte sich um 5,5 Grad, waehrend sich der Koerper um
// 88,9 Grad drehte, und der Rollwinkel zerfiel mit rund 35 s Zeitkonstante in
// die Boot-Lage zurueck. Deshalb erbt eine unbekannte MAC hier grundsaetzlich
// nichts: sie bekommt ID 0 und ein abgeschaltetes Magnetometer.
//
// Die Tabelle selbst steht in UnitIdentity.cpp, nicht hier. Sie ist reine
// Messdatenpflege und soll keinen Rebuild aller Uebersetzungseinheiten
// ausloesen, die AngleReader.h einbinden.

// Hard-/Soft-Iron-Korrektur: mag_kalibriert = matrix * (mag_roh - offset).
// Steht hier und nicht mehr in AngleReader.h, weil die Werte pro Einheit aus
// der Tabelle unten kommen und AngleReader damit nur noch Verbraucher ist.
// AngleReader.h inkludiert diesen Header weiter, die bisherige API bleibt also
// unveraendert gueltig.
//
// verified == false bedeutet: Magnetometer aus. Der EKF laeuft dann nur auf
// Gyro und Beschleunigung, der Gierwinkel driftet, Roll und Nick bleiben
// gestuetzt. Das ist der sichere Zustand, nicht der Ausnahmefall.
struct MagCalibration
{
    bool verified;
    float matrix[3][3];
    float offset[3];
};

// Neutral: Einheitsmatrix, Nulloffset, Magnetometer abgeschaltet. Fallback fuer
// jede MAC, die nicht in der Tabelle steht, und explizit eintragbar fuer eine
// Einheit mit defektem Sensor.
constexpr MagCalibration MAG_CALIBRATION_DISABLED = {
    false,
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    },
    {0.0f, 0.0f, 0.0f},
};

struct UnitInfo
{
    std::uint64_t mac;      // Basis-MAC in derselben Reihenfolge wie gedruckt
    std::uint8_t oarlockId; // 1..15, oder OARLOCK_ID_UNKNOWN wenn keine Dolle
    const char *name;
    MagCalibration mag;
};

// Die ID muss in 4 Bit passen (IDSEQ_ID_MASK in AppTypes.h). ID 0 ist fuer die
// Telemetrie des Hubs reserviert, Dollen belegen 1..15. Die App haengt an
// dieser ID ihre Sitz- und Seitenzuordnung auf (SeatSlot in boat_config.dart),
// die ID einer Einheit sollte sich also nicht mehr aendern.
//
// Derselbe Wert steht fuer "unbekannte MAC" und fuer "diese Einheit ist keine
// Dolle". Beide duerfen nicht als Dolle senden, der Sender braucht die beiden
// Faelle also nicht zu unterscheiden.
constexpr std::uint8_t OARLOCK_ID_UNKNOWN = 0;

// Die gelesene Basis-MAC, damit eine unbekannte Einheit ihre eigene MAC
// ausgeben kann und man sie direkt in die Tabelle uebernehmen kann.
std::uint64_t unitMac();

// Dollen-ID aus der Tabelle. OARLOCK_ID_UNKNOWN, wenn die MAC nicht eingetragen
// ist oder die Einheit keine Dolle ist (Hub).
std::uint8_t unitOarlockId();

// Klartextname aus der Tabelle, sonst "unbekannt". Nur fuer Logausgaben.
const char *unitName();

// Magnetometer-Kalibrierung dieser Einheit. Fuer eine unbekannte MAC immer
// MAG_CALIBRATION_DISABLED, niemals die Werte einer anderen Einheit.
const MagCalibration &unitMagCalibration();
