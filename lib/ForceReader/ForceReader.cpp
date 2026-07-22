// ForceReader.cpp
#include "ForceReader.h"

ForceReader::ForceReader() {
    // Konstruktor-Code
}

// TODO/ENTSCHEIDUNG (Team): Der Kraftpfad ist ein Stub — Sensor::ReadForce()
// liefert einen Dummy. Echte Implementierungen liegen ungenutzt in:
//  - lib/DMS (NAU7802, I2C): wuerde auf dem XIAO ESP32S3 laufen und koennte
//    sich den I2C-Bus (D4/D5) mit dem ICM-20948 teilen (Adresse 0x2A vs. 0x69).
//    Achtung: dms_init() blockiert bei fehlender Hardware mit while(1) ->
//    vor der Integration auf das begin()-Muster wie AngleReader umbauen.
//  - lib/AD7124 (Software-SPI auf GPIO 5/18/19/23): diese Pins existieren auf
//    dem XIAO ESP32S3 NICHT (verfuegbar: GPIO 1-9, 43, 44) -> neues
//    Pin-Mapping noetig, oder der AD7124 laeuft auf einem anderen Board.
// Solange soll Kraft auf diesem Board laufen? -> Wenn ja: welcher ADC?
// Bis zu der Entscheidung bleibt der Dummy in Sensor::ReadForce() aktiv.
float ForceReader::sampleForce() {
    // TODO: Janis echten Code hier
    return 0.0f;
}