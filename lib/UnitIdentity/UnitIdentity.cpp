#include "UnitIdentity.h"

#include <esp_system.h>

// ============================================================================
// Einheitentabelle
// ============================================================================
//
// Neue Einheit? Hier eintragen. Die MAC liefert:
//   ~/.platformio/penv/bin/python \
//     ~/.platformio/packages/tool-esptoolpy/esptool.py --port <port> read_mac
//
// Neue Kalibrierwerte? Nur den mag-Block der betroffenen Einheit ersetzen. Die
// Werte kommen aus scripts/calibrate_magnetometer.py, das genau diesen Block
// druckfertig ausgibt. Keine andere Datei anfassen.
//
// Nicht eingetragene Boards senden bewusst nicht (siehe unitOarlockId) und
// laufen ohne Magnetometer, damit ein unbekanntes Geraet weder still die ID
// eines anderen ueberschreibt noch dessen Hard-/Soft-Iron erbt.
//
// ----------------------------------------------------------------------------
// Warum die Kalibrierung nicht geteilt werden darf
// ----------------------------------------------------------------------------
// Hard- und Soft-Iron sind Eigenschaften des Einbaus, nicht des Sensortyps.
// Gemessen 2026-07-29 auf Board 14:c1:9f:c6:fa:98 mit den Werten von c5:c8:50:
// der Magnetometervektor drehte sich um 5,5 Grad, waehrend sich der Koerper um
// 88,9 Grad drehte. Der EKF las den nicht weggerechneten koerperfesten Rest als
// absolute Referenz und zog den Rollwinkel mit rund 35 s Zeitkonstante in die
// Boot-Lage zurueck.
//
// ----------------------------------------------------------------------------
// Aufnahmen kurz halten
// ----------------------------------------------------------------------------
// Vier Versuche auf c5:c8:50: 120-s-Laeufe ergaben 8,76 %, 12,05 % und 10,48 %
// Residuum, ein 60-s-Lauf 5,04 %. Der Kugelmittelpunkt driftet waehrend der
// Aufnahme (32 uT mit lose haengendem USB-Kabel, 16 uT mit abgeklebtem Kabel,
// 14 uT ueber 60 s), das Hard Iron dieses Aufbaus ist also nicht voellig
// stabil, und ein laengeres Taumeln mittelt nur ueber mehr von dieser Drift.
// Kabel sichern hilft, kuerzer aufnehmen hilft mehr. Die Abdeckung war nie das
// Problem — der schlechteste Lauf hatte die beste Abdeckung.

namespace
{

    constexpr UnitInfo UNITS[] = {

        // --------------------------------------------------------------------
        // Dolle "Prototyp", ID 1
        // --------------------------------------------------------------------
        // scripts/calibrate_magnetometer.py aus logs/oarlock_mag_09.csv
        // (2026-07-30), Residuum 6,36 %, Feldbetrag 50,8 uT gegen die rund
        // 48 uT des Erdfelds in Deutschland, Abdeckung 94 % der
        // Richtungszellen.
        //
        // Nicht das niedrigste Residuum gewinnt, sondern das ehrlichste.
        // Kreuzvalidierung auf dieser vollflaechigen Aufnahme (2026-07-30):
        //
        //   Kalibrierung           Betrag     Streuung
        //   aus 60 s / 60 % Fleck  87,9 uT      31,3 %   <- war aktiv
        //   aus 16 s / 40 % Fleck  84,4 uT      29,0 %
        //   diese hier             50,7 uT       6,4 %
        //
        // Die beiden Fleck-Fits meldeten selbst 5,04 % und 3,73 % und sahen
        // damit besser aus als diese Werte. Ueber die volle Kugel liefern sie
        // aber rund 86 uT statt der physikalisch moeglichen 48 — ein Fit auf
        // einem Ausschnitt beschreibt eben nur den Ausschnitt. Ein niedriges
        // Residuum ist deshalb nur zusammen mit der Abdeckung aussagekraeftig,
        // und der Feldbetrag ist die eigentliche Probe: er MUSS beim lokalen
        // Erdfeld landen.
        {0x14c19fc5c850ULL,
         1,
         "Prototyp",
         {
             true,
             {
                 {0.902244222f, 0.0874501208f, 0.0051898252f},
                 {0.0874501208f, 1.04080628f, 0.00991110014f},
                 {0.0051898252f, 0.00991110014f, 1.02591005f},
             },
             {-27.8425147f, -114.111435f, 16.1564392f},
         }},

        // --------------------------------------------------------------------
        // Dolle "Einheit 2", ID 2 — Magnetometer defekt, bewusst abgeschaltet
        // --------------------------------------------------------------------
        // Drei Aufnahmen, eine davon mit 97 % Kugelabdeckung, alle unbrauchbar:
        // Radius 31,8 uT, Streuung 37,1 %, Residuum 35,9 %, Rohbetrag rund
        // 110 uT gegenueber rund 85 uT auf c5:c8:50. Gleicher Raum, gleiches
        // Vorgehen, gleiche Firmware — also ein Hardwarefehler dieser Einheit
        // und kein Aufnahmefehler.
        //
        // Der eigene 97-%-Fit wurde probiert. Er beseitigt zwar das
        // Zurueckziehen, steuert aber nur rund 0,7 Grad/min Korrektur bei:
        // harmlos, aber nutzlos. Deshalb nicht eingetragen, sondern hier
        // dokumentiert, falls die Einheit spaeter neu vermessen wird:
        //   matrix {2.77623467f, 0.697274236f, -0.86762364f},
        //          {0.697274236f, 0.482282293f, -0.191071711f},
        //          {-0.86762364f, -0.191071711f, 1.4394058f}
        //   offset {77.3478971f, 1.94298809f, -71.6231525f}
        {0x14c19fc6fa98ULL,
         2,
         "Einheit 2",
         MAG_CALIBRATION_DISABLED},

        // --------------------------------------------------------------------
        // Boot-Hub — keine Dolle, sendet nicht per ESP-NOW
        // --------------------------------------------------------------------
        // Neu kalibriert 2026-07-29 aus logs/boat_mag_02.csv: Residuum 4,59 %,
        // Spannenverhaeltnis 0,659, Feldbetrag 40,2 uT. Die Soft-Iron-Matrix
        // reproduziert den Lauf vom 2026-07-27 fast exakt (Diagonale damals
        // 1,284/1,463/0,927, jetzt 1,203/1,471/0,910) — zwei unabhaengige
        // Messungen, die sich ueber eine Eigenschaft des Aufbaus einig sind,
        // und das ist der beste verfuegbare Beleg dafuer, dass beide Laeufe
        // sauber waren. Das Hard Iron ist dagegen gewandert, von |b| 3,9 uT auf
        // 11,7 uT, und genau das hat die Wiederholung gelohnt.
        //
        // Vorherige Werte (2026-07-27, calibration/boat_mag_01):
        //   matrix {1.28429129f, -0.0190608838f, 0.103510479f},
        //          {-0.0190608838f, 1.4633201f, 0.0141170056f},
        //          {0.103510479f, 0.0141170056f, 0.927376894f}
        //   offset {3.29506649f, -0.102700858f, 2.1655548f}
        {0x14c19fc6d478ULL,
         OARLOCK_ID_UNKNOWN,
         "Boot-Hub",
         {
             true,
             {
                 {1.20315044f, -0.0354236374f, 0.109790254f},
                 {-0.0354236374f, 1.47096324f, 0.074241181f},
                 {0.109790254f, 0.074241181f, 0.909921722f},
             },
             {5.15215573f, 4.03903918f, -9.73854238f},
         }},
    };

    std::uint64_t readBaseMac()
    {
        std::uint8_t mac[6]{};
        esp_efuse_mac_get_default(mac);
        // Byteweise zusammensetzen statt die 6 Bytes auf einen uint64 zu
        // casten: ESP.getEfuseMac() tut genau das und liefert die MAC deshalb
        // rueckwaerts. Hier soll der Wert so aussehen wie die gedruckte MAC,
        // damit die Tabelle oben lesbar bleibt.
        std::uint64_t value = 0;
        for (std::uint8_t byte : mac)
        {
            value = (value << 8) | byte;
        }
        return value;
    }

    const UnitInfo &identity()
    {
        // Die MAC liegt in eFuses und aendert sich zur Laufzeit nicht, also
        // einmal lesen und merken. Function-local static laeuft beim ersten
        // Aufruf, nicht in der ungeordneten statischen Initialisierung.
        static const UnitInfo resolved = []
        {
            const std::uint64_t mac = readBaseMac();
            for (const UnitInfo &unit : UNITS)
            {
                if (unit.mac == mac)
                {
                    return unit;
                }
            }
            // Kein Erben: unbekannte MAC bekommt ID 0 und Magnetometer aus.
            return UnitInfo{mac, OARLOCK_ID_UNKNOWN, "unbekannt",
                            MAG_CALIBRATION_DISABLED};
        }();
        return resolved;
    }

} // namespace

std::uint64_t unitMac() { return identity().mac; }

std::uint8_t unitOarlockId() { return identity().oarlockId; }

const char *unitName() { return identity().name; }

const MagCalibration &unitMagCalibration() { return identity().mag; }
