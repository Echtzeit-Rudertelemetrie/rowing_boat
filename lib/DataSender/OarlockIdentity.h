#pragma once

#include <Arduino.h>

#include <cstdint>

// Jede Dolle identifiziert sich ueber ihre eigene MAC statt ueber eine fest
// einkompilierte Nummer. Damit laeuft auf allen Einheiten derselbe Build, und
// zwei gleichzeitig eingeschaltete Sender koennen nicht mehr dieselbe ID
// belegen. Genau das hat schon einmal einen Systemtest ruiniert: wechselnde
// Sequenznummern und Nullwinkel in der App, siehe
// docs/SESSION_ZUSAMMENFASSUNG_WINKELMESSUNG.md.
//
// Die ID muss in 4 Bit passen (IDSEQ_ID_MASK in AppTypes.h). ID 0 ist fuer die
// Telemetrie des Hubs reserviert, Dollen belegen 1..15. Die App haengt an
// dieser ID ihre Sitz- und Seitenzuordnung auf (SeatSlot in boat_config.dart),
// die ID einer Einheit sollte sich also nicht mehr aendern.
struct OarlockUnit
{
    std::uint64_t mac;  // Basis-MAC in derselben Reihenfolge wie gedruckt
    std::uint8_t id;    // 1..15
    const char *name;
};

// Neue Einheit? Hier eintragen. Die MAC liefert:
//   ~/.platformio/penv/bin/python \
//     ~/.platformio/packages/tool-esptoolpy/esptool.py --port <port> read_mac
//
// Nicht eingetragene Boards senden bewusst nicht (siehe oarlockId), damit ein
// unbekanntes Geraet nicht still die ID eines anderen ueberschreibt.
constexpr OarlockUnit OARLOCK_UNITS[] = {
    {0x14c19fc5c850ULL, 1, "Prototyp"},
    {0x14c19fc6fa98ULL, 2, "Einheit 2"},
};

constexpr std::uint8_t OARLOCK_ID_UNKNOWN = 0;

// Liest die MAC einmalig aus den eFuses und schlaegt sie in OARLOCK_UNITS nach.
// Liefert OARLOCK_ID_UNKNOWN, wenn die MAC nicht in der Tabelle steht.
std::uint8_t oarlockId();

// Klartextname aus der Tabelle, sonst "unbekannt". Nur fuer Logausgaben.
const char *oarlockName();

// Die gelesene Basis-MAC, damit eine unbekannte Einheit ihre eigene MAC
// ausgeben kann und man sie direkt in die Tabelle uebernehmen kann.
std::uint64_t oarlockMac();
