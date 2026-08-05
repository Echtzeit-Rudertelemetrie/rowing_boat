#include "OarlockIdentity.h"

#include <esp_system.h>

namespace
{

    struct Identity
    {
        std::uint64_t mac;
        std::uint8_t id;
        const char *name;
    };

    std::uint64_t readBaseMac()
    {
        std::uint8_t mac[6]{};
        esp_efuse_mac_get_default(mac);
        // Byteweise zusammensetzen statt die 6 Bytes auf einen uint64 zu
        // casten: ESP.getEfuseMac() tut genau das und liefert die MAC deshalb
        // rueckwaerts. Hier soll der Wert so aussehen wie die gedruckte MAC,
        // damit die Tabelle in OarlockIdentity.h lesbar bleibt.
        std::uint64_t value = 0;
        for (std::uint8_t byte : mac)
        {
            value = (value << 8) | byte;
        }
        return value;
    }

    const Identity &identity()
    {
        // Die MAC liegt in eFuses und aendert sich zur Laufzeit nicht, also
        // einmal lesen und merken. Function-local static laeuft beim ersten
        // Aufruf, nicht in der ungeordneten statischen Initialisierung.
        static const Identity resolved = []
        {
            const std::uint64_t mac = readBaseMac();
            for (const OarlockUnit &unit : OARLOCK_UNITS)
            {
                if (unit.mac == mac)
                {
                    return Identity{mac, unit.id, unit.name};
                }
            }
            return Identity{mac, OARLOCK_ID_UNKNOWN, "unbekannt"};
        }();
        return resolved;
    }

} // namespace

std::uint8_t oarlockId() { return identity().id; }

const char *oarlockName() { return identity().name; }

std::uint64_t oarlockMac() { return identity().mac; }
