"""Configure the NEO-M8T receiver over its own USB port.

The hub firmware only parses GGA and RMC, but the receiver ships with fourteen
NMEA sentences per epoch at 1 Hz and 9600 baud, which saturates the UART to the
XIAO at 82 percent and rules out higher rates. This script disables the unused
sentences, raises the navigation rate and reconfigures UART1, then stores the
result in the receiver's flash so it survives a power cycle.

The settings live in the module, not in the firmware. A replacement receiver
arrives at 9600 baud and will not match ``UartGps::BAUD`` until this script has
been run against it once.

Usage:
    python3 scripts/configure_gps.py                 # 10 Hz, 115200 baud
    python3 scripts/configure_gps.py --rate 5
    python3 scripts/configure_gps.py --port /dev/cu.usbmodem1301

Connect the receiver's USB port to the computer; the UART wiring to the XIAO is
not used here.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial fehlt: python3 -m pip install -r scripts/requirements-angle-tools.txt")


UBLOX_VID = 0x1546

# NMEA standard message IDs within UBX class 0xF0.
NMEA_MESSAGES = {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
    "ZDA": 0x08,
}

# TinyGPSPlus reads position and satellite count from GGA, speed and course
# from RMC. Everything else is dead weight on the wire.
DEFAULT_KEEP = ("GGA", "RMC")

# CFG-PRT mode for 8N1: charLen=8 (bits 6-7), parity=none (bit 11), 1 stop bit.
UART_MODE_8N1 = 0x000008C0
PROTO_UBX_NMEA = 0x0003
PROTO_NMEA = 0x0002


def ubx(msg_class, msg_id, payload=b""):
    """Wrap a payload in a UBX frame including the Fletcher checksum."""
    body = bytes([msg_class, msg_id]) + len(payload).to_bytes(2, "little") + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


def find_receiver():
    """Return the port of the first attached u-blox receiver, if any."""
    for port in list_ports.comports():
        if port.vid == UBLOX_VID:
            return port.device
    return None


def await_ack(link, msg_class, msg_id, timeout=2.0):
    """Scan the incoming stream for UBX-ACK-ACK/NAK matching a message."""
    deadline = time.time() + timeout
    buffer = b""
    while time.time() < deadline:
        buffer += link.read(256)
        index = 0
        while (index := buffer.find(b"\xb5\x62\x05", index)) != -1:
            if len(buffer) >= index + 10 and buffer[index + 3] in (0x00, 0x01):
                if buffer[index + 6] == msg_class and buffer[index + 7] == msg_id:
                    return "ACK" if buffer[index + 3] == 0x01 else "NAK"
            index += 1
    return "keine Antwort"


def apply(link, msg_class, msg_id, payload, label):
    link.write(ubx(msg_class, msg_id, payload))
    result = await_ack(link, msg_class, msg_id)
    print(f"  {label:<44} {result}")
    return result == "ACK"


def read_version(link):
    link.write(ubx(0x0A, 0x04))
    time.sleep(0.8)
    raw = link.read(4096)
    start = raw.find(b"\xb5\x62\x0a\x04")
    if start == -1:
        return None
    length = int.from_bytes(raw[start + 4:start + 6], "little")
    payload = raw[start + 6:start + 6 + length]
    fields = {
        "sw": payload[0:30].split(b"\x00")[0].decode(errors="replace"),
        "hw": payload[30:40].split(b"\x00")[0].decode(errors="replace"),
    }
    fields["ext"] = [
        payload[40 + offset:70 + offset].split(b"\x00")[0].decode(errors="replace")
        for offset in range(0, max(0, length - 40), 30)
    ]
    return fields


def measure(link, seconds, keep):
    """Read the live stream back to confirm what the receiver actually emits."""
    link.reset_input_buffer()
    start = time.time()
    buffer = b""
    while time.time() - start < seconds:
        buffer += link.read(1024)
    elapsed = time.time() - start
    text = buffer.decode(errors="replace")
    counts = {name: text.count(name) for name in NMEA_MESSAGES}
    leftover = {n: c for n, c in counts.items() if c and n not in keep}
    rates = ", ".join(f"{n} {counts[n] / elapsed:.1f}/s" for n in keep)
    print(f"  aktiv:        {rates}")
    print(f"  Datenrate:    {len(buffer) / elapsed:.0f} Byte/s")
    print(f"  abgeschaltet: {'noch aktiv: ' + str(leftover) if leftover else 'bestaetigt'}")
    sample = next((line for line in text.splitlines() if "RMC" in line), "")
    if sample:
        print(f"  Beispiel:     {sample[:88]}")
    return counts, len(buffer) / elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="Serieller Port des Empfaengers (sonst automatisch)")
    parser.add_argument("--rate", type=float, default=10.0, help="Navigationsrate in Hz (Standard 10)")
    parser.add_argument("--baud", type=int, default=115200, help="UART1-Baudrate (Standard 115200)")
    parser.add_argument("--keep", default=",".join(DEFAULT_KEEP),
                        help="Aktive NMEA-Saetze, kommagetrennt (Standard GGA,RMC)")
    args = parser.parse_args()

    port = args.port or find_receiver()
    if not port:
        sys.exit("Kein u-blox-Empfaenger gefunden. USB-Kabel pruefen oder --port angeben.")

    keep = tuple(name.strip().upper() for name in args.keep.split(",") if name.strip())
    unknown = [name for name in keep if name not in NMEA_MESSAGES]
    if unknown:
        sys.exit(f"Unbekannte NMEA-Saetze: {', '.join(unknown)}")

    interval_ms = round(1000.0 / args.rate)
    if not 25 <= interval_ms <= 65535:
        sys.exit(f"Rate {args.rate} Hz liegt ausserhalb des unterstuetzten Bereichs.")

    link = serial.Serial(port, 115200, timeout=0.4, dsrdtr=False)
    link.dtr = True
    link.rts = True
    time.sleep(0.4)
    link.reset_input_buffer()

    print(f"Empfaenger an {port}")
    version = read_version(link)
    if version:
        model = next((e for e in version["ext"] if e.startswith("MOD=")), "")
        print(f"  {model or 'Modell unbekannt'}  sw={version['sw']} hw={version['hw']}")

    ok = True

    print("\nNMEA-Saetze")
    for name, msg_id in NMEA_MESSAGES.items():
        enabled = 1 if name in keep else 0
        # Rate per port: 0=DDC/I2C 1=UART1 2=UART2 3=USB 4=SPI 5=reserved.
        rates = bytes([enabled, enabled, 0, enabled, 0, 0])
        ok &= apply(link, 0x06, 0x01, bytes([0xF0, msg_id]) + rates,
                    f"{name} {'aktiv' if enabled else 'aus'}")

    print("\nNavigationsrate")
    ok &= apply(link, 0x06, 0x08,
                interval_ms.to_bytes(2, "little")   # measRate
                + (1).to_bytes(2, "little")         # navRate, ein Fix je Messung
                + (1).to_bytes(2, "little"),        # timeRef = GPS
                f"{interval_ms} ms ({1000 / interval_ms:.1f} Hz)")

    print("\nUART1 zum XIAO")
    ok &= apply(link, 0x06, 0x00,
                bytes([1, 0])                            # portID 1, reserviert
                + (0).to_bytes(2, "little")              # txReady aus
                + UART_MODE_8N1.to_bytes(4, "little")
                + args.baud.to_bytes(4, "little")
                + PROTO_UBX_NMEA.to_bytes(2, "little")   # Eingang
                + PROTO_NMEA.to_bytes(2, "little")       # Ausgang: nur NMEA
                + (0).to_bytes(2, "little")
                + (0).to_bytes(2, "little"),
                f"{args.baud} Baud 8N1, nur NMEA")

    print("\nDauerhaft speichern")
    ok &= apply(link, 0x06, 0x09,
                (0).to_bytes(4, "little")            # clearMask
                + (0xFFFF).to_bytes(4, "little")     # saveMask: alle Bereiche
                + (0).to_bytes(4, "little")          # loadMask
                + bytes([0x17]),                     # BBR, Flash, EEPROM, SPI-Flash
                "CFG-CFG save")

    print("\nGegenprobe am USB-Port")
    measure(link, 6.0, keep)
    link.close()

    if not ok:
        sys.exit("\nMindestens ein Befehl wurde nicht bestaetigt; Konfiguration unvollstaendig.")
    print(f"\nFertig. UartGps::BAUD muss auf {args.baud} stehen.")


if __name__ == "__main__":
    main()
