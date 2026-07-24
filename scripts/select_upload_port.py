"""Select known production-board serial ports without tying PlatformIO to one OS.

If no known device is present, PlatformIO keeps its normal auto-detection. This
preserves the previous macOS workflow while selecting unambiguous /dev/serial/
by-id paths on the Linux development machine.
"""

Import("env")

from glob import glob
from pathlib import Path
import platform


ENVIRONMENT = env.subst("$PIOENV")

LINUX_PORTS = {
    "espNow_receiver_esp": [
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    ],
    "prod_BLE_sender": [
        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C6:D4:78-if00",
    ],
    "xiao_s3": [
        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C5:C8:50-if00",
    ],
}

MACOS_PORTS = {
    "espNow_receiver_esp": [
        "/dev/cu.usbserial-110",
        "/dev/cu.usbserial-*",
        "/dev/cu.wchusbserial*",
    ],
    "prod_BLE_sender": [
        "/dev/cu.usbmodem101",
        "/dev/cu.usbmodem*",
    ],
    "xiao_s3": [
        "/dev/cu.usbmodem*",
    ],
}


def base_environment(name):
    for base in ("espNow_receiver_esp", "prod_BLE_sender", "xiao_s3"):
        if name == base or name.startswith(f"{base}_"):
            return base
    return name


def first_existing(patterns):
    for pattern in patterns:
        if "*" in pattern:
            matches = sorted(glob(pattern))
            if matches:
                return matches[0]
        elif Path(pattern).exists():
            return pattern
    return None


system = platform.system()
base = base_environment(ENVIRONMENT)
patterns = LINUX_PORTS.get(base, []) if system == "Linux" else MACOS_PORTS.get(base, [])
port = first_existing(patterns)

if port:
    env.Replace(UPLOAD_PORT=port, MONITOR_PORT=port)
    print(f"Using {system} serial port for {ENVIRONMENT}: {port}")
else:
    print(f"No configured {system} serial port found for {ENVIRONMENT}; using PlatformIO auto-detection")
