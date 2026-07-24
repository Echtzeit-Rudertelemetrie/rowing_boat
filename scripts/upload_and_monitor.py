#!/usr/bin/env python3
"""Upload an environment and monitor the exact same serial device.

PlatformIO resolves the monitor port before running pre-build scripts. A
combined ``pio run -t upload -t monitor`` can therefore upload to the selected
by-id device but monitor an unrelated auto-detected device. This wrapper passes
both ports explicitly and works on Linux and macOS.
"""

from glob import glob
from pathlib import Path
import platform
import shutil
import subprocess
import sys


PORTS = {
    "Linux": {
        "espNow_receiver_esp": ["/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"],
        "prod_BLE_sender": [
            "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C6:D4:78-if00"
        ],
        "xiao_s3": [
            "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C5:C8:50-if00"
        ],
    },
    "Darwin": {
        "espNow_receiver_esp": [
            "/dev/cu.usbserial-110",
            "/dev/cu.usbserial-*",
            "/dev/cu.wchusbserial*",
        ],
        "prod_BLE_sender": ["/dev/cu.usbmodem101", "/dev/cu.usbmodem*"],
        "xiao_s3": ["/dev/cu.usbmodem*"],
    },
}


def base_environment(name: str) -> str:
    for base in ("espNow_receiver_esp", "prod_BLE_sender", "xiao_s3"):
        if name == base or name.startswith(f"{base}_"):
            return base
    return name


def first_existing(patterns: list[str]) -> str | None:
    for pattern in patterns:
        matches = sorted(glob(pattern)) if "*" in pattern else [pattern]
        for match in matches:
            if Path(match).exists():
                return match
    return None


def platformio_command() -> str:
    command = shutil.which("platformio") or shutil.which("pio")
    if command:
        return command

    bundled = Path.home() / ".platformio" / "penv" / "bin" / "platformio"
    if bundled.exists():
        return str(bundled)
    raise SystemExit("PlatformIO wurde weder im PATH noch unter ~/.platformio gefunden.")


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit(f"Aufruf: {Path(sys.argv[0]).name} PLATFORMIO_ENVIRONMENT")

    environment = sys.argv[1]
    system = platform.system()
    patterns = PORTS.get(system, {}).get(base_environment(environment), [])
    port = first_existing(patterns)
    if not port:
        raise SystemExit(
            f"Kein bekannter serieller Port fuer {environment} unter {system} gefunden."
        )

    pio = platformio_command()
    print(f"Upload und Monitor verwenden beide: {port}", flush=True)
    upload = subprocess.run(
        [pio, "run", "-e", environment, "-t", "upload", "--upload-port", port],
        check=False,
    )
    if upload.returncode:
        return upload.returncode

    return subprocess.run(
        [pio, "device", "monitor", "-e", environment, "--port", port],
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
