import serial

port = "/dev/tty.usbserial-*"  # anpassen
ser = serial.Serial(port, 115200, timeout=5)

# Auf Start-Marker warten
while ser.read(2) != b'\xAA\xBB':
    pass

size = int.from_bytes(ser.read(4), 'little')
data = ser.read(size)

with open("scripts/log.bin", "wb") as f:
    f.write(data)

print(f"Empfangen: {len(data)} Bytes")