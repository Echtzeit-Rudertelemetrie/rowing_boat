Entities:

Controller: AZ-Delivery ESP32 Dev Kit C V4 (Foto anbei)
    Chip: ESP32-WROOM-32 (4MB Flash)
    USB-UART: Silicon Labs CP2102
    38-Pin DevKitC Layout, 3.3V Logik
    Board-Bezeichnung auf dem Shield: "AZ-Delivery"

ADC-Frontend: Analog Devices EVAL-AD7124-8-PMDZ
    IC: AD7124-8, 24-Bit Sigma-Delta ADC mit integriertem PGA
    Betrieb: 3.3V Single Supply (IOVDD = DVDD = AVDD, über Lötbrücke P5 intern verbunden)
    Referenz: interne 2.5V Referenz aktiv (P4 und P7 auf dem Board sind geschlossen)
    SPI Mode 3 (CPOL=1, CPHA=1), max 5 MHz


Wiring:

Controller - ADC Frontend

G5 - P1 1
G23 - P1 2
G19 - P1 3
G18 - P1 4
GND - P1 5
3V3 - P1 6

ADC Frontend - Sensor

AIN0 -> DMS -> 350 Ohm resistor -> 350 Ohm resistor -> 350 Ohm resistor -> AIN1

ADC Frontend - Sensor

ADC Frontend
    AIN0 -> DMS -> P3 15
                -> 350 Ohm resistor -> AIN1
                                    -> 350 Ohm resistor -> GND
                                                        -> 350 Ohm resistor -> back to DMS
