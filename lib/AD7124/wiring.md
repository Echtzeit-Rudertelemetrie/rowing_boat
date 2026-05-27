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

AIN0 -> DMS -> 350 Ohm resistor -> 350 Ohm resistor -> 350 Ohm resistor -> AIN1 -> GND

ADC Frontend - Sensor

ADC Frontend
    AIN0 -> DMS -> P3 15
                -> 350 Ohm resistor -> AIN1
                                    -> 350 Ohm resistor -> GND
                                                        -> 350 Ohm resistor -> back to DMS

Gain um 2⁷ reduzieren:
6704,5094391,15643.46,-282.61
6804,5096121,15653.29,-272.78
6904,5085791,15650.94,-275.13
7004,5092911,15650.28,-275.78
7104,5080327,15638.86,-287.20
7204,5073185,15624.34,-301.72
7319,5072663,15605.84,-320.22
7419,5072043,15592.65,-333.41
7519,5077019,15592.20,-333.87
7619,5075495,15593.92,-332.14
7719,5075391,15598.82,-327.24
7819,5078085,15600.01,-326.06

AIN0 und AIN1 gegen GND
AIN0 - GND - 1.650
AIN1 - GND - 1.640

AIN0 und AIN1 kurzschließen:
kurzgeschlossen:
18355,2487,7.65,2.03
18455,2483,7.64,2.03
18570,2541,7.65,2.03
18670,2329,7.73,2.11
18770,2409,7.68,2.06
18870,2277,7.56,1.95
18970,2531,7.40,1.78
19070,2495,7.51,1.89

kurzschluss entfernt:
19170,4467213,963.69,958.07
19270,7618941,8249.77,8244.15
19370,7622965,15565.35,15559.73
19470,7637691,22796.31,22790.70
19570,7646687,23451.99,23446.37
19685,7661397,23490.81,23485.19


VREF ist stabil
