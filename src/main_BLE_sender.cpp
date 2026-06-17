/***
 * BLE sender — telemetry hub (Seeed XIAO nRF52840, env: prod_telemetry)
 *
 * Role in the boat:
 *   Collects oarlock data over UART (RX) from the ESP-NOW receiver, reads the
 *   onboard GPS (u-blox, I2C/DDC @ 0x42) and 6-axis IMU (MPU6050), and
 *   broadcasts sequenced packets via Bluetooth Low Energy to up to 4 phones.
 *   Also responsible for battery charging management in the boat.
 *
 * Status (2026-06-17):
 *   Done  - GPS read (I2C, ~1 Hz debug) + IMU read (MPU6050 @ 100 Hz) to
 *           Serial/teleplot (>imu/...).
 *   TODO  - UART input from the receiver, BLE broadcast, battery charging.
 */


#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// GPS-Modul (u-blox, I2C/DDC) und IMU (MPU6050) teilen sich den I2C-Bus:
// XIAO SDA (D4) -> SDA, XIAO SCL (D5) -> SCL
const uint8_t GPS_I2C_ADDR = 0x42; // u-blox DDC Standardadresse
const unsigned long IMU_SAMPLE_INTERVAL_MS = 10; // 100 Hz
const unsigned long GPS_DEBUG_INTERVAL_MS = 1000; // 1 Hz Status-Ausgabe

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
unsigned long lastImuSample = 0;
unsigned long lastGpsDebug = 0;
bool imuOk = false;

// u-blox DDC: Register 0xFF liefert den naechsten Byte des NMEA/UBX-Streams,
// 0xFF selbst bedeutet "keine Daten verfuegbar"
void readGPS() {
    Wire.requestFrom(GPS_I2C_ADDR, (uint8_t)32);
    while (Wire.available()) {
        char c = Wire.read();
        if (c != (char)0xFF) {
            gps.encode(c);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);

    Serial.println("=== Telemetry: GPS (I2C) + IMU ===");

    imuOk = mpu.begin();
    if (!imuOk) {
        Serial.println("MPU6050 not found - continuing without IMU");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
}

void loop() {
    readGPS();

    if (gps.location.isUpdated()) {
        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print("  Lon: ");
        Serial.println(gps.location.lng(), 6);
    }

    if (gps.speed.isUpdated()) {
        Serial.print("Speed: ");
        Serial.print(gps.speed.mps());
        Serial.println(" m/s");
    }

    if (gps.course.isUpdated()) {
        Serial.print("Heading: ");
        Serial.print(gps.course.deg());
        Serial.println(" deg");
    }

    unsigned long now = millis();

    // Status-Ausgabe: hilft zu erkennen, ob ueberhaupt NMEA-Daten vom GPS
    // ankommen (charsProcessed), auch ohne Satelliten-Fix
    if (now - lastGpsDebug >= GPS_DEBUG_INTERVAL_MS) {
        lastGpsDebug = now;
        Serial.printf("GPS: chars=%lu sats=%d fix=%d checksumFail=%lu\n",
                      gps.charsProcessed(), gps.satellites.value(),
                      gps.location.isValid(), gps.failedChecksum());
    }

    if (imuOk && now - lastImuSample >= IMU_SAMPLE_INTERVAL_MS) {
        lastImuSample = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        Serial.printf(">imu/Acc_x: %.4f\n", a.acceleration.x);
        Serial.printf(">imu/Acc_y: %.4f\n", a.acceleration.y);
        Serial.printf(">imu/Acc_z: %.4f\n", a.acceleration.z);
        Serial.printf(">imu/Gyro_x: %.4f\n", g.gyro.x);
        Serial.printf(">imu/Gyro_y: %.4f\n", g.gyro.y);
        Serial.printf(">imu/Gyro_z: %.4f\n", g.gyro.z);
    }
}