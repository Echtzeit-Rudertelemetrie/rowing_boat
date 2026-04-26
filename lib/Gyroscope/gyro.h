#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Initialize the MPU6050 hardware
void setupGyro();

// Calibrate the Gyroscope to remove resting drift
// Note: Default parameter (2000) only goes in the header file!
void calibrateGyro(int samples = 2000);

// Read the sensor, integrate the angle, and print to Serial
// We pass the servoAngle in so we can print it alongside the gyro data
void sampleAndPlot(float servoAngle);

