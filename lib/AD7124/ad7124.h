#ifndef AD7124_H
#define AD7124_H

#include <Arduino.h>

// AD7124 Register Adressen (Datenblatt)
#define AD7124_COMM_REG      0x00
#define AD7124_ADC_CONTROL  0x01
#define AD7124_ADC_STATUS   0x02
#define AD7124_CH0          0x09
#define AD7124_CONFIG0      0x19
#define AD7124_FILTER0      0x29
#define AD7124_DATA         0x30

// Kommunikation
#define AD7124_COMM_READ    0x80
#define AD7124_COMM_WRITE   0x00

// ADC_CONTROL Bits
#define AD7124_ADC_STATUS_RDY   (1 << 7)
#define AD7124_ADC_CONTROL_REF_EN (1 << 2)

// Filter Bits
#define AD7124_FILTER_FS(x)     ((x) & 0x7FF)

// --- Globale Variablen fuer Gleitender Mittelwert ---
extern volatile float ad7124_tara_uV;
extern volatile float ad7124_kraft;
extern volatile int   ad7124_sample_count;

// --- Funktionen ---
void ad7124_init();
void ad7124_update();
void ad7124_tara();

#endif