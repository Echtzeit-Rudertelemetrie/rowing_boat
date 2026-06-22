#include "Battery.h"

void Battery::begin() {
#ifdef VBAT_ENABLE
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);            // connect the voltage divider
    pinMode(PIN_CHARGING_CURRENT, OUTPUT);
    digitalWrite(PIN_CHARGING_CURRENT, HIGH);  // default (slow ~50 mA) charge rate

    analogReference(AR_INTERNAL_3_0);          // 3.0 V reference
    analogReadResolution(12);                  // 0..4095
    delay(1);
#endif
}

BatteryData Battery::read() {
    BatteryData d{};
#ifdef VBAT_ENABLE
    uint16_t raw = analogRead(PIN_VBAT);
    float mv = ((float)raw * ADC_REF_MV / ADC_MAX) * DIVIDER_RATIO;
    d.millivolts = (uint16_t)(mv + 0.5f);

    int pct = (int)(((float)d.millivolts - BAT_EMPTY_MV) * 100.0f /
                    (BAT_FULL_MV - BAT_EMPTY_MV));
    d.percent = (uint8_t)constrain(pct, 0, 100);

    // USB 5V present => charger active (charging or topped off).
    #if defined(NRF_POWER)
    d.charging = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) ? 1 : 0;
    #else
    d.charging = 0;
    #endif
#endif
    return d;
}
