#include <Arduino.h>

// DMS-Kraftmessung: der Verstaerker wird ueber ein Build-Flag in der
// platformio.ini gewaehlt.
//   -D USE_AD7124   -> AD7124-8 (Hardware-SPI),   env: prod_ad7124
//   -D USE_NAU7802  -> NAU7802 (I2C),             env: prod_nau7802_s3
// Beide Treiber liegen in lib/DMS/.

#if defined(USE_NAU7802)
  #include "NAU7802/nau7802.h"
#elif defined(USE_AD7124)
  #include "AD7124/ad7124.h"
#else
  #error "Kein DMS-Verstaerker gewaehlt: -D USE_AD7124 oder -D USE_NAU7802 setzen"
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
#if defined(USE_NAU7802)
  Serial.println("\n=== NAU7802 DMS Messung ===");
  nau7802_init();
#else
  Serial.println("\n=== AD7124-8 DMS Messung ===");
  ad7124_init();
#endif
}

void loop() {
#if defined(USE_NAU7802)
  nau7802_update();
#else
  ad7124_update();
#endif
}
