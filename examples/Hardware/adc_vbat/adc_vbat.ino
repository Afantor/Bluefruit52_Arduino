/*
  ADC get battery value
  get adc value from pin and print value.

  This example code is in the public domain.

  2019/01/03
  by Afantor
*/

#include <bluefruit52.h>
#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.87890625F)   // 3.6V ADC range and 12-bit ADC resolution = 3600mV/4096
#define VBAT_DIVIDER      (0.78740157F)   // 100K + 27K voltage divider on VBAT = (100K / (100K + 27K))  0.71275837F
#define VBAT_DIVIDER_COMP (1.27F)        // Compensation factor for the VBAT divider

int readVBAT(void) {
  int raw;

  // Set the analog reference to 3.6V (default = 3.6V)
  analogReference(AR_INTERNAL);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
  uint8_t battery_level;
  if (mvolts >= 3600)
  {
    battery_level = 100;
  }
  else if (mvolts > 3500)
  {
    battery_level = 100 - ((3600 - mvolts) * 58) / 100;
  }
  else if (mvolts > 3340)
  {
    battery_level = 42 - ((3500 - mvolts) * 24) / 160;
  }
  else if (mvolts > 3040)
  {
    battery_level = 18 - ((3340 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2700)
  {
    battery_level = 6 - ((3040 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }
  return battery_level;
}

// uint8_t mvToPercent(float mvolts) {
//   uint8_t battery_level;
//   if (mvolts >= 3700)
//   {
//     battery_level = 100;
//   }
//   else if (mvolts > 3632)//80%
//   {
//     battery_level = 100 - ((3700 - mvolts) * 29) / 100;
//   }
//   else if (mvolts > 3413)//60%
//   {
//     battery_level = 80 - ((3632 - mvolts) * 9) / 100;
//   }
//   else if (mvolts > 3191)//40%
//   {
//     battery_level = 60 - ((3413 - mvolts) * 9) / 100;
//   }
//   else if (mvolts > 2971)//20%
//   {
//     battery_level = 20 - ((3191 - mvolts) * 9) / 100;
//   }
//   else
//   {
//     battery_level = 0;
//   }
//   return battery_level;
// }
void setup() {

  BF52.begin(true, true, false);

  // Get a single ADC sample and throw it away
  readVBAT();
}

void loop() {
  // Get a raw ADC reading
  int vbat_raw = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095),
  // VBAT voltage divider is 2M + 0.806M, which needs to be added back
  float vbat_mv = (float)vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER_COMP;

  // Display the results
  Serial.print("ADC = ");
  Serial.print(vbat_raw * VBAT_MV_PER_LSB);
  Serial.print(" mV (");
  Serial.print(vbat_raw);
  Serial.print(") ");
  Serial.print("LIPO = ");
  Serial.print(vbat_mv);
  Serial.print(" mV (");
  Serial.print(vbat_per);
  Serial.println("%)");

  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setCursor(0, 0);  
  // Test some print formatting functions
   // Set the font colour to be blue with no background, set to font 4
  BF52.Lcd.setTextColor(YELLOW);    
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.print(" ADC = "); 
  BF52.Lcd.print(vbat_raw * VBAT_MV_PER_LSB);           // Print floating point number
  BF52.Lcd.println(" mV"); 
  BF52.Lcd.println(" "); 
  BF52.Lcd.print("LIPO = "); 
  BF52.Lcd.print(vbat_mv); // Print as integer value in binary
  BF52.Lcd.println(" mV"); 
  delay(5000);
}
