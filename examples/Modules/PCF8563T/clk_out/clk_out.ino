#include <pcf8563t.h>

PCF8563 rtc;

void setup() {
  Serial.begin(115200);
  rtc.init();//initialise PCF8563
  rtc.enableClkOut();//enable CLKOUT pin
}

void loop() {
    //set output frequency to 32768 Hz
    Serial.println("Outputing 32768Hz");
    rtc.setClkOutputFrequency(CLKOUT_32768_Hz);
    delay(10000);

    //set output frequency to 1024 Hz
    Serial.println("Outputing 1024_Hz");
    rtc.setClkOutputFrequency(CLKOUT_1024_Hz);
    delay(10000);

    //set output frequency to 32Hz
    Serial.println("Outputing 32Hz");
    rtc.setClkOutputFrequency(CLKOUT_32_Hz);
    delay(10000);

    //set output frequency to 1Hz
    Serial.println("Outputing 1Hz");
    rtc.setClkOutputFrequency(CLKOUT_1_Hz);
 
}