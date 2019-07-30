#include <pcf8563t.h>

PCF8563 rtc;

void setup() {
  Serial.begin(115200);
  rtc.init();//initialize the clock

  rtc.stopClock();//stop the clock

  //set time to to 30/5/2019 17:33:0

  rtc.setYear(19);//set year
  rtc.setMonth(5);//set month
  rtc.setDay(30);//set dat
  rtc.setHour(17);//set hour
  rtc.setMinut(33);//set minut
  rtc.setSecond(0);//set second

  rtc.startClock();//start the clock
  Serial.println("RTC-Set clock!");
}

void loop() {
  Time nowTime = rtc.getTime();//get current time

  Serial.print("Time: ");
  //print current time
  Serial.print(nowTime.day);
  Serial.print("/");
  Serial.print(nowTime.month);
  Serial.print("/20");
  Serial.print(nowTime.year);
  Serial.print(" ");
  Serial.print(nowTime.hour);
  Serial.print(":");
  Serial.print(nowTime.minute);
  Serial.print(":");
  Serial.println(nowTime.second);
  delay(1000);
}