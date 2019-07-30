/* Demonstration of PCF8563T Clock on LCD. 
 *
 * I used a RBBB with Arduino IDE, the pins are mapped a 
 * bit differently.  Change for your hw.
 * SCK - A5, SDA - A4
 *
 * This sketch connects a lcd to display the clock output.
 * 
 * setup:  see Pcf8563 data sheet.
 *         1x 10Kohm pullup on INT
 *         No pullups on Pin5 or Pin6 (I2C internals used)
 *         1x 0.1pf on power
 *         1x 32khz chrystal
 */

#include <bluefruit52.h>
#include <pcf8563t.h>

PCF8563 rtc;

void setup() {
  BF52.begin(true, true, false);
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
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setCursor(0, 10);
  BF52.Lcd.setTextColor(YELLOW); 
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.println("Time : ");
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
  BF52.Lcd.drawNumber(0, 60, nowTime.day, 2, YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(24, 60, '/', YELLOW, BLACK, 2);
  BF52.Lcd.drawNumber(36, 60, nowTime.month, 2, YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(60, 60, '/', YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(72, 60, '2', YELLOW, BLACK, 2);  
  BF52.Lcd.drawChar(84, 60, '0', YELLOW, BLACK, 2);
  BF52.Lcd.drawNumber(96, 60, nowTime.year, 2, YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(120, 60, ' ', YELLOW, BLACK, 2);
  BF52.Lcd.drawNumber(132, 60, nowTime.hour, 2, YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(156, 60, ':', YELLOW, BLACK, 2);
  BF52.Lcd.drawNumber(168, 60, nowTime.minute, 2, YELLOW, BLACK, 2);
  BF52.Lcd.drawChar(192, 60, ':', YELLOW, BLACK, 2);
  BF52.Lcd.drawNumber(204, 60, nowTime.second, 2, YELLOW, BLACK, 2);
  delay(1000);
}
