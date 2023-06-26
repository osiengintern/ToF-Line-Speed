/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_VL53L1X.h"

//#define XSHUT_PIN 3

RTC_DS3231 rtc;
Adafruit_7segment matrix1 = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Keep track of the displayHours, displayMinutes, seconds displayed by the clock.
// Start off at 0:00:00 as a signal that the time should be read from
// the DS1307 to initialize it.
int displayHours = 0;
int displayMinutes = 0;
int seconds = 0;

int ifPMTrue = -1;

void setup () {
  Serial.begin(57600);

#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  matrix1.begin(0x70);
  matrix2.begin(0x71);

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

}

void loop () {
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");

    // calculate a date which is 7 days, 12 displayHours, 30 displayMinutes, 6 seconds into the future
    DateTime future (now + TimeSpan(7,12,30,6));

    Serial.print(" now + 7d + 12h + 30m + 6s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C");

    Serial.println();



    // 7 segment display component

    displayHours = now.hour();
    displayMinutes = now.minute();
    int displayTime = displayHours * 100 + displayMinutes;

    if (displayHours >= 12) {
      ifPMTrue = 1;
    } else {
      ifPMTrue = 0;
    }

    if (displayHours >= 13) {
      displayTime -= 1200;
    } else if (displayHours == 0) {
      displayTime += 1200;
    }

    matrix1.print(displayTime, DEC);
    matrix1.drawColon(true);

    if (ifPMTrue == 1) {
      matrix1.writeDigitRaw(2, 0x04);
    }


    matrix1.writeDisplay();





    //matrix2.print(1234, DEC);
    matrix2.printError();
    matrix2.writeDisplay();

    delay(1000);

}