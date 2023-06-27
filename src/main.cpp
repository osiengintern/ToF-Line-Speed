#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "RTClib.h"

// DS3231 Real Time Clock stuff
RTC_DS3231 rtc;

// VL53L1X Time of Flight stuff
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

// 1.2" 7-Segment Display stuff
Adafruit_7segment display1 = Adafruit_7segment();
Adafruit_7segment display2 = Adafruit_7segment();

// counters and other relevant variables
int vl53_distance;
int transitCounter = 0;
boolean isObjectDetected = false;
boolean pastObjectDetect = false;

unsigned long timeWhenTOFLost = 0, timeWhenTOFRestored, timeTOFDifference;
boolean wasTOFLost = false;

void setup() {
  while (!Serial) {}
  Serial.begin(115200);
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
  vl53.setTimingBudget(200);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
  
  vl53.VL53L1X_SetDistanceMode(1);

  // 1.2" 7-Segment Display stuff
  display1.begin(0x71);
  display2.begin(0x70);
}

void loop() {

  if (vl53.dataReady()) {

    // new measurement for the taking!
    vl53_distance = vl53.distance();

    if (vl53_distance == -1) {
      // something went wrong! (lost object)

      if (!wasTOFLost) {
        timeWhenTOFLost = millis();
        wasTOFLost = true;
        pastObjectDetect = false;
      }

      display1.print(99999, DEC);
      display1.writeDisplay();

      Serial.print(F("VL53L1X couldn't get distance! -> "));
      Serial.println(vl53.vl_status);
      return;
    }

    if (wasTOFLost) {
      timeWhenTOFRestored = millis();
      timeTOFDifference = timeWhenTOFRestored - timeWhenTOFLost; // time (in ms) it took for measurements from the VL53L1X to be restored.

      if (timeTOFDifference < 200) {
        return;
      } else {
        wasTOFLost = false;
      }
    }

    if (vl53_distance <= 800) {
      isObjectDetected = true;
    } else {
      isObjectDetected = false;
    }

    if (isObjectDetected && !pastObjectDetect) {
      transitCounter++;
      pastObjectDetect = true;
    } else if (!isObjectDetected) {
      pastObjectDetect = false;
    }

    Serial.println(isObjectDetected);
    Serial.println(pastObjectDetect);
    Serial.println();

    Serial.println(F("Distances (mm): "));
    Serial.println(vl53_distance);

    // data is read out, time for another reading!
    vl53.clearInterrupt();

  }

  display1.print(vl53_distance);
  display1.writeDisplay();

  display2.print(transitCounter);
  display2.writeDisplay();


  delay(100);

}