#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "RTClib.h"

// DS3231 Real Time Clock stuff
RTC_DS3231 rtc;

// 1.2" 7-Segment Display stuff
Adafruit_7segment FPSDisplay = Adafruit_7segment();
Adafruit_7segment clockDisplay = Adafruit_7segment();

// VL53L1X Class Definition
class ToFSensor {
  public:
    ToFSensor();
    int getDistance();
  private:
    Adafruit_VL53L1X VL53Instance;
    unsigned long millisObjectDetected;
    unsigned long millisObjectLost;
};

ToFSensor::ToFSensor() {
  VL53Instance = Adafruit_VL53L1X();
  millisObjectDetected, millisObjectLost = 0;
}

int ToFSensor::getDistance() {
  return VL53Instance.distance();
}


// counters and other relevant variables
int vl53_distance;
int transitCounter = 0;
boolean isObjectDetected = false;
boolean pastObjectDetect = false;

unsigned long timeWhenTOFLost = 0, timeWhenTOFRestored, timeTOFDifference;
boolean isVL53Lost = false;


unsigned long previousMillis_RTCPrintOut = 0; // the last time the time was printed to serial from the RTC
long RTCPrintInterval = 1000; // the time (in milliseconds) in between RTC print outs

// function prototypes
void VL53IsLost();
void printCurrentTime(DateTime);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // VL53L1X Time of Flight stuff
  ToFSensor ToFOne, ToFTwo;
  int distanceOne = ToFOne.getDistance();

  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  
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
  vl53.setTimingBudget(15);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
  
  //vl53.VL53L1X_SetDistanceMode(1); i dont think this does anything

  // 1.2" 7-Segment Display stuff
  FPSDisplay.begin(0x71);
  clockDisplay.begin(0x70);
}

void loop() {
  DateTime now = rtc.now();
  printCurrentTime(now);

  if (vl53.dataReady()) {

    // new measurement for the taking!
    vl53_distance = vl53.distance();

    if (vl53_distance)



    if (vl53_distance == -1) { // the sensor is no longer able to detect something
      if (!isVL53Lost) { // only runs once (the first time when measurements are lost)
        VL53IsLost();
      }
      return;
    } else {
      isVL53Lost = false;
    }

    /*if (vl53_distance == -1) {
      // something went wrong! (lost object)

      if (!wasTOFLost) {
        timeWhenTOFLost = millis();
        wasTOFLost = true;
        pastObjectDetect = false;
      }

      FPSDisplay.print(99999, DEC);
      FPSDisplay.writeDisplay();

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
    }*/

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

    Serial.print(F("Distances (mm): "));
    Serial.println(vl53_distance);
    Serial.println();

    // data is read out, time for another reading!
    vl53.clearInterrupt();

  }

  FPSDisplay.print(vl53_distance);
  FPSDisplay.writeDisplay();

  clockDisplay.print(transitCounter);
  clockDisplay.writeDisplay();


  //delay(200);

}

void VL53IsLost() {
  timeWhenTOFLost = millis();


  Serial.print(F("VL53L1X couldn't get distance!"));
  isVL53Lost = true;
}

void printCurrentTime(DateTime now) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_RTCPrintOut >= RTCPrintInterval) {
    Serial.println();
    Serial.print(F("Current Time from DS3231 RTC: "));
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(F(" "));
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    previousMillis_RTCPrintOut = millis();
  }  
}