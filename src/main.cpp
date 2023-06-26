#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"

// VL53L1X Time of Flight Sensor stuffs
const int sensorCount = 2;
const int xshutPins[sensorCount] = {5, 6};
Adafruit_VL53L1X VL53Array[sensorCount];

// 1.2" 7-Segment Display stuff
Adafruit_7segment display1 = Adafruit_7segment();
Adafruit_7segment display2 = Adafruit_7segment();

void setup() {
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();

  // VL53L1X Time of Flight Sensor stuffs
  for (int i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (int j = 0; j < sensorCount; j++) {
    pinMode(xshutPins[j], INPUT);
    delay(10);

    if (!VL53Array[j].begin(0x2A + j, &Wire)) {
      Serial.print(F("Error on init of VL sensor: "));
      Serial.println(VL53Array[j].vl_status);
      while (1)       delay(10);
    }

    if (!VL53Array[j].startRanging()) {
      Serial.print(F("Couldn't start ranging: "));
      Serial.println(VL53Array[j].vl_status);
      while (1)       delay(10);
    }

    Serial.print(F("VL53L1X sensor with ID: 0x"));
    Serial.print(0x2A + j);
    Serial.println(F(" OK!"));

    VL53Array[j].setTimingBudget(100);
  }

  Serial.println(F("Ranging started"));

  // 1.2" 7-Segment Display stuff
  display1.begin(0x70);
  display2.begin(0x71);
}

void loop() {

  int VL53_0_distance;
  int VL53_1_distance;

  if (VL53Array[0].dataReady() && VL53Array[1].dataReady()) {

    // new measurement for the taking!
    VL53_0_distance = VL53Array[0].distance();
    VL53_1_distance = VL53Array[1].distance();

    if (VL53_0_distance == -1) {
      // something went wrong!
      Serial.print(F("VL53L1X (0x2A) couldn't get distance! -> "));
      Serial.println(VL53Array[0].vl_status);
      return;
    }

    if (VL53_1_distance == -1) {
      // something went wrong!
      Serial.print(F("VL53L1X (0x2B) couldn't get distance! -> "));
      Serial.println(VL53Array[1].vl_status);
      return;
    }

    Serial.println(F("Distances (mm): "));
    Serial.println("0x2A | 0x2B");
    Serial.print(" ");
    Serial.print(VL53_0_distance);
    Serial.print("  |  ");
    Serial.println(VL53_1_distance);

    // data is read out, time for another reading!
    VL53Array[0].clearInterrupt();
    VL53Array[1].clearInterrupt();

  }

  display1.print(VL53_0_distance);
  display2.print(VL53_1_distance);

  display1.writeDisplay();
  display2.writeDisplay();

  delay(100);

}