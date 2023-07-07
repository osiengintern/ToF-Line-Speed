#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "RTClib.h"

#define REFLECTOR_DISTANCE 1000 //millimeters
#define RTC_PRINT_INTERVAL 1000 //milliseconds (the time in between RTC print outs)
#define OBJECT_DETECTION_THRESHOLD 200 //milliseconds

// DS3231 Real Time Clock stuff
RTC_DS3231 rtc;

// 1.2" 7-Segment Display stuff
Adafruit_7segment FPSDisplay = Adafruit_7segment();
Adafruit_7segment clockDisplay = Adafruit_7segment();

// Time Of Flight Sensor (VL53L1X) Class Definition
class ToFSensor : public Adafruit_VL53L1X {
  public:
    ToFSensor() { millisObjectDetected = 0, millisObjectLost = 0; };
    void takeMeasurement() { distanceMeasurement = distance(); };
    void objectDetectCheck();
    int distanceMeasurement;
    int transitCounter;
    unsigned long millisObjectDetected, millisObjectLost;
    boolean previousObjectDetected, previousMeasurementLost;    
};

void ToFSensor::objectDetectCheck() {
// TODO: might want to add a minimum distance range as well.
  if (distanceMeasurement == -1 || distanceMeasurement >= REFLECTOR_DISTANCE) { // the sensor is no longer able to detect a T-Bar/Component
    if (!previousMeasurementLost) { // only runs once (the first time when sensor can no longer detect a T-Bar)
      millisObjectLost = millis(); // timestamp of when the sensor can no longer detect a t-bar

      Serial.println(F("VL53L1X: Can't see anything.")); // TODO: get Sensor ID showing.
      previousMeasurementLost = true;
    }

    if (millis() - millisObjectLost > OBJECT_DETECTION_THRESHOLD) { // if the sensor doesn't see a t-bar for more than 0.2 seconds, consider the component has gone past the sensor
      previousObjectDetected = false;
    }
  } else { // the sensor CAN see a t-bar/component

    // this variable will hold the time that an object was detected until 200 milliseconds have passed, after which it will pass the value on to millisObjectDetected.
    // this ensures that whenever millisObjectDetected is pulled, it is the most recent time that a confirmed object has passed the sensor.
    unsigned long holdingMillis; 

    if (previousMeasurementLost) { // the first time the sensor does detect a t-bar
      holdingMillis = millis();
      
      Serial.println(F("VL53L1X: Possible object.")); // TODO: get Sensor ID showing.
      previousMeasurementLost = false;
    }

    if ((millis() - holdingMillis > OBJECT_DETECTION_THRESHOLD) && !previousObjectDetected) {
      millisObjectDetected = holdingMillis;
      previousObjectDetected = true;

      Serial.println(F("VL53L1X: Object successfully detected!")); // TODO: get Sensor ID showing.
    }
  }
}

// VL53L1X Time of Flight stuff
ToFSensor VLSensorArray[2]; // hard coded simply because there are only two VL53L1X sensors being used.
const int xshutPins[2] = {5, 6};

unsigned long previousMillis_RTCPrintOut = 0; // the last time the time was printed to serial from the RTC

// function prototypes
void printCurrentTime(DateTime);

//DEBUG STUFF
int DEBUG_HERTZ_COUNTER;

void setup() {
  while (!Serial); // wait for serial port to connect. Needed for native USB
  Serial.begin(115200);
  Wire.begin();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  for (int i = 0; i < 2; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (int j = 0; j < 2; j++) {
    pinMode(xshutPins[j], INPUT);
    delay(10);

    if (!VLSensorArray[j].begin(0x2A + j, &Wire)) {
      Serial.print(F("VL53L1X Sensor (ID: "));
      Serial.print(0x2A + j);
      Serial.print(F("): Initialization Error -> "));
      Serial.println(VLSensorArray[j].vl_status);
      while (1)       delay(10);
    }

    if (!VLSensorArray[j].startRanging()) {
      Serial.print(F("VL53L1X Sensor (ID: "));
      Serial.print(0x2A + j);
      Serial.print(F("): Couldn't start ranging -> "));
      Serial.println(VLSensorArray[j].vl_status);
      while (1)       delay(10);
    }

    Serial.print(F("VL53L1X Sensor (ID: "));
    Serial.print(0x2A + j);
    Serial.print(F("): Successfully initialized!"));

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    VLSensorArray[j].setTimingBudget(15);
    Serial.print(F("VL53L1X Sensor (ID: "));
    Serial.print(0x2A + j);
    Serial.print(F("): Timing budget in ms -> "));
    Serial.println(VLSensorArray[j].getTimingBudget());

  }

  Serial.println(F("Ranging started"));

  // 1.2" 7-Segment Display stuff
  FPSDisplay.begin(0x71);
  clockDisplay.begin(0x70);
}

void loop() {
  DateTime now = rtc.now();
  printCurrentTime(now);

  if (VLSensorArray[0].dataReady()) {

    VLSensorArray[0].takeMeasurement();
    VLSensorArray[0].objectDetectCheck();   

    if (VLSensorArray[0].previousObjectDetected) {
      FPSDisplay.print(1, DEC);
    } else {
      FPSDisplay.print(0, DEC);
    }
    FPSDisplay.writeDisplay();
    
    int smthlikeyou = VLSensorArray[0].millisObjectDetected / 10;
    clockDisplay.print(smthlikeyou);
    clockDisplay.drawColon(true);
    clockDisplay.writeDisplay();

    Serial.println();
    Serial.println(smthlikeyou);
    Serial.println();

  }

  if (VLSensorArray[1].dataReady()) {

    // new distanceMeasurement for the taking!
    VLSensorArray[1].takeMeasurement();
    //VLSensorArray[1].objectDetectCheck();
    //^ uncommenting this somehow breaks the whole thing. i bet its smth with values from different sensors being tied up as a result of the class inheritance thing. check it out.

    //clockDisplay.print(VLSensorArray[1].distanceMeasurement);
    //clockDisplay.writeDisplay();
  }

  DEBUG_HERTZ_COUNTER++;

}

void printCurrentTime(DateTime now) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_RTCPrintOut >= RTC_PRINT_INTERVAL) {
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

    

    //DEBUG STUFF
    Serial.print(F("The program is running at "));
    Serial.print(DEBUG_HERTZ_COUNTER);
    Serial.println(F(" Hertz."));
    Serial.println();
    DEBUG_HERTZ_COUNTER = 0;
    
  }  
}