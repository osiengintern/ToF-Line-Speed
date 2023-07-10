// TODO: Decrease precision from #.### to #.## to improve time til overflow from 49 days (7 weeks) to roughly 497 days (more than 1 year).

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
    ToFSensor(int);
    int I2C_ID;
    void takeMeasurement() { measurementDistance = distance(); };
    void objectDetectCheck();
    int measurementDistance;
    int transitCounter;
    unsigned long millisObjectDetected, millisObjectLost, holdingMillis; 
    boolean previousObjectDetected, previousMeasurementLost;    
};

//HOLDING MILLIS
  // this variable will hold the time that an object was detected until 200 milliseconds have passed, after which it will pass the value on to millisObjectDetected.
  // this ensures that whenever millisObjectDetected is pulled, it is the most recent time that a confirmed object has passed the sensor.
ToFSensor::ToFSensor(int ID) {
  I2C_ID = ID, millisObjectDetected = 0, millisObjectLost = 0, holdingMillis = 0;
}

void ToFSensor::objectDetectCheck() {
// TODO: might want to add a minimum distance range as well.
  if (measurementDistance == -1 || measurementDistance >= REFLECTOR_DISTANCE) { // the sensor is no longer able to detect a T-Bar/Component
    if (!previousMeasurementLost) { // only runs once (the first time when sensor can no longer detect a T-Bar)
      millisObjectLost = millis(); // timestamp of when the sensor can no longer detect a t-bar

      Serial.print(F("VL53L1X ("));
      Serial.print(I2C_ID);
      Serial.println(F("): Can't see anything."));
      previousMeasurementLost = true;
    }

    if (millis() - millisObjectLost > OBJECT_DETECTION_THRESHOLD) { // if the sensor doesn't see a t-bar for more than 0.2 seconds, consider the component has gone past the sensor
      previousObjectDetected = false;
    }
  } else { // the sensor CAN see a t-bar/component    

    if (previousMeasurementLost) { // the first time the sensor does detect a t-bar
      holdingMillis = millis();
      
      Serial.print(F("VL53L1X ("));
      Serial.print(I2C_ID);
      Serial.println(F("): Possible object."));
      previousMeasurementLost = false;
    }

    if ((millis() - holdingMillis > OBJECT_DETECTION_THRESHOLD) && !previousObjectDetected) {
      millisObjectDetected = holdingMillis;
      previousObjectDetected = true;

      Serial.print(F("VL53L1X ("));
      Serial.print(I2C_ID);
      Serial.println(F("): Object successfully detected!"));
    }
  }
}

// VL53L1X Time of Flight stuff
ToFSensor VLSensorArray[] = { ToFSensor(0x2A), ToFSensor(0x2B) }; // IDs are hard coded simply because there are only two VL53L1X sensors being used.
const int xshutPins[2] = {5, 6};

unsigned long previousMillis_RTCPrintOut = 0; // the last time the time was printed to serial from the RTC
int DEBUG_HERTZ_COUNTER;

// function prototypes
void printCurrentTime(DateTime);
void printDebugToSerial();

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

    if (!VLSensorArray[j].begin(0x2A + j, &Wire)) { // 0x2A
      Serial.print(F("VL53L1X Sensor (ID: "));
      Serial.print(0x2A + j);
      Serial.print(F("): Initialization Error -> "));
      Serial.println(VLSensorArray[j].vl_status);
      while (1)       delay(10);
    }

    if (!VLSensorArray[j].startRanging()) { // 0x2B
      Serial.print(F("VL53L1X Sensor (ID: "));
      Serial.print(0x2A + j);
      Serial.print(F("): Couldn't start ranging -> "));
      Serial.println(VLSensorArray[j].vl_status);
      while (1)       delay(10);
    }

    Serial.print(F("VL53L1X Sensor (ID: "));
    Serial.print(0x2A + j);
    Serial.println(F("): Successfully initialized!"));

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    VLSensorArray[j].setTimingBudget(100);
    Serial.print(F("VL53L1X Sensor (ID: "));
    Serial.print(0x2A + j);
    Serial.print(F("): Timing budget in ms -> "));
    Serial.println(VLSensorArray[j].getTimingBudget());

  }

  Serial.println(F("All VL53L1X sensors are initialized!"));

  // 1.2" 7-Segment Display stuff
  FPSDisplay.begin(0x71);
  clockDisplay.begin(0x70);

  FPSDisplay.print("INIT");
  clockDisplay.print("INIT");
  FPSDisplay.writeDisplay();
  clockDisplay.writeDisplay();

  Serial.println(F("Setup is complete. The main program will now begin."));

}

void loop() {
  DateTime now = rtc.now();
  printCurrentTime(now);

  if (VLSensorArray[0].dataReady()) {

    VLSensorArray[0].takeMeasurement();
    VLSensorArray[0].objectDetectCheck();

  }

  if (VLSensorArray[1].dataReady()) {

    VLSensorArray[1].takeMeasurement();
    VLSensorArray[1].objectDetectCheck();

  }  

  FPSDisplay.print(VLSensorArray[0].millisObjectDetected / 10);
  FPSDisplay.drawColon(true);
  FPSDisplay.writeDisplay();

  clockDisplay.print(VLSensorArray[1].millisObjectDetected / 10);
  clockDisplay.writeDisplay();

  printDebugToSerial();
  DEBUG_HERTZ_COUNTER++;
}

void printDebugToSerial() {

  Serial.println(F(" \t \t 0x2A (42) \t 0x2B (43)"));
  Serial.print(F("Distances (mm):  "));
  Serial.print(VLSensorArray[0].measurementDistance);
  Serial.print(F(" \t \t "));
  Serial.println(VLSensorArray[1].measurementDistance);
  
  // TODO: improve the hertz rate. it's going from 90s to 70s because of the division i assume.
  Serial.print(F("Last Detection:  "));
  Serial.print(VLSensorArray[0].millisObjectDetected / 1000);
  Serial.print(F("."));
  Serial.print(VLSensorArray[0].millisObjectDetected % 1000);
  Serial.print(F(" \t \t "));
  Serial.print(VLSensorArray[1].millisObjectDetected / 1000);
  Serial.print(F("."));
  Serial.println(VLSensorArray[1].millisObjectDetected % 1000);
  Serial.println(F("(#.### s):  "));
  Serial.println();

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

    Serial.print(F("The program is running at "));
    Serial.print(DEBUG_HERTZ_COUNTER);
    Serial.println(F(" Hertz."));
    Serial.println();

    DEBUG_HERTZ_COUNTER = 0;
    previousMillis_RTCPrintOut = millis();
    
  }  
}