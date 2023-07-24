// TODO: Decrease precision from #.### to #.## to improve time til overflow from 49 days (7 weeks) to roughly 497 days (more than 1 year).
// TODO: URGENT: 2077 gets reduced to 2.77 in the serial debugging.
// TODO: Add rolling average display function, add clock time function as well.
// TODO: 350 feet to oven. Calculate the time it would take to reach there.
// TODO: might want to add a minimum distance range as well for object detection.

#include <Wire.h>
#include "Arduino.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_GFX.h"
#include "Adafruit_LEDBackpack.h"
#include "RTClib.h"

#define MAXIMUM_THRESHOLD_DISTANCE 1000 //millimeters
#define MINIMUM_THRESHOLD_DISTANCE 500 //millimeters
#define RTC_PRINT_INTERVAL 5 //seconds (the time in between RTC print outs)
#define OBJECT_DETECTION_TIMING_THRESHOLD 200 //milliseconds
#define ROLLING_AVERAGE_ARRAY_SIZE 10
#define INCHES_BETWEEN_VL53L1X_SENSORS 4
#define DISTANCE_FROM_MONITOR_TO_OVEN 350 //feet
#define TOTAL_LENGTH_OF_OVEN 225 //feet

// DS3231 Real Time Clock stuff
RTC_DS3231 rtc;
unsigned long previousMillis_RTCPrintOut = 0; // the last time the time was printed to serial from the RTC

// 1.2" 7-Segment Display stuff
Adafruit_7segment FPSDisplay = Adafruit_7segment();
Adafruit_7segment clockDisplay = Adafruit_7segment();

void printCurrentTime(DateTime);
void printDebugToSerial();
int findNextAvailableIndexInArray(unsigned long[]);
int addValueToArray(unsigned long[], unsigned long, int);
int addValueToArray(double[], double, int);
void calculateLineSpeed(int);
DateTime timeToStopHanging(DateTime);

unsigned long VLOneDetectionTimes[ROLLING_AVERAGE_ARRAY_SIZE]; // 0x2A (42)
unsigned long VLTwoDetectionTimes[ROLLING_AVERAGE_ARRAY_SIZE]; // 0x2B (43)
double lineSpeeds[ROLLING_AVERAGE_ARRAY_SIZE];

int VLOneTransitCounter = 0, VLTwoTransitCounter = 0;

// THE AMAZING VARIABLE CONTAINING WHAT WE CARE ABOUT MOST
double rollingAverageLineSpeed = 0;

// Time Of Flight Sensor (VL53L1X) Class Definition
class ToFSensor : public Adafruit_VL53L1X {
  public:
    ToFSensor(int);
    int I2C_ID;
    void takeMeasurement() { measurementDistance = distance(); };
    void objectDetectCheck();
    int measurementDistance;
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
  if (measurementDistance == -1 || measurementDistance >= MAXIMUM_THRESHOLD_DISTANCE) { // the sensor is no longer able to detect a T-Bar/Component
    if (!previousMeasurementLost) { // only runs once (the first time when sensor can no longer detect a T-Bar)
      millisObjectLost = millis(); // timestamp of when the sensor can no longer detect a t-bar

      Serial.print(F("VL53L1X ("));
      Serial.print(I2C_ID);
      Serial.println(F("): Can't see anything."));
      previousMeasurementLost = true;
    }

    if (millis() - millisObjectLost > OBJECT_DETECTION_TIMING_THRESHOLD) { // if the sensor doesn't see a t-bar for more than 0.2 seconds, consider the component has gone past the sensor
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

    if ((millis() - holdingMillis > OBJECT_DETECTION_TIMING_THRESHOLD) && !previousObjectDetected) {
      millisObjectDetected = holdingMillis;
      previousObjectDetected = true;

      int nextVLOneIndex = findNextAvailableIndexInArray(VLOneDetectionTimes);
      int nextVLTwoIndex = findNextAvailableIndexInArray(VLTwoDetectionTimes);

      if (I2C_ID == 0x2A) { // components pass by this sensor first

        // redundancy case if the second sensor misses a component.
        if (VLOneTransitCounter > VLTwoTransitCounter) {

          if (nextVLTwoIndex == -1) {
            addValueToArray(VLOneDetectionTimes, millisObjectDetected, ROLLING_AVERAGE_ARRAY_SIZE - 1);
            Serial.println(F("Data replaced."));
          } else {
            // insert the value at the latest matching index, and replace the current value.
            addValueToArray(VLOneDetectionTimes, millisObjectDetected, nextVLTwoIndex);
          }

          Serial.print(F("VL53L1X ("));
          Serial.print(I2C_ID);
          Serial.println(F("): Data discarded! (Sensor 43 missed data.)"));

        } else { // what we expect to happen
          addValueToArray(VLOneDetectionTimes, millisObjectDetected, nextVLOneIndex);

          VLOneTransitCounter++;

          Serial.print(F("VL53L1X ("));
          Serial.print(I2C_ID);
          Serial.println(F("): Object successfully detected!"));
        }

      } else if (I2C_ID == 0x2B) { // components pass by this sensor second

        // redundancy case if the first sensor misses a component.
        if (VLTwoTransitCounter == VLOneTransitCounter) {

          Serial.print(F("VL53L1X ("));
          Serial.print(I2C_ID);
          Serial.println(F("): Data discarded! (Sensor 42 missed data.)"));

        } else { // what we expect to happen
          addValueToArray(VLTwoDetectionTimes, millisObjectDetected, nextVLTwoIndex);
          calculateLineSpeed(nextVLTwoIndex);

          VLTwoTransitCounter++;

          Serial.print(F("VL53L1X ("));
          Serial.print(I2C_ID);
          Serial.println(F("): Object successfully detected!"));
        }

      }

      Serial.print(F("nextVLOneIndex: "));
      Serial.println(nextVLOneIndex);
      Serial.print(F("nextVLTwoIndex: "));
      Serial.println(nextVLTwoIndex);

      Serial.print(F("oneTransit: "));
      Serial.println(VLOneTransitCounter);
      Serial.print(F("twoTransit: "));
      Serial.println(VLTwoTransitCounter);

      Serial.println();
      for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
        Serial.print(VLOneDetectionTimes[i]);
        Serial.print(F(" "));
      }
      Serial.println();
      for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
        Serial.print(VLTwoDetectionTimes[i]);
        Serial.print(F(" "));
      }
      Serial.println();
      for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
        Serial.print(lineSpeeds[i]);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }
}

// VL53L1X Time of Flight stuff
ToFSensor VLSensorArray[] = { ToFSensor(0x2A), ToFSensor(0x2B) }; // IDs are hard coded simply because there are only two VL53L1X sensors being used.
const int xshutPins[2] = {5, 6};

// Counter that tracks the hertz rate of the program.
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

  // set all values in the two arrays to 0
  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    VLOneDetectionTimes[i] = 0;
    VLTwoDetectionTimes[i] = 0;
    lineSpeeds[i] = 0;
  }

  Serial.println(F("Setup is complete. The main program will now begin."));

}

void loop() {
  DateTime now = rtc.now();
  //printCurrentTime(now);

  if (VLSensorArray[0].dataReady()) {

    VLSensorArray[0].takeMeasurement();
    VLSensorArray[0].objectDetectCheck();

  }

  if (VLSensorArray[1].dataReady()) {

    VLSensorArray[1].takeMeasurement();
    VLSensorArray[1].objectDetectCheck();

  }

  DateTime stopHanging = timeToStopHanging(now);


  //FPSDisplay.print(timeToStopHanging(now));
  FPSDisplay.drawColon(true);
  FPSDisplay.writeDisplay();

  //clockDisplay.print(VLSensorArray[1].millisObjectDetected / 10);
  clockDisplay.print(rollingAverageLineSpeed);
  if (rollingAverageLineSpeed >= 100) {
    clockDisplay.writeDigitRaw(2, 0x10);
  } else {
    clockDisplay.writeDigitRaw(2, 0x02);
  }
  clockDisplay.writeDisplay();

  //printDebugToSerial();
  DEBUG_HERTZ_COUNTER++;

}

// MAIN FUNCTIONS

// Returns the next available index in an array, and -1 if the array is considered full and needs to be shifted to the left by one element.
int findNextAvailableIndexInArray(unsigned long array[]) {
  int nextAvailableIndex = -1;

  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    if (array[i] == 0) {
      nextAvailableIndex = i;
      break;
    }
  }
  
  return nextAvailableIndex;
}

// Adds an unsigned long to an unsigned long array into a specific index.
// If the 'index' input is -1, the array is considered full and is shifted down, creating a rolling array.
// Returns the index that the valueToInsert was ultimately placed in.
int addValueToArray(unsigned long array[], unsigned long valueToInsert, int index) {

  // If the index is -1, this means that the array is full with data values, and must be shifted down.
  if (index == -1) { 

    // shift all values in the array one element to the left to make room for the new value.
    for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE - 1; i++) { 
      array[i] = array[i+1];
    }

    // this sets the index variable (which lets the function know what index to add the valueToInsert variable to) to the maximum possible index, which
    // should be empty.
    index = ROLLING_AVERAGE_ARRAY_SIZE - 1;
  }

  // adds the valueToInsert into the array at the appropriate location.
  array[index] = valueToInsert;
  
  return index;
}

// Adds an unsigned long to an unsigned long array into a specific index.
// If the 'index' input is -1, the array is considered full and is shifted down, creating a rolling array.
// Returns the index that the valueToInsert was ultimately placed in.
int addValueToArray(double array[], double valueToInsert, int index) {

  // If the index is -1, this means that the array is full with data values, and must be shifted down.
  if (index == -1) { 

    // shift all values in the array one element to the left to make room for the new value.
    for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE - 1; i++) { 
      array[i] = array[i+1];
    }

    // this sets the index variable (which lets the function know what index to add the valueToInsert variable to) to the maximum possible index, which
    // should be empty.
    index = ROLLING_AVERAGE_ARRAY_SIZE - 1;
  }

  // adds the valueToInsert into the array at the appropriate location.
  array[index] = valueToInsert;

  return index;
}

void calculateLineSpeed(int insertIndex) {
  int dataIndex = insertIndex;

  if (insertIndex == -1) {
    dataIndex = ROLLING_AVERAGE_ARRAY_SIZE - 1;
  }

  double distanceBetweenSensorsInFeet = INCHES_BETWEEN_VL53L1X_SENSORS / 12.0;

  unsigned long firstSensorTime = VLOneDetectionTimes[dataIndex];
  unsigned long secondSensorTime = VLTwoDetectionTimes[dataIndex]; 

  unsigned long timeDifInMS = secondSensorTime - firstSensorTime;

  double timeDifInMinutes = timeDifInMS / 60000.0;

  double lineSpeedInFPM = distanceBetweenSensorsInFeet / timeDifInMinutes;

  addValueToArray(lineSpeeds, lineSpeedInFPM, insertIndex);

  // calculates the rolling average. dataIndex + 1 tells you how many values are in the array at the time, used to calculate the denominator of the average
  double sum = 0.0;

  for (int i = 0; i <= dataIndex; i++) {
    sum += lineSpeeds[i];
  }

  rollingAverageLineSpeed = sum / (dataIndex + 1);

  Serial.print(F("linespeed: "));
  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    Serial.print(lineSpeeds[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("rolling avr: "));
  Serial.println(rollingAverageLineSpeed);
  Serial.print(F("index: "));
  Serial.println(dataIndex);
  Serial.print(F("sum: "));
  Serial.println(sum);
}

DateTime timeToStopHanging(DateTime now) {
  
  int minutesUntilPartsReachOven = int((DISTANCE_FROM_MONITOR_TO_OVEN / rollingAverageLineSpeed) + 0.5);

  int minutesPartsInOven = int((TOTAL_LENGTH_OF_OVEN / rollingAverageLineSpeed + 0.5));

  DateTime lineStopTime (now.year(), now.month(), now.day(), 16, 30, 0);
  DateTime stopHangingTime (lineStopTime - TimeSpan(0, 0, (minutesUntilPartsReachOven + minutesPartsInOven), 0));

  return stopHangingTime;
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
  Serial.println(F("(#.### s)  "));
  Serial.println();
  
  Serial.println();
  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    Serial.print(VLOneDetectionTimes[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    Serial.print(VLTwoDetectionTimes[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  for (int i = 0; i < ROLLING_AVERAGE_ARRAY_SIZE; i++) {
    Serial.print(lineSpeeds[i]);
    Serial.print(F(" "));
  }
  Serial.println();

}

void printCurrentTime(DateTime now) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_RTCPrintOut >= (RTC_PRINT_INTERVAL * 1000)) {
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
    Serial.print(DEBUG_HERTZ_COUNTER / RTC_PRINT_INTERVAL);
    Serial.println(F(" Hertz."));
    Serial.println();

    DEBUG_HERTZ_COUNTER = 0;
    previousMillis_RTCPrintOut = millis();
    
  }  
}