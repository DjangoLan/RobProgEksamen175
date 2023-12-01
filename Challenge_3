#include <Zumo32U4.h>
#include <Wire.h>


Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;

// for line sensing
Zumo32U4LineSensors lineSensors;
#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];
int s1, s2, s3;

/* bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive; */

int leftSensor;
int centerLeftSensor;
int centerRightSensor;
int rightSensor;
//int challenge3Calibration = 7;
int threshold = 1000;

void setup() {
  Serial.begin(9600);
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels(50, 30);
  proxSensors.setPulseOffTimeUs(578);
  proxSensors.setPulseOnTimeUs(421);
}

void printReadingsToSerial() {
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d\n",
          lineSensorValues[0],
          lineSensorValues[1],
          lineSensorValues[2]);
  Serial.print(buffer);
}

// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark)
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  printReadingsToSerial();
}

void loop() {
  // put your main code here, to run repeatedly:
  challenge3();
}


void challenge3() {
  readLineSensors();
  if (lineSensorValues[0] < threshold && lineSensorValues[1] < threshold && lineSensorValues[2] < threshold) {
    proxRead3();
    Serial.print(leftSensor);
    Serial.print(" ");
    Serial.print(centerLeftSensor);
    Serial.print(" ");
    Serial.print(centerRightSensor);
    Serial.print(" ");
    Serial.println(rightSensor);

    if (leftSensor + centerLeftSensor < rightSensor + centerRightSensor) {
      motors.setSpeeds(55, 200);
    } else if (leftSensor + centerLeftSensor > rightSensor + centerRightSensor) {
      motors.setSpeeds(200, 55);
    } else {
      motors.setSpeeds(200, 200);
    }
  } else {
    motors.setSpeeds(0, 0);
    delay(1000);
  }
}

int proxRead3() {
  proxSensors.read();
  leftSensor = proxSensors.countsLeftWithLeftLeds();
  centerLeftSensor = proxSensors.countsFrontWithLeftLeds();
  centerRightSensor = proxSensors.countsFrontWithRightLeds();
  rightSensor = proxSensors.countsRightWithRightLeds();
}
