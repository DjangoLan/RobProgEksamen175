#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED oled;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
int threshold1 = 500;
int threshold2 = 600;
uint32_t turnAngle = 0;
double alignAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;
// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;
// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;
//int allignRepetitions = 30;  //Er nødvendig for allign

int i = 0;
int stage = 0;
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
  Serial.begin(9600);
  randomSeed(analogRead(0));
}

void loop() {
  challenge_6(40);
}

void challenge_6(int parameter) {
  switch (stage) {
    case 0:
      align();
      turnSensorSetup();
      delay(500);
      turnSensorReset();
      oled.clear();
/*      motors.setSpeeds(100, 100);
      delay(500);
      motors.setSpeeds(0, 0);*/
      stage=1;
      break;
    case 1:
      int32_t turnDegrees = getTurnAngleInDegrees();
      oled.gotoXY(0, 0);
      oled.print((((int32_t)turnAngle >> 16) * 360) >> 16);
      oled.print(F("   "));
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
      if (lineSensorValues[0] > threshold1 || lineSensorValues[2] > threshold1 || lineSensorValues[4] > threshold1) {
        motors.setSpeeds(0, 0);
      } else if (turnDegrees >= (parameter - 1) && turnDegrees <= (parameter + 1)) {
        motors.setSpeeds(200, 200);
      } else if (turnDegrees < (parameter - 1)) {
        motors.setSpeeds(-100, 100);
      } else if (turnDegrees > (parameter + 1)) {
        motors.setSpeeds(100, -100);
      }
      break;
  }
}

void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  oled.clear();
  oled.print(F("Gyro cal"));
  delay(500);
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  oled.clear();
  turnSensorReset();

  oled.clear();
}

void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += d;
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // Calculate the turn angle in degrees
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

void align() {
  int alignRepetitions = 100;  //Er nødvendig
  while (alignRepetitions > 0) {
    oled.gotoXY(0, 0);
    oled.print(alignRepetitions);
    delay(50);
    oled.clear();
    readLineSensors();

    if (lineSensorValues[0] < threshold2 && lineSensorValues[2] < threshold1 && lineSensorValues[4]) {
      motors.setSpeeds(50, 50);
      readLineSensors();
      delay(10);
    }

    if (lineSensorValues[4] > threshold2) {
      motors.setSpeeds(-50, -100);
      alignRepetitions--;
      if (lineSensorValues[0] > threshold2 || lineSensorValues[2] > threshold1) {
        motors.setSpeeds(0, 0);
      }
    }

    if (lineSensorValues[0] > threshold2) {
      motors.setSpeeds(-100, -50);
      alignRepetitions--;
      if (lineSensorValues[4] > threshold2 || lineSensorValues[2] > threshold1) {
        motors.setSpeeds(0, 0);
      }
    }
  }
}
