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
int threshold = 500;

uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;
// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;
// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;

int parameter = 40;
int allowedError = 1;
int stage = 0;
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
}

void loop() {
  challenge_6(40);
}

void challenge_6(int parameter) {
  switch (stage) {
    case 0:
      //align();
      turnSensorSetup();
      delay(500);
      turnSensorReset();
      oled.clear();
      stage = 1;
      break;
    case 1:
      int32_t turnDegrees = getTurnAngleInDegrees();
      oled.gotoXY(0, 0);
      oled.print(turnDegrees);
      oled.print(F("   "));
      if (turnDegrees >= (parameter - allowedError) && turnDegrees <= (parameter + allowedError)) {
        motors.setSpeeds(100, 100);
        delay(500);
        stage = 2;
      } else if (turnDegrees < (parameter - allowedError)) {
        motors.setSpeeds(-100, 100);
      } else if (turnDegrees > (parameter + allowedError)) {
        motors.setSpeeds(100, -100);
      }
      break;
    case 2:
      turnDegrees = getTurnAngleInDegrees();
      oled.gotoXY(0, 0);
      oled.print(turnDegrees);
      oled.print(F("   "));
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
      if (lineSensorValues[0] > threshold || lineSensorValues[2] > threshold || lineSensorValues[4] > threshold) {
        motors.setSpeeds(0, 0);
      } else if (turnDegrees >= (parameter - allowedError) && turnDegrees <= (parameter + allowedError)) {
        motors.setSpeeds(100, 100);
      } else if (turnDegrees < (parameter - allowedError)) {
        motors.setSpeeds(-100, 100);
      } else if (turnDegrees > (parameter + allowedError)) {
        motors.setSpeeds(100, -100);
      }
      break;
    default:
      oled.gotoXY(0, 0);
      oled.print("Fejl");
      motors.setSpeeds(0, 0);
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
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {} //gyroDataReady is a Bool returning function. Line 343 in Zumo32U4IMU.cpp
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
  turnAngle += (int64_t)d * 14680064 / 17578125;;
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // Calculate the turn angle in degrees
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}
