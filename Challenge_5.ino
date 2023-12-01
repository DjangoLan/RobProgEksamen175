#include <Zumo32U4.h>
#include <Wire.h>


Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4OLED oled;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4IMU imu;

int leftIRValue;
int rightIRValue;
int frontIRValue;
int turnValue = 2;
int hookValue = 4;
bool turnReady = true;
int roundCounter = 0;
int stageChallenge5 = 0;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
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

void setup() {
  // put your setup code here, to run once:
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  //calibrateLineSensors();
  uint16_t levels[] = { 4, 15, 32, 55, 85, 120 };
  proxSensors.setBrightnessLevels(levels, sizeof(levels));
  //proxSensors.setBrightnessLevels(50, 30);
  proxSensors.setPulseOffTimeUs(578);
  proxSensors.setPulseOnTimeUs(421);

  turnSensorSetup();
  delay(500);
  turnSensorReset();
}

void loop() {
  challenge5();
}


void challenge5() {
  proxRead();
  oledPrint();
  gyroCount();
  switch (stageChallenge5) {
    case 0:
      checkTurnRight();
      if(leftIRValue >= turnValue && turnReady == true){
      forward();
      stageChallenge5 = 1;
      turnReady = false;
    }
    
      break;
    case 1:
      checkTurnLeft();
      if (rightIRValue >= turnValue && turnReady == true) {
        forward();
        stageChallenge5 = 0;
        turnReady = false;
        beep();
      }
      break;
  }
}

void proxRead() {
  static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100) {
    lastSampleTime = millis();

    // Send IR pulses and read the proximity sensors.
    proxSensors.read();

    // Read the IR sensor values.
    leftIRValue = (proxSensors.countsLeftWithLeftLeds() + proxSensors.countsLeftWithRightLeds()) / 2;
    frontIRValue = (proxSensors.countsFrontWithLeftLeds() + proxSensors.countsFrontWithRightLeds()) / 2;
    rightIRValue = (proxSensors.countsRightWithLeftLeds() + proxSensors.countsRightWithRightLeds()) / 2;


    // You now have the IR sensor values in the variables
    // leftIRValue, frontIRValue, and rightIRValue.
  }
}

void oledPrint() {
  oled.clear();
  oled.gotoXY(0, 1);
  oled.print(leftIRValue);
  oled.print(" ");
  oled.print(rightIRValue);
  oled.print(" |#");
  oled.print(roundCounter);

  oled.gotoXY(0, 0);
  oled.print(frontIRValue);
  oled.print("|");
  oled.print(getTurnAngleInDegrees()/2);
  oled.print("|");
  if(turnReady == false){oled.print("F");}
  if(turnReady == true){oled.print("T");}
  if(stageChallenge5 == 0){oled.print("R");}
  if(stageChallenge5 == 1){oled.print("L");}
}


void checkTurnRight() {
  if (rightIRValue == hookValue) {
    forward();
  } else if (rightIRValue < hookValue) {
    motors.setSpeeds(150, 50);
  }
  delay(20);
}
void checkTurnLeft() {
  if (leftIRValue == hookValue) {
    forward();
  } else if (leftIRValue < hookValue) {
    motors.setSpeeds(50, 150);
  }
  delay(20);
}


void stop() {
  motors.setSpeeds(0, 0);
}

void forward() {
  motors.setSpeeds(100, 100);
}


void beep() {
  buzzer.playNote(NOTE_A(4), 20, 15);
}

void calibrateLineSensors() {
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the display.
  ledYellow(1);
  oled.clear();
  oled.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++) {
    oled.gotoXY(0, 1);
    oled.print(i);
    lineSensors.calibrate();
  }

  ledYellow(0);
  oled.clear();
}

void gyroCount() {
  if (getTurnAngleInDegrees() >= 175) {
    turnSensorReset();
    roundCounter = roundCounter + 1;
    turnReady = true;
  }
}

int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // Calculate the turn angle in degrees
  return fabsf((((int32_t)turnAngle >> 16) * 360) >> 16);
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
