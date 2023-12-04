#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4IMU imu;
Zumo32U4OLED oled;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;

int parameter = 0;  // The parameter determening the value needed for certain challenges

// variables for gyro (challenge 3)
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

int chosenChallenge = 1;  // Current challenge number

// control the flow of the program. 0 wait for command
//                                  1 wait for parameter
//                                  2 running the command.
int stage = 0;

// value used for switch in challenge 1
int stageChallenge1 = 0;
// known distance from wall used in challenge 1
float calibratedDistance = 13;
// floats used in challenge 1
float calculatedDistance = 0;
float drivenDistance = 0;
float wheelCirc = 13.0;

// value used for setup in challenge 3 and 5
int chal = 0;
// values uses in challenge 3
int leftSensor;
int centerLeftSensor;
int centerRightSensor;
int rightSensor;
int threshold = 1000;

int chosenCommand = 0;  //ændre snere med navn

// values used for challenge 3
int stage_chl6 = 0;
int threshold1 = 500;

//values used for challenge 4
int stage_chl4 = 0;  // used in challenge 4

// values used for challenge 5
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

//threshold for align code
int threshold2 = 600;

// Variables for line sensors
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  randomSeed(analogRead(0));
  proxSensors.setBrightnessLevels(50, 30);
  proxSensors.setPulseOffTimeUs(578);
  proxSensors.setPulseOnTimeUs(421);
  delay(100);
}

void loop() {
  switch (stage) {  // starting a switch with the parameter 'stage'
    case 0:
      selectChallenge();
      break;
    case 1:
      if (chosenChallenge == 1 || chosenChallenge == 4 || chosenChallenge == 6) {
        selectParameter();
      } else {
        stage = 2;
      }
      break;
    case 2:
      if (chosenChallenge == 1) {
        align();
        challenge1(parameter);
      } else if (chosenChallenge == 2) {
        challenge2();

      } else if (chosenChallenge == 3) {
        //challenge 3 needs a special configuration for proximity sensing
        switch (chal) {
          case 0:
            // initializes sensors
            lineSensors.initThreeSensors();
            proxSensors.initThreeSensors();
            proxSensors.setBrightnessLevels(50, 30);
            proxSensors.setPulseOffTimeUs(578);
            proxSensors.setPulseOnTimeUs(421);
            chal3++;
            delay(1000);
            break;
          case 1:
            challenge3();
            break;
        }

      } else if (chosenChallenge == 4) {
        challenge4(parameter);

      } else if (chosenChallenge == 5) {
                switch (chal) {
          case 0:
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
            break;
          case 1:
            challenge5();
            break;

      } else if (chosenChallenge == 6) {
        challenge_6(parameter);

      } else if (chosenChallenge == 7) {
      }
      break;
  }
}

void selectChallenge() {
  readEncodersMovement();
  int const countJump = 80;  // number of counts before jumping to next command in stage 0

  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("Challeng");
  oled.gotoXY(0, 1);
  oled.print("e: ");
  oled.gotoXY(2, 1);
  oled.print(chosenChallenge);

  if (chosenCommand > countJump) {
    bip();
    chosenCommand = 0;
    chosenChallenge++;
    if (chosenChallenge > 7) chosenChallenge = 1;
  } else if (chosenCommand < -countJump) {
    bip();
    chosenCommand = 0;
    chosenChallenge--;
    if (chosenChallenge < 1) chosenChallenge = 7;
  }
  button();
}

// Selects the parameter for challenges needing a set perameter
void selectParameter() {
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("Value:");
  parameter = (encoders.getCountsRight() / 10);
  oled.gotoXY(0, 1);
  oled.print(parameter);
  delay(50);
  button();
}
// challenge 1 code
void challenge1(int determedDistance) {
  int distanceFromWall = determedDistance;
  switch (stageChallenge1) {
    case 0:
      proximityRead();
      forward();
      if (proximityRead() >= 20) {
        if (distanceFromWall >= calibratedDistance) {
          stageChallenge1 = 1;
          reverse();

        } else {
          stageChallenge1 = 2;
          forward();
        }
      }
      break;

    case 1:
      getDistance();
      if (getDistance() * (-1) >= distanceFromWall - calibratedDistance) {
        stop();
      }
      break;

    case 2:
      getDistance();
      if (getDistance() >= calibratedDistance - distanceFromWall) {
        stop();
      }
      break;
  }
}
float proximityRead() {
  // Read proximity sensor values
  uint16_t centerValueL, centerValueR;

  proxSensors.read();  // Read all sensors

  // Retrieve the sensor values
  centerValueL = proxSensors.countsFrontWithLeftLeds();
  centerValueR = proxSensors.countsFrontWithRightLeds();

  float centerValue = (centerValueR + centerValueL) / 2;

  // Print sensor values to the serial monitor
  Serial.print("Center: ");
  Serial.println(centerValue);

  return centerValue;
  delay(5);
}
// challenge 1 get distance
float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCirc;
  float distanceR = countsR / 900.0 * wheelCirc;

  Serial.println("Distance: ");
  Serial.println((distanceL + distanceR) / 2);

  return (distanceL + distanceR) / 2;
}
// motor settings for challenge 1
void stop() {
  motors.setSpeeds(0, 0);
}
void forward() {
  motors.setSpeeds(100, 100);
}
void reverse() {
  motors.setSpeeds(-100, -100);
}

// challenge 2 code
void challenge2() {

  int speed_chl2 = 350;
  int threshold_chl2 = 250;
  int threshold2_chl2 = 600;
  int differential_chl2 = 250;
  int differential2_chl2 = 700;

  readLineSensors();
  if (lineSensorValues[2] > threshold_chl2) {  //Hvis sensor 0 og 4 ikke ser linjen, korrigerer den med sensor 1, 2 og 3
    motors.setSpeeds(speed_chl2, speed_chl2);
    if (lineSensorValues[1] > threshold_chl2) {
      motors.setSpeeds(speed_chl2 - differential_chl2, speed_chl2);
    } else if (lineSensorValues[3] > threshold_chl2) {
      motors.setSpeeds(speed_chl2, speed_chl2 - differential_chl2);
    }
  }
  if (lineSensorValues[0] > threshold2_chl2) {  //Hvis sensor 0 eller 4 ser linjen drejer den skarpt
    motors.setSpeeds(speed_chl2 - differential2_chl2, speed_chl2);
    delay(40);
  }
  if (lineSensorValues[4] > threshold2_chl2) {
    motors.setSpeeds(speed_chl2, speed_chl2 - differential2_chl2);
    delay(40);
  }
}

// challenge 3 code
void challenge3() {
  readLineSensors();
  if (lineSensorValues[0] < threshold && lineSensorValues[2] < threshold && lineSensorValues[4] < threshold) {
    proxRead3();
    if (leftSensor + centerLeftSensor < rightSensor + centerRightSensor) {
      motors.setSpeeds(10, 200);
    } else if (leftSensor + centerLeftSensor > rightSensor + centerRightSensor) {
      motors.setSpeeds(200, 10);
    } else {
      motors.setSpeeds(200, 200);
    }
  } else {
    motors.setSpeeds(0, 0);
    delay(1000);
  }
}
// proximity reader for challenge 3
int proxRead3() {
  proxSensors.read();
  leftSensor = proxSensors.countsLeftWithLeftLeds();
  centerLeftSensor = proxSensors.countsFrontWithLeftLeds();
  centerRightSensor = proxSensors.countsFrontWithRightLeds();
  rightSensor = proxSensors.countsRightWithRightLeds();
}

// challenge 4
void challenge4(int parameter) {  //Patameteren vælges i challenge 0 (challenge selection), og bestemmer afstanden robotten kører efter den har alignet
  int parameter4 = parameter;
  switch (stage_chl4) {
    case 0:  //Robotten aligner sig til tapen, resetter encodersne og
      align();
      stage_chl4++;
      resetEncoders();
      break;
    case 1:  //Kører fremad efter et halvt sekund, indtil kørte afstand er over den valgte parameter
      delay(500);
      motors.setSpeeds(80, 80);
      if (parameter4 < getDistance()) {
        motors.setSpeeds(0, 0);
      }
  }
}

// Challenge 5 code
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

//Challenge 6 code
void challenge_6(int parameter) {
  switch (stage_chl6) {
    case 0:
      align();
      turnSensorSetup();
      delay(500);
      turnSensorReset();
      oled.clear();
      motors.setSpeeds(100, 100);
      delay(500);
      motors.setSpeeds(0, 0);
      stage_chl6 = 1;
      break;
    case 1:
      int32_t turnDegrees = getTurnAngleInDegrees();
      oled.gotoXY(0, 0);
      oled.print((((int32_t)turnAngle >> 16) * 360) >> 16);
      oled.print(F("   "));
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
      printReadingsToSerial();
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

// global angle meassurer
int32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // Calculate the turn angle in degrees
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}
// global turn sensor updator
void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += d;
}
//global align function
void align() {
  int alignRepetitions = 200;  //Tæller 1 ned fra 100 hver loop i while-løkken hvor linesensor 0 eller 4 ser tape

  while (alignRepetitions > 0) {  //Looper alignfunktionen mens alignRepetitions er over 0
    oled.gotoXY(0, 0);
    oled.print(alignRepetitions);  //Viser alignRepetitions-værdien på displayet
    readLineSensors();

    if (lineSensorValues[0] < threshold2 && lineSensorValues[2] < threshold1 && lineSensorValues[4]) {
      motors.setSpeeds(50, 50);
      delay(30);
    }  //Hvis ingen sensorer ser tapen, kører robotten fremad med en hastighed

    if (lineSensorValues[4] > threshold2) {  //Hvis højre sensor ser tapen, bakker robotten bagud til venstre
      motors.setSpeeds(-50, -100);
      alignRepetitions--;
      if (lineSensorValues[0] > threshold2 || lineSensorValues[2] > threshold1) {  //Hvis højre og midterste sensor ser tapen sættes hastigheden til 0
        motors.setSpeeds(0, 0);
      }
      delay(20);
    }

    if (lineSensorValues[0] > threshold2) {  //Hvis venstre sensor ser tapen, bakker robotten bagud til højre
      motors.setSpeeds(-100, -50);
      alignRepetitions--;
      if (lineSensorValues[4] > threshold2 || lineSensorValues[2] > threshold1) {  //Hvis venstre og midterste sensor ser tapen sættes hastigheden til 0
        motors.setSpeeds(0, 0);
      }
      delay(20);
      oled.clear();  //"Clearer" skærmen, så en ny værdi kan printes.
    }
  }
}
// makes funny bib haha
void bip() {
  buzzer.playFrequency(1000, 1000, 15);
  delay(70);
  buzzer.stopPlaying();
}
// global motor encoder reader
void readEncodersMovement() {
  chosenCommand = chosenCommand + encoders.getCountsRight();
  resetEncoders();
}
// global line sensor reader - used in multiple challenges
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  printReadingsToSerial();
}
// global function - used in multiple challenges
void printReadingsToSerial() {
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d\n",
          lineSensorValues[0],
          lineSensorValues[1],
          lineSensorValues[2]);
  Serial.print(buffer);
}
// global function for resetting the encoders
void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}
// determines if button a is pressed and goes on to the next stage after doing so
void button() {
  if (buttonA.isPressed()) {
    bip();
    stage = stage + 1;
    buttonA.waitForRelease();
  }
}
// setup for turn sensors
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  oled.clear();
  oled.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  oled.clear();
  turnSensorReset();
}
// reset for turn sensors
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}
// countdown before challenge
void countdown() {
  for (int t = 0; t < 5; t++) {
    oled.clear();
    oled.print("Cuntdown: ");
    oled.gotoXY(0, 1);
    oled.println(String(5 - t));

    buzzer.playFrequency(1000, 1000, 15);
    delay(500);
    buzzer.stopPlaying();
    delay(500);
  }
}
