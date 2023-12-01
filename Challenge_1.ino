#include <Zumo32U4.h>
#include <Wire.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;

float calibratedDistance = 13;

float calculatedDistance = 0;
float drivenDistance = 0;
float wheelCirc = 13.0;

int speed = 100;

int stageChallenge1 = 0;


void setup() {
  // put your setup code here, to run once:
  proxSensors.initThreeSensors();
  // Define custom brightness levels for the LEDs
  uint16_t customLevels[] = {10, 40, 10};
  proxSensors.setBrightnessLevels(customLevels, 30);
  Serial.begin(9600);
}

void loop() {
 challenge1(20);
}

void stop() {
  motors.setSpeeds(0, 0);
}

void forward() {
  motors.setSpeeds(speed, speed);
}
void reverse() {
  motors.setSpeeds(-speed, -speed);
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

float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCirc;
  float distanceR = countsR / 900.0 * wheelCirc;

  Serial.println("Distance: ");
  Serial.println((distanceL + distanceR) / 2);

  return (distanceL + distanceR) / 2;
}

void buzzerCheck() {
  buzzer.playNote(NOTE_E(4), 350, 15);
}

void challenge1(int selectedDistance) {
  int distanceFromWall = selectedDistance;
  switch (stageChallenge1) {
    case 0:
      proximityRead();
      forward();
      if (proximityRead() >= 20) {
        stop();  //Slet hvis færdig
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
