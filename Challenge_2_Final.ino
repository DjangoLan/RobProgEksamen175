/*I denne challenge har vi brug for Zumo 32U4 front sensor array til at være forbundet  
og jumpere på front sensor array skal være installeret for at forbinde pin 4 til DN4 og pin 20 til DN2*/

#include <Wire.h>
#include <Zumo32U4.h>

/* Efterfølgende tilføjer vi alle vores funktioner som vi får brug for */

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];


void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

/*
Thresholds for sensorerne på træbanen

Linesensor 0 400
Linesensor 1 225
Linesensor 2 200
Linesensor 3 225
Linesensor 4 400
*/


void setup() {
  // put your setup code here, to run once:
  lineSensors.initFiveSensors();
  Serial.begin(9600);
  randomSeed(analogRead(0));
}

void challenge2() {

  int speed_chl2 = 350;          // der sættest en total fart for bægge motorer.
  int threshold_chl2 = 250;      // værdi for de 3 indre linsensore.
  int threshold2_chl2 = 600;     // værdi for de to ydre linsensore.
  int differential_chl2 = 250;   // differential fart for de indre linsensore.
  int differential2_chl2 = 700;  // differential fart for de indre linsensore.

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




void loop() {
  challenge2();
}
