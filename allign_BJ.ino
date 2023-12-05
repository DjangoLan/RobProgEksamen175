/* for at vores zumo skal aligne med en sort tape linje har vi brug for Zumo 32U4 front sensor array til at være forbundet  
og jumpere på front sensor array skal være installeret for at forbinde pin 4 til DN4 og pin 20 til DN2 */

#include <Wire.h>
#include <Zumo32U4.h>

/* Efterfølgende tilføjer vi alle vores funktioner som vi får brug for */

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;
Zumo32U4IMU imu;

/* Der bliver defineret vores 5 linesensorer, og vi laver to thresholds, 
hvor den ene threshold er for de tre indre linesensore og den anden er for de to ydere linesensore. 
der er forskellige værdie på dem, fordi de to ydere linesonsore får mere lys til sig end de tre indre linesensore*/

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
int threshold1 = 500;
int threshold2 = 600;

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

void loop() {
  // put your main code here, to run repeatedly:

  /* vi tilføjer Zumo32U4 display og får den til at vise linesenorvalue for den der er mest til venstre (a.k.a. [0]),
så vi kan se hvordan den opfører sig når den rammer en sort tape linje. */

  display.gotoXY(0, 0);
  display.print(lineSensorValues[0]);
  //delay(50);
  //display.clear();
  readLineSensors();

  /* nu har vi tilføjet alt vi har brug for til at skrive den kode som gør at zumo´en udfører en allignment.
så vi laver nogle if statements hvor vi siger at hvis alle sensorværdierne er under thresholdet 
skal bægge motore køre lige ud med farten (50, 50)
så snart som en af de to ydere linesensore får en værdi der er højere (enn) threshold vil den køre baglæns,
hvor den ene motor køre lidt hurtigere (ann uppá) hvilken en af de ydere sensore rammer linjen.
Det kommer den til at gøre til de to ydere linesonsore rammer samtidig, fordi så stopper den og har klaret at aligne sig med linjen.

*/

  if (lineSensorValues[0] < threshold1 && lineSensorValues[2] < threshold1 && lineSensorValues[4] < threshold1) {
    motors.setSpeeds(50, 50);
    readLineSensors();
    delay(10);
  }
  if (lineSensorValues[3] > threshold2) {
    motors.setSpeeds(-50, -150);
    if (lineSensorValues[1] > threshold2) {
      motors.setSpeeds(0, 0);
    }
  }
  if (lineSensorValues[1] > threshold2) {
    motors.setSpeeds(-150, -50);
    if (lineSensorValues[3] > threshold2) {
      motors.setSpeeds(0, 0);
    }
  }
}



