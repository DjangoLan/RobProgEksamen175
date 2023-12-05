#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED oled;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;


int parameter = 15;
int stage_chl4 = 0;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
int threshold1 = 500;
int threshold2 = 600;
uint32_t turnAngle = 0;
double alignAngle = 0;

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}


void setup() {
  lineSensors.initFiveSensors();
  Serial.begin(9600);
  randomSeed(analogRead(0));
}



void challenge4() { //Patameteren vælges i challenge 0 (challenge selection)
  switch (stage_chl4) {
    case 0:
      align();
      stage_chl4++;
      resetEncoders();
      delay(50);
      break;
    case 1:
      delay(500);
      motors.setSpeeds(100, 100);
      if (parameter < getDistance()) {
        motors.setSpeeds(0, 0);
      }
  }
}

void loop() {
  challenge4();
}






void resetEncoders() {  //Nulstiller encodersne (og dermed afstandstælleren)
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

float getDistance() {  //funktion til at returnere kørt afstand ud fra encoders
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * 13;  //Encoder tæller 12 pr. fuld rotation. Motorernes gearforholdet er 1:75
  float distanceR = countsR / 900.0 * 13;  //12 * 75 = 900 svarer derfor til en fuld rotation. 13 er omkredsen af hjulet m. bæltet

  return (distanceL + distanceR) / 2;  //returnerer afstanden på venstre og højre hjul
}

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



