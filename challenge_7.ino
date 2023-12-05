#include <Wire.h>
#include <Zumo32U4.h>  //her henter vi biblioteket til zumo32u4

Zumo32U4Motors motors;  // og kalder vi på nogle forskellige klasser og angiver dem de "passende" navne.
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED oled;


#define NUM_SENSORS 5
int16_t lineSensorValues[NUM_SENSORS]; //array på størrelsen med 5
int threshold1 = 700;  //den værdi der gør at robotten skal stoppe til at align
int threshold2 = 700;  //den værdi der gør at robotten skal stoppe til at align
float distanceL;       // her har vi nogle varible i float og int og årsagen til det er float var så vi kunne vi kunne være mere præcis, komma tal
float distanceR;
int wheelCirc = 13;   // omkredsen på hjulet i cm
int threshold = 900;  // den værdi der gør at robotten skal stoppe
int stage_chl7 = 0; // bruges til switch statement
int i = 0; // bruges til switch igen i loop


void setup() {
  Serial.begin(9600); // vi aktivere det så den godt kan snakke sammen med computeren 9600 baudrate = 9600 bits pr. sekund
  lineSensors.initThreeSensors(); // initialiserer linesensonerne 3 
  lineSensors.initFiveSensors(); //initialiserer linesensonerne 5 
}


void readLineSensors() { //her læser man lineSensors
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}


void readLinesensor() {
  // Read the sensor values
  unsigned int sensors[3]; //array med størrelsen 3 eller også kaldet elementer. 
  lineSensors.read(sensors);

  // Print the sensor values
  Serial.print("Sensor Values: ");
  Serial.print("Left: ");
  Serial.print(sensors[0]);
  Serial.print(", Center: ");
  Serial.print(sensors[1]);
  Serial.print(", Right: ");
  Serial.println(sensors[2]);
}

void getDistance() {
  float countsL = encoders.getCountsLeft(); // man tæller hvor rotationen om hjulet.
  float countsR = encoders.getCountsRight(); 

  distanceL = countsL / 900 * wheelCirc; //regner ud hvor lang den har kørt.
  distanceR = countsR / 900 * wheelCirc; // 900 kliks pr. rotation, kommer fra forholdet, 12 er der fordi det er en roation på en steppermotor, og zumoet er 1:75, så 12*75=900

  Serial.println("Distance: "); //her printes der ud de forskkelige værdierne fra de forskellige sensonerne
  Serial.print("left:");
  Serial.print(distanceL);
  Serial.println(" ");
  Serial.print("Right:");
  Serial.print(distanceR);
  Serial.println(" ");
}



void loop() { //her har vi en switch statement. 
  switch (i) {
    case 0:
      align(); // i case 0 kører den align kode op
      i++; // når case 0 så er kørt så plusses der en på i, dvs så kører programemt den næste case.
    case 1: // køre case et
      challenge7(); // det er funktion challenge 7
  }
}



void challenge7() {
  switch (stage_chl7) { // her har vi switch statement
    case 0: //case 0
      getDistance(); // hvordan den skal forholde sig. 
      if (distanceL == distanceR) {
        motors.setSpeeds(200, 200);
      } else if (distanceR > distanceL) {
        motors.setSpeeds(150, 130);
      } else if (distanceR < distanceL) {
        motors.setSpeeds(130, 150);
      }
      stage_chl7++;
      break;
    case 1:
      delay(100); //delay op 100 så den lige kører ud så hele koden ikke kører og derefter stopper med det samme pga aline kode.
      stage_ch7++;
      break;
    case 2
      if (distanceL == distanceR) {
        motors.setSpeeds(200, 200);
      } else if (distanceR > distanceL) {
        motors.setSpeeds(150, 130);
      } else if (distanceR < distanceL) {
        motors.setSpeeds(130, 150);
      }

      readLinesensor(); // her kalder jeg på readlinesensonsor som er skerevet lægnere oppe. 

      // Read the line sensor values
      unsigned int lineSensorValues[3]; // vi har en array her med størelse på 3. 
      lineSensors.read(lineSensorValues); // læser af arrayet

      // Check if all line sensors are below the threshold
      if (lineSensorValues[0] > threshold || lineSensorValues[2] > threshold) { // her har vi en if statement der indeholder logical operatorer "||", "eller"
        stage_chl7++; // så når denne usagn er opfyldt går den videre til case 1 fordi man så sætter en plus på stage_chl7. 
        break;
      }
    case 3: // case et, motor stopper. 
      motors.setSpeeds(0, 0);
  }
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
