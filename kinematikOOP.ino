#include "Kinematik.h"
#include "Antrieb.h"

// Beispielhafte Parameter
#define WHEEL_RADIUS 0.05f    // 5 cm
#define PLATFORM_RADIUS 0.15f // 15 cm zum Mittelpunkt
#define REGULATION_INTERVAL 100 //ms

// Erzeuge 3 Antriebe int dirPin, int pwmPin, int brakePin, int encoderPinA, int encoderPinB, float kp, float ki, float kd
// PID-Parameter
float Kp = 0.1, Ki = 5.0, Kd = 0.0001;
Antrieb wheel1(21, 22, 23, 32, 33, Kp, Ki, Kd);
Antrieb wheel2(17, 18, 19, 34, 35, Kp, Ki, Kd);
Antrieb wheel3(12, 13, 14, 25, 26, Kp, Ki, Kd);

// Kinematik
Kinematik kinematics(WHEEL_RADIUS, PLATFORM_RADIUS);

unsigned long prevTime = 0; //previous time

//Sollgeschwindigkeit in x,y
float vx = 0.2; // m/s
float vy = 0.0; // m/s

//Aufgerufen erst zur Laufzeit
void setup() {
    Serial.begin(115200);
    wheel1.begin(); //pinModes dürfen erst im setup festgelegt werden (zur Laufzeit)
    wheel2.begin();
    wheel3.begin();
    // Weitere Initialisierung
}

void loop() {
    // 1) Regelung in Zeitintervall REGULATION_INTERVAL
    unsigned long currentTime = millis();
    if(currentTime-prevTime>=REGULATION_INTERVAL){ //statt if auch mit delay am Loopende möglich
      prevTime = currentTime;

      // 3) Kinematik berechnen
      float wheelSpeeds[3];
      kinematics.computeWheelSpeeds(vx, vy, wheelSpeeds);
  
      // 4) Für jeden Antrieb setSpeed() aufrufen
      wheel1.setSpeed(wheelSpeeds[0]);
      wheel2.setSpeed(wheelSpeeds[1]);
      wheel3.setSpeed(wheelSpeeds[2]);
  
      
      //Regelung der Radgeschwindigkeiten
      wheel1.update(REGULATION_INTERVAL);
      wheel2.update(REGULATION_INTERVAL);
      wheel3.update(REGULATION_INTERVAL);
  
      // 6) Debug-Ausgabe oder sonstiges
      Serial.print("Set: "); 
      Serial.print(wheel1.getSetpoint());
      Serial.print(" Ist: "); 
      Serial.println(wheel1.getCurrentSpeed());
    }
}
