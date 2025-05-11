/****************************
 * Kinematik.cpp
 ****************************/
#include "Kinematik.h"
#include <Arduino.h> // falls benötigt

//Konstruktor
Kinematik::Kinematik(float wheelRadius, float platformRadius)
: _wheelRadius(wheelRadius), _platformRadius(platformRadius)
{
}

void Kinematik::computeWheelSpeeds(float vx, float vy, float wheelSpeeds[3]) {
    // Beispiel für 3 Omni-Räder im 120° Versatz
    // Annahmen:
    // Rad1: 0°, Rad2: 120°, Rad3: 240°
    // Ausrichtung: tangential, also Achse 90° versetzt
    // Hier musst du deine kinematische Herleitung einfügen.
    // Der Einfachheit halber ein grobes Beispiel:
    // Rad i Speed = (vx * cos(Alpha_i) + vy * sin(Alpha_i)) / Raddurchmesser
    // oder ähnliche Formel. Du passt sie an deine Geometrie an.

    // Beispiel: Reine translatorische (keine Rotation)
    // rad1
    float alpha1 = 0.0; 
    wheelSpeeds[0] = (vx * cos(alpha1) + vy * sin(alpha1)) / _wheelRadius;
    wheelSpeeds[0]=10; //damit ich mich erstmal nicht um Berechnung kümmern muss

    // rad2
    float alpha2 = 2.0 * PI / 3.0; // 120°
    wheelSpeeds[1] = (vx * cos(alpha2) + vy * sin(alpha2)) / _wheelRadius;
    wheelSpeeds[1]=20;


    // rad3
    float alpha3 = 4.0 * PI / 3.0; // 240°
    wheelSpeeds[2] = (vx * cos(alpha3) + vy * sin(alpha3)) / _wheelRadius;
    wheelSpeeds[2]=30;

}
