/****************************
 * Kinematik.h (Interface)
 ****************************/
//if not defined ifndef
#ifndef KINEMATIK_H
#define KINEMATIK_H

class Kinematik {
public:
    Kinematik(float wheelRadius, float platformRadius); //Konstruktor
    void computeWheelSpeeds(float vx, float vy, float wheelSpeeds[3]); //Public Function

//Attribute
private:
    float _wheelRadius;
    float _platformRadius;
};

#endif
