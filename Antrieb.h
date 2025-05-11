/****************************
 * Antrieb.h
 ****************************/
#ifndef ANTRIEB_H
#define ANTRIEB_H
#include <PID_v1.h>

class Antrieb {
public:
    // Konstruktur mit Pin-Definitionen, Regelungsparametern etc.
    Antrieb(int dirPin, int pwmPin, int brakePin, int encoderPinA, int encoderPinB,
            float kp, float ki, float kd);

    //Methodendeklarationen
    void begin();                   //festelgen der pinMode
    static void encoderISR();       //static void ist wichtig!
    void setSpeed(float speed);     // Soll-Geschwindigkeit
    void update(int dt);          // Aufruf z. B. in loop() zur Aktualisierung
    
    float getCurrentSpeed() const;  // Gibt die gemessene Drehzahl zurück
    float getSetpoint() const;      // Gibt den letzten Sollwert zurück
    float readEncoderSpeed(int dt); //liest Istgeschwindigkeit aus
    void applyOutput();             //Passt Drehzahl an

private:
    //Attributdeklarationen
    int _ppu=500;                   //Pulse pro Kanal und Umdrehung laut Datenblatt 100/500
    int _dirPin;
    int _pwmPin;
    int _brakePin;
    int _encoderPinA;
    int _encoderPinB;

    double _setpoint; // Soll-Drehzahl
    double _currentSpeed; // Ist-Drehzahl
    double _pidOutput; //Berechnete pwm vom PID-Objekt - double nicht int, da als Zeiger an PID Objekt übergeben -> double gefordert

    PID _controller; // eingebettetes Regelungsobjekt

    static volatile int _encoderPos;      //Zählt Quadraturencoder aus (static volatile sind wichtig)
};

#endif
