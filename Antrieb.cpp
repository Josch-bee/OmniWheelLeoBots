/****************************
 * Antrieb.cpp
 ****************************/
#include "Antrieb.h"
#include <Arduino.h>

Antrieb::Antrieb(int dirPin, int pwmPin, int brakePin, int encoderPinA, int encoderPinB,
                 float kp, float ki, float kd)
: _dirPin(dirPin), _pwmPin(pwmPin), _brakePin(brakePin), _encoderPinA(encoderPinA), _encoderPinB(encoderPinB),
  _currentSpeed(0.0), _pidOutput(0), _setpoint(0.0),
  _controller(&_currentSpeed, &_pidOutput, &_setpoint, kp, ki, kd,DIRECT) //encoderPos darf in Konstruktor nicht initialisiert werden, da static (kein Klassenattribut)
{
  _controller.SetMode(AUTOMATIC); //Automatische konitnuierliche Berechnung des Ausgangs
}

void Antrieb::begin(){
    // PinModes setzen, Encoder-ISR ggf. einrichten
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_encoderPinA, INPUT);
    pinMode(_encoderPinB, INPUT);
    //Interrupt aktivieren, um Quadraturencoder auslesen zu können
    attachInterrupt(digitalPinToInterrupt(_encoderPinA), encoderISR, CHANGE); //Pin A auszulesen ist ausreichend - Pin B zusätzlich wäre overkill
}

//Festelgen der Solldrehzahl
void Antrieb::setSpeed(float speed) {
    _setpoint = speed;
}

void Antrieb::update(int dt) {
    // 1) Encoder auslesen -> _currentSpeed
    _currentSpeed = readEncoderSpeed(dt);

    // 2) Regelung aufrufen - pidOutput wird angepasst
    _controller.Compute();

    // 3) Motoransteuerung mit dem Output
    applyOutput();
}

float Antrieb::getCurrentSpeed() const {
    return _currentSpeed;
}

float Antrieb::getSetpoint() const {
    return _setpoint;
}

void Antrieb::applyOutput() {
    // Output in einen PWM-Wert konvertieren
    analogWrite(_pwmPin, constrain(_pidOutput,0,255)); //Begrenze Output auf 0-255
}

//Auslesen Quadraturencoder - Berechnen Geschwindigekit
void Antrieb::encoderISR(){
  _encoderPos++;
}

float Antrieb::readEncoderSpeed(int dt) {
  int secondsPerIntervall=1000/dt; //??
  int calculatedRPS=(_encoderPos/_ppu)*secondsPerIntervall; //encoderPos[Pulse/Intervall], ppu[Pulse/Umdrehung], secondsPerIntervall[Intervalle/Sekunde]
  _encoderPos=0;
  return calculatedRPS;
}
