#include "Arduino.h"
#include <PowerTrain.h>

PowerTrain::PowerTrain(int pinIn1, int pinIn2) {
    _pinPwm = -1;
    _pinIn1 = pinIn1;
    _pinIn2 = pinIn2;
    _pwmVal = 0;
    _isMoving = false;
    _direction = FORWARD;

    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
}

PowerTrain::PowerTrain(int pinPwm, int pinIn1, int pinIn2) {
    _pinPwm = pinPwm;
    _pinIn1 = pinIn1;
    _pinIn2 = pinIn2;
    _pwmVal = 0;
    _isMoving = false;
    _direction = FORWARD;

    pinMode(_pinPwm, OUTPUT);
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
}

void PowerTrain::setSpeed(unsigned short pwmVal) {
    unsigned short value = pwmVal;
    if (pwmVal > MAX_SPEED) {
        value = MAX_SPEED;
    }
    if (isMoving() && _pwmVal != value) {
        analogWrite(_pinPwm, value);
    }
    _pwmVal = value;
}

/**
 * Move vehicle forward.
 */
void PowerTrain::forward() {
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, LOW);
    if (_pinPwm != -1) {
        for (int value = 255; value < MAX_SPEED; value = value + 5) {
            analogWrite(_pinPwm, value);
            delay(100);
        }
    }
    _direction = FORWARD;
    _isMoving = true;
}

/**
 * Move vehicle backward.
 */
void PowerTrain::backward() {
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, HIGH);

    if (_pinPwm != -1) {
        for (int value = 255; value < MAX_SPEED; value = value + 5) {
            analogWrite(_pinPwm, value);
            delay(100);
        }
    }

    _direction = BACKWARD;
    _isMoving = true;
}

bool PowerTrain::isMoving() {
    return _isMoving;
}

int PowerTrain::getDirection() {
    return _direction;
}

/**
 * Stop the vehicle.
 */
void PowerTrain::stop() {
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
    if (_pinPwm != -1) {
        analogWrite(_pinPwm, 2000);
    }

    _isMoving = false;
}

void PowerTrain::release() {
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, LOW);
    if (_pinPwm != -1) {
        digitalWrite(_pinPwm, 0);
    }
    _isMoving = false;
}
