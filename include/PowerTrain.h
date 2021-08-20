#ifndef AUTOPILOT_POWERTRAIN_H
#define AUTOPILOT_POWERTRAIN_H

#define FORWARD 1
#define BACKWARD -1
#define MAX_SPEED 1000

class PowerTrain {
public:
    PowerTrain(int pinIn1, int pinIn2);
    PowerTrain(int pinPwm, int pinIn1, int pinIn2);
    void setSpeed(unsigned short pwmVal);
    void forward();
    void backward();
    bool isMoving();
    int getDirection();
    void stop();
    void release();

private:
    int _pinPwm;
    int _pinIn1;
    int _pinIn2;
    short unsigned _pwmVal;
    bool _isMoving;
    int _direction;

};
#endif //AUTOPILOT_POWERTRAIN_H
