#ifndef AUTOPILOT_AUTOPILOT_H
#define AUTOPILOT_AUTOPILOT_H

#include "Arduino.h"

#define MOTOR_PWM_PIN 0
#define MOTOR_IN1_PIN 4
#define MOTOR_IN2_PIN 5

#define STEERING_PIN 2
#define STEERING_MIN 30   // High left angle because of hardware issue
#define STEERING_DEFAULT_ANGLE 90
#define STEERING_MAX 170

#define OBSTACLE_AVOIDANCE_FRONT_SENSOR_PIN 12
#define OBSTACLE_AVOIDANCE_REAR_SENSOR_PIN 14

class AutoPilot {
public:
    AutoPilot();
    void init();
    void update();
    static void forward(int speed);
    void forwardFor(unsigned long delay, int speed);
    static void backward(int speed);
    void backwardFor(unsigned long delay, int speed);
    static void turn(int angle);
    static void turnLeft();
    static void turnRight();
    static void brake();
    void demo();
    void runInCircle();

    void release();

    void runStraight();

    void applyPilotUpdates(char *buffer, int packetSize);

private:
    volatile unsigned long stopAtMs;
    unsigned short speed;
    int direction;
    bool left;
    bool right;
    int turnAngle;
    bool stop;

    ICACHE_RAM_ATTR static void frontObstacleAvoidanceISR();
    ICACHE_RAM_ATTR static void rearObstacleAvoidanceISR();
};

#endif //AUTOPILOT_AUTOPILOT_H
