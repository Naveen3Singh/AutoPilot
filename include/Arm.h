#ifndef AUTOPILOT_ARM_H
#define AUTOPILOT_ARM_H

class Arm {
public:
    Arm();
    void write(int x, int y, int z);
    void demo();
};
#endif //AUTOPILOT_ARM_H

#define ARM_AXIS_X_PIN 8
#define ARM_AXIS_Y_PIN 9
#define ARM_AXIS_Z_PIN 10
#define ARM_HAND_PIN 11

#define BASE_DEFAULT_ANGLE 90
#define FIRST_DEFAULT_ANGLE 0
#define MIDDLE_DEFAULT_ANGLE 0
#define HAND_DEFAULT_ANGLE 0

