#include "Servo.h"
#include "Arm.h"

Servo base;
Servo first;
Servo middle;
Servo hand;

Arm::Arm() {
    // Setup pins
    pinMode(ARM_AXIS_X_PIN, OUTPUT);
    pinMode(ARM_AXIS_Y_PIN, OUTPUT);
    pinMode(ARM_AXIS_Z_PIN, OUTPUT);
    pinMode(ARM_HAND_PIN, OUTPUT);

    // Attach servos
    base.attach(ARM_AXIS_X_PIN);
    first.attach(ARM_AXIS_Y_PIN);
    middle.attach(ARM_AXIS_Z_PIN);
    hand.attach(ARM_HAND_PIN);

    // Set default values i.e. closed arm
    base.write(BASE_DEFAULT_ANGLE);
    first.write(FIRST_DEFAULT_ANGLE);
    middle.write(MIDDLE_DEFAULT_ANGLE);
    hand.write(HAND_DEFAULT_ANGLE);
}

void Arm::demo() {
    write(random(120), random(120), random(120));
}

void Arm::write(int x, int y, int z) {
    base.write(x);
    first.write(y);
    middle.write(z);
}