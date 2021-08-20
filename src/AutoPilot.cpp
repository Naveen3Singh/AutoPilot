#include <PowerTrain.h>
#include <Pilot.h>
#include "Arduino.h"
#include "Servo.h"
#include "AutoPilot.h"
#include "ArduinoJson.h"

Servo steering;
PowerTrain powerTrain(MOTOR_IN1_PIN, MOTOR_IN2_PIN);

/**
 * Autonomous Driving.
 */
AutoPilot::AutoPilot() {
    stopAtMs = 0;
    speed = 0;
    direction = FORWARD;
    left = false;
    right = false;
    turnAngle = STEERING_DEFAULT_ANGLE;
    stop = false;
}

void AutoPilot::init() {
    // Steering
    pinMode(STEERING_PIN, OUTPUT);
    steering.attach(STEERING_PIN);
    steering.write(STEERING_MAX);
    delay(1500);
    steering.write(STEERING_MIN);
    delay(1500);
    steering.write(STEERING_DEFAULT_ANGLE);

    // Obstacle avoidance interrupt
    pinMode(OBSTACLE_AVOIDANCE_FRONT_SENSOR_PIN, INPUT);
    pinMode(OBSTACLE_AVOIDANCE_REAR_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(OBSTACLE_AVOIDANCE_FRONT_SENSOR_PIN),
                    frontObstacleAvoidanceISR,
                    FALLING);
    attachInterrupt(digitalPinToInterrupt(OBSTACLE_AVOIDANCE_REAR_SENSOR_PIN),
                    rearObstacleAvoidanceISR,
                    FALLING);
}

/**
 * Update internal state.
 */
void AutoPilot::update() {
    if (stopAtMs != 0 && millis() >= stopAtMs && powerTrain.isMoving()) {
        brake();
        stopAtMs = 0;
    }
}

/**
 * Move vehicle forward.
 * @param speed
 */
void AutoPilot::forward(int speed) {
    powerTrain.setSpeed(speed);
    powerTrain.forward();
}

/**
 * Move vehicle forward for given interval.
 * @param delay
 * @param speed
 */
void AutoPilot::forwardFor(unsigned long delay, int speed) {
    stopAtMs = millis() + delay;
    forward(speed);
}

/**
 * Move vehicle backward.
 * @param speed
 */
void AutoPilot::backward(int speed) {
    powerTrain.setSpeed(speed);
    powerTrain.backward();
}

/**
 * Move vehicle backward for given interval.
 * @param delay
 * @param speed
 */
void AutoPilot::backwardFor(unsigned long delay, int speed) {
    stopAtMs = millis() + delay;
    backward(speed);
}

/**
 * Turn steering right or left.
 * @param angle
 */
void AutoPilot::turn(int angle) {
    int servoMotorAngle = angle + STEERING_DEFAULT_ANGLE;
    // Check invalid value
    if (servoMotorAngle < STEERING_MIN) {
        servoMotorAngle = STEERING_MIN;
    } else if (servoMotorAngle > STEERING_MAX) {
        servoMotorAngle = STEERING_MAX;
    } else if (servoMotorAngle == 0) {
        servoMotorAngle = STEERING_DEFAULT_ANGLE;
    }
    steering.write(servoMotorAngle);
}

/**
 * Turn steering left.
 */
void AutoPilot::turnLeft() {
    turn(-90);
}

/**
 * Turn steering right.
 */
void AutoPilot::turnRight() {
    turn(90);
}

/**
 * Stop the vehicle.
 */

void AutoPilot::brake() {
    powerTrain.stop();
}


/**
 * Release the vehicle.
 */

void AutoPilot::release() {
    powerTrain.release();
    stopAtMs = 0;
}

/**
 * Demonstrate controls.
 */
void AutoPilot::demo() {
    turnLeft();
    delay(2000);
    turnRight();
    delay(2000);
    forward(MAX_SPEED);
    delay(500);
    powerTrain.stop();
    delay(1000);
    turnRight();
    backward(MAX_SPEED);
    delay(500);
    powerTrain.stop();
    delay(2000);
    turn(0);
}

/**
 * Run in circle.
 */
void AutoPilot::runInCircle() {
    turnLeft();
    delay(2000);
    turnRight();
    delay(2000);
    forward(MAX_SPEED);
    delay(10000);
}

/**
 * Run in circle.
 */
void AutoPilot::runStraight() {
    forwardFor(10000, MAX_SPEED);
}

/**
 * Interrupt Service Routine to avoid any possible crash collisions
 * with obstacle in front of the vehicle.
 */
void AutoPilot::frontObstacleAvoidanceISR() {
    Serial.println("Obstacle detected!!");
    while (digitalRead(OBSTACLE_AVOIDANCE_FRONT_SENSOR_PIN) == LOW) {
        AutoPilot::backward(3000);
    }
    AutoPilot::brake();
}

/**
 * Interrupt Service Routine to avoid any possible crash collisions
 * with obstacle in back of the vehicle.
 */
void AutoPilot::rearObstacleAvoidanceISR() {
    Serial.println("Obstacle detected!!");
    while (digitalRead(OBSTACLE_AVOIDANCE_REAR_SENSOR_PIN) == LOW) {
        AutoPilot::forward(3000);
    }
    AutoPilot::brake();
}


/**
 * Apply Pilot settings from a jsonObject.
 * @param jsonDocument
 */
void AutoPilot::applyPilotUpdates(char *buffer, int packetSize) {
    StaticJsonDocument<200> jsonDocument;
    DeserializationError error = deserializeJson(jsonDocument, buffer, packetSize);

    Serial.println(error.c_str());
    if (jsonDocument.containsKey(SPEED)) {
        speed = jsonDocument[SPEED];
    }
    if (jsonDocument.containsKey(DIRECTION)) {
        direction = jsonDocument[DIRECTION];
        Serial.printf("Applying direction %d with speed: %d", direction, speed);
        if (jsonDocument[DIRECTION] == FORWARD) {
            forward(speed);
        } else {
            backward(speed);
        }
    }
    if (jsonDocument.containsKey(TURN_LEFT)) {
        left = jsonDocument[TURN_LEFT];
        if (jsonDocument[TURN_LEFT] == 1) {
            turnLeft();
            Serial.println("Turning left");
        } else {
            turn(STEERING_DEFAULT_ANGLE);
        }
    }
    if (jsonDocument.containsKey(TURN_RIGHT)) {
        right = jsonDocument[TURN_RIGHT];
        if (jsonDocument[TURN_RIGHT] == 1) {
            turnRight();
            Serial.println("Turning right");
        } else {
            turn(STEERING_DEFAULT_ANGLE);
        }
    }
    if (jsonDocument.containsKey(TURN)) {
        turnAngle = jsonDocument[TURN];
        turn(jsonDocument[TURN]);
        Serial.printf("Turning by %d", int(jsonDocument[TURN]));
    }
    if (jsonDocument.containsKey(BRAKE)) {
        stop = jsonDocument[BRAKE];
        if (jsonDocument[BRAKE] == 1) brake();
    }

    // Follow pre-programmed paths
    if (jsonDocument.containsKey(DEMO)) {
        if (jsonDocument[DEMO] == 1) demo();
    }
    if (jsonDocument.containsKey(RUN_STRAIGHT)) {
        if (jsonDocument[RUN_STRAIGHT] == 1) runStraight();
    }
    if (jsonDocument.containsKey(RUN_IN_CIRCLE)) {
        if (jsonDocument[RUN_IN_CIRCLE] == 1) runInCircle();
    }
}
