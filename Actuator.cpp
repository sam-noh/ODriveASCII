#include "Actuator.h"

//Alternate constructor with given parameters
Actuator::Actuator(ODrive & givenODrive, uint8_t givenAxis): myODrive(givenODrive), axis(givenAxis) {
    position = 0;
    velocity = 0;
    current = 0;
    calibrated = Actuator::isCalibrated();
    ready = false;
}

//Destructor
Actuator::~Actuator() {}

bool Actuator::initActuator() {
    if (!calibrated && !myODrive.runFullCalibration(axis)){
        return false;   // not calibrated and couldn't calibrate
    }
    myODrive.setControlMode(axis, 3);       // position control
    myODrive.setInputMode(axis, 5);         // trapezoidal trajectory control

    ready = true;
    return ready;
}

// returns the motor position
float Actuator::getPosition() {

    position = myODrive.getPosition(axis);
    return position;
}

// returns the motor velocity
float Actuator::getVelocity() {
    velocity = myODrive.getVelocity(axis);
    return velocity;
}

// returns the motor current draw
float Actuator::getCurrent() {
    current = myODrive.getCurrent(axis);
    return current;
}

// returns the motor/encoder calibration status
bool Actuator::isCalibrated() {
    calibrated = myODrive.isCalibrated(axis);
    return calibrated;
}


void Actuator::setPosition(float pos) {
    myODrive.setPosition(axis, pos);
}

void Actuator::setVelocity(float vel) {
    myODrive.setVelocity(axis, vel);
}

// moves to the zero position
void Actuator::moveToHome() const {

}