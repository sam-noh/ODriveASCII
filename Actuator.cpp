#include "Actuator.h"

// alternate constructor
Actuator::Actuator(const ODrive & givenODrive, uint8_t givenAxis, usb_serial_class & usbSerial): myODrive(givenODrive), axis(givenAxis), myUSBSerial(usbSerial) {
    pos_abs = 0;
    pos_rel = 0;
    pos_home = 0;
    pos_min = -1;
    pos_max = 150;
    velocity = 0;
    current = 0;
    torque = 0;
    homed = false;
    error = 0;
}

bool Actuator::enable() {
    if (myODrive.getCurrentState(axis) != 8) {      // from Discord #communication: only request states to change
        return myODrive.runClosedLoopControl(axis);
    } else {
        return true;
    }
}

bool Actuator::disable() {
    if (myODrive.getCurrentState(axis) != 1) {
        return myODrive.runIdle(axis);
    } else {
        return true;
    }
    
}

void Actuator::setPosition(float pos) {
    myODrive.setPosition(axis, pos + pos_home);
}

void Actuator::setVelocity(float vel) {
    myODrive.setVelocity(axis, vel);
}

void Actuator::setTorque(float torque) {
    myODrive.setTorque(axis, torque);
}

void Actuator::switchToPositionControl() {
    myODrive.switchToPositionControl(axis);
}

void Actuator::switchToVelocityControl() {
    myODrive.switchToVelocityControl(axis);
}

void Actuator::switchToTorqueControl() {
    myODrive.switchToTorqueControl(axis);
}

void Actuator::setHome() {
    pos_home = myODrive.getPosition(axis);
    Actuator::getPosition();
    homed = true;
    snprintf(sentData, sizeof(sentData), "Home position set.\n");
    myUSBSerial.print(sentData);
}

void Actuator::setMinPos(float pos) {
    pos_min = pos;
    snprintf(sentData, sizeof(sentData), "Minimum position set.\n");
    myUSBSerial.print(sentData);
}

void Actuator::setMaxPos(float pos) {
    pos_max = pos;
    snprintf(sentData, sizeof(sentData), "Maximum position set.\n");
    myUSBSerial.print(sentData);
}

float Actuator::getPosition() {
    pos_abs = myODrive.getPosition(axis);
    pos_rel = pos_abs - pos_home;
    return pos_rel;
}

float Actuator::getVelocity() {
    velocity = myODrive.getVelocity(axis);
    return velocity;
}

float Actuator::getTorque() {
    torque = myODrive.getTorque(axis);
    return torque;
}

float Actuator::getCurrent() {
    current = myODrive.getCurrent(axis);
    return current;
}

float Actuator::getMinPos() {
    return pos_min;
}

float Actuator::getMaxPos() {
    return pos_max;
}

uint32_t Actuator::getError() {
    error = myODrive.getAxisError(axis);
    return error;
}