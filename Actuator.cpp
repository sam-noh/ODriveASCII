#include "Actuator.h"

// alternate constructor
Actuator::Actuator(const ODrive & givenODrive, uint8_t givenAxis, usb_serial_class & usbSerial): myODrive(givenODrive), axis(givenAxis), myUSBSerial(usbSerial) {
    pos_abs = 0;
    pos_rel = 0;
    pos_home = 0;
    pos_min = FLT_MIN;
    pos_max = FLT_MAX;
    velocity = 0;
    current = 0;
    torque = 0;
    homed = false;
    error = 0;
}

bool Actuator::enable() {
    return myODrive.runClosedLoopControl(axis);
}

bool Actuator::disable() {
    return myODrive.runIdle(axis);
}

void Actuator::setPosition(float pos) {
    if (myODrive.getCurrentState(axis) == 8) {
        if (myODrive.getControlMode(axis) == 3 && myODrive.getInputMode(axis) == 5) {
            if (pos > pos_min && pos < pos_max) {
                myODrive.setPosition(axis, pos + pos_home);
            } else {
                snprintf(sentData, sizeof(sentData), "Invalid command: the requested position is outside of the limits.\n");
                myUSBSerial.print(sentData);
            }
            
        } else {
            snprintf(sentData, sizeof(sentData), "Invalid command: the actuator is not in position control.\n");
            myUSBSerial.print(sentData);
        }
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command: the axis is not in closed-loop control.\n");
            myUSBSerial.print(sentData);
    }
    
}

void Actuator::setVelocity(float vel) {
    if (myODrive.getControlMode(axis) == 2 && myODrive.getInputMode(axis) == 1) {
        myODrive.setVelocity(axis, vel);
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command: the actuator is not in velocity control.\n");
        myUSBSerial.print(sentData);
    }
}

void Actuator::setTorque(float torque) {
    if (myODrive.getControlMode(axis) == 1 && myODrive.getInputMode(axis) == 1) {
        myODrive.setTorque(axis, torque);
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command: the actuator is not in torque control.\n");
        myUSBSerial.print(sentData);
    }
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