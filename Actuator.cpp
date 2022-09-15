#include "Actuator.h"

// alternate constructor
Actuator::Actuator(const ODrive & givenODrive, uint8_t givenAxis, usb_serial_class & usbSerial): myODrive(givenODrive), axis(givenAxis), myUSBSerial(usbSerial) {
    position = 0;
    velocity = 0;
    current = 0;
    torque = 0;
    error = 0;
}

void Actuator::setPosition(float pos) {
    if (myODrive.getControlMode(axis) == 3 && myODrive.getInputMode(axis) == 5) {
        myODrive.setPosition(axis, pos);
    } else {
        snprintf(sentData, sizeof(sentData), "Invalid command: the actuator is not in position control.\n");
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

void Actuator::startPositionControl() {
    myODrive.setControlMode(axis, 3);
    myODrive.setInputMode(axis, 5);
    snprintf(sentData, sizeof(sentData), "Started position control\n");
    myUSBSerial.print(sentData);
}

void Actuator::startVelocityControl() {
    myODrive.setControlMode(axis, 2);
    myODrive.setInputMode(axis, 1);
    snprintf(sentData, sizeof(sentData), "Started velocity control\n");
    myUSBSerial.print(sentData);
}

void Actuator::startTorqueControl() {
    myODrive.setControlMode(axis, 1);
    myODrive.setInputMode(axis, 1);
    snprintf(sentData, sizeof(sentData), "Started torque control\n");
    myUSBSerial.print(sentData);
}

float Actuator::getPosition() {
    position = myODrive.getPosition(axis);
    return position;
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

uint32_t Actuator::getError() {
    error = myODrive.getAxisError(axis);
    return error;
}