#include "ODrive.h"

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// alternate constructor
ODrive::ODrive(usb_serial_class & usbSerial, HardwareSerial & serial, uint32_t baud): myUSBSerial(usbSerial), mySerial(serial) {
    // USB serial begin is run once in the main code, not here
    mySerial.begin(baud);
}

ODrive::~ODrive() {}

float ODrive::readFloat() {
    return readString().toFloat();
}

int32_t ODrive::readInt() {
    return readString().toInt();
}

float ODrive::getPosition(uint8_t axis) {     // returns the motor position
    mySerial << "r axis" << axis << ".encoder.pos_estimate\n";
    return ODrive::readFloat();
}

float ODrive::getVelocity(uint8_t axis) {     // returns the motor velocity
    mySerial << "r axis" << axis << ".encoder.vel_estimate\n";
    return ODrive::readFloat();
}

float ODrive::getCurrent(uint8_t axis) {      // returns the motor current draw
    mySerial << "r axis" << axis << ".motor.current_control.Iq_measured\n";
    return ODrive::readFloat();
}

uint8_t ODrive::getCurrentState(uint8_t axis) { // returns the current axis state
    mySerial << "r axis" << axis << ".current_state\n";
    return ODrive::readInt();
}

bool ODrive::isCalibrated(uint8_t axis) { // returns true if the motor and encoder are calibrated
    snprintf(sentData, sizeof(sentData), "Checking axis %d calibration...\n", axis+1);
    myUSBSerial.print(sentData);
    mySerial << "r axis" << axis << ".motor.is_calibrated\n";
    int motorCalibration = ODrive::readInt();

    if (motorCalibration) {
        snprintf(sentData, sizeof(sentData), "Motor is calibrated.\n");
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Motor is not calibrated.\n");
        myUSBSerial.print(sentData);
    }

    mySerial << "r axis" << axis << ".encoder.is_ready\n";
    int encoderCalibration = ODrive::readInt();

    if (encoderCalibration) {
        snprintf(sentData, sizeof(sentData), "Encoder is calibrated.\n\n");
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Encoder is not calibrated.\n\n");
        myUSBSerial.print(sentData);
    }
    return motorCalibration && encoderCalibration;
}

uint32_t ODrive::getAxisError(uint8_t axis) { // returns the axis error if any
    mySerial << "r axis" << axis << ".error\n";
    return ODrive::readInt();
}

void ODrive::clearErrors() {                     // clears any ODrive errors
    mySerial << "w axis" << 0 << "error " << 0 << '\n';
    mySerial << "w axis" << 1 << "error " << 0 << '\n';
}

bool ODrive::runState(uint8_t axis, uint8_t requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    mySerial << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            mySerial << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

bool ODrive::runIdle(uint8_t axis) {
    bool success =  ODrive::runState(axis, 1, false, 3.0f);

    if (success) {
        snprintf(sentData, sizeof(sentData), "Axis %d in idle mode.\n", axis+1);
        myUSBSerial.print(sentData);    
    } else {
        snprintf(sentData, sizeof(sentData), "Axis %d failed to enter idle mode.\n", axis+1);
        myUSBSerial.print(sentData);    
    }
    return success;
}

bool ODrive::runFullCalibration(uint8_t axis) {
    return ODrive::runState(axis, 3, false, 3.0f);
}

bool ODrive::runClosedLoopControl(uint8_t axis) {
    bool success = ODrive::runState(axis, 8, false, 3.0f);
    
    if (success) {
        snprintf(sentData, sizeof(sentData), "Axis %d in closed-loop control.\n", axis+1);
        myUSBSerial.print(sentData);    
    } else {
        snprintf(sentData, sizeof(sentData), "Axis %d failed to enter closed-loop control.\n", axis+1);
        myUSBSerial.print(sentData);    
    }
    return success;
}

bool ODrive::runHoming(uint8_t axis) {
    return ODrive::runState(axis, 11, false, 3.0f);
}

uint8_t ODrive::getControlMode(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.control_mode\n";
    return ODrive::readInt();
}

uint8_t ODrive::getInputMode(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.input_mode\n";
    return ODrive::readInt();
}

void ODrive::setControlMode(uint8_t axis, uint8_t mode) {
    mySerial << "w axis" << axis << ".controller.config.control_mode " << mode << '\n';
}

void ODrive::setInputMode(uint8_t axis, uint8_t mode) {
    mySerial << "w axis" << axis << ".controller.config.input_mode " << mode << '\n';
}

void ODrive::setCurrentLim(uint8_t axis, float current) {
    mySerial << "w axis" << axis << ".motor.config.current_lim " << current << '\n';
}

void ODrive::setVelLim(uint8_t axis, float vel) {
    mySerial << "w axis" << axis << ".controller.config.vel_lim " << vel << '\n';
}

void ODrive::setPosGain(uint8_t axis, float gain) {
    mySerial << "w axis" << axis << ".controller.config.pos_gain " << gain << '\n';
}

void ODrive::setVelGain(uint8_t axis, float gain) {
    mySerial << "w axis" << axis << ".controller.config.vel_gain " << gain << '\n';
}

void ODrive::setVelIntGain(uint8_t axis, float gain) {
    mySerial << "w axis" << axis << ".controller.config.vel_integrator_gain " << gain << '\n';
}

void ODrive::setPosition(uint8_t axis, float pos) const {       // sends a position command to the ODrive
    mySerial << "q " << axis << " " << pos << "\n";
}

void ODrive::setVelocity(uint8_t axis, float vel) const {  // sends a velocity command to the ODrive
    mySerial << "v " << axis << " " << vel << "\n";
}

String ODrive::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!mySerial.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = mySerial.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}