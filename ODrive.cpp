#include "ODrive.h"

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// alternate constructor
ODrive::ODrive(HardwareSerial & serial, uint32_t baud, usb_serial_class & usbSerial): mySerial(serial), myUSBSerial(usbSerial) {
    mySerial.begin(baud);   // usb_serial_class.begin() is run once in the main code, not here
}

// helper functions
String ODrive::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    char c;
    for (;;) {
        while (!mySerial.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        c = mySerial.read();
        if (c == '\n') {
            break;
        }
        str += c;
    }
    return str;
}

float ODrive::readFloat() {
    return readString().toFloat();
}

int32_t ODrive::readInt() {
    return readString().toInt();
}

// system behaviors
bool ODrive::ready() {              // checks for calibration of both axes and enters closed-loop control
    return ODrive::isCalibrated(0) & ODrive::isCalibrated(1);
    // if (ODrive::isCalibrated(0) & ODrive::isCalibrated(1)) {
    //     return ODrive::runClosedLoopControl(0) & ODrive::runClosedLoopControl(1);
    // } else {
    //     return false;
    // }
}

bool ODrive::stop() {               // enters idle state on both axes
    return ODrive::runIdle(0) & ODrive::runIdle(1);
}

void ODrive::reboot() {             // reboots the ODrive
    mySerial << "sr\n";
}

// system read functions
float ODrive::getBusVoltage() {     // returns the bus voltage of the ODrive
    mySerial << "r vbus_voltage\n";
    return ODrive::readFloat();
}

float ODrive::getBusCurrent() {     // returns the bus current of the ODrive
    mySerial << "r ibus\n";
    return ODrive::readFloat();
}

// motor read functions
float ODrive::getPosition(uint8_t axis) {       // returns the motor position
    mySerial << "r axis" << axis << ".encoder.pos_estimate\n";
    return ODrive::readFloat();
}

float ODrive::getVelocity(uint8_t axis) {       // returns the motor velocity
    mySerial << "r axis" << axis << ".encoder.vel_estimate\n";
    return ODrive::readFloat();
}

float ODrive::getTorque(uint8_t axis) {         // returns the motor torque
    mySerial << "r axis" << axis << ".motor.current_control.Iq_measured\n";
    float current = ODrive::readFloat();

    mySerial << "r axis" << axis << ".motor.config.torque_constant\n";
    float torque_constant = ODrive::readFloat();

    return current*torque_constant;
}

float ODrive::getCurrent(uint8_t axis) {        // returns the motor current draw
    mySerial << "r axis" << axis << ".motor.current_control.Iq_measured\n";
    return ODrive::readFloat();
}

// motor command functions
void ODrive::setPosition(uint8_t axis, float pos) const {       // sends a position command to the ODrive
    mySerial << "q " << axis << " " << pos << "\n";
}

void ODrive::setVelocity(uint8_t axis, float vel) const {       // sends a velocity command to the ODrive
    mySerial << "v " << axis << " " << vel << "\n";
}

void ODrive::setTorque(uint8_t axis, float torque) const {      // sends a torque command to the ODrive
    mySerial << "c " << axis << " " << torque << "\n";
}

// state/controller read functions
uint8_t ODrive::getCurrentState(uint8_t axis) { // returns the current axis state
    mySerial << "r axis" << axis << ".current_state\n";
    return ODrive::readInt();
}

uint32_t ODrive::getAxisError(uint8_t axis) { // returns the axis error if any
    mySerial << "r axis" << axis << ".error\n";
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

    delay(50);  // seems to be necessary to not overload serial prints and comms

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

uint8_t ODrive::getControlMode(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.control_mode\n";
    return ODrive::readInt();
}

uint8_t ODrive::getInputMode(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.input_mode\n";
    return ODrive::readInt();
}

float ODrive::getCurrentLim(uint8_t axis) {
    mySerial << "r axis" << axis << ".motor.config.current_lim\n";
    return ODrive::readFloat();
}

float ODrive::getVelLim(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.vel_limit\n";
    return ODrive::readFloat();
}

float ODrive::getPosGain(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.pos_gain\n";
    return ODrive::readFloat();
}

float ODrive::getVelGain(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.vel_gain\n";
    return ODrive::readFloat();
}

float ODrive::getVelIntGain(uint8_t axis) {
    mySerial << "r axis" << axis << ".controller.config.vel_integrator_gain\n";
    return ODrive::readFloat();
}

float ODrive::getTrapAccelLim(uint8_t axis) {
    mySerial << "r axis" << axis << ".trap_traj.config.accel_limit\n";
    return ODrive::readFloat();
}

float ODrive::getTrapDecelLim(uint8_t axis) {
    mySerial << "r axis" << axis << ".trap_traj.config.decel_limit\n";
    return ODrive::readFloat();
}

float ODrive::getTrapVelLim(uint8_t axis) {
    mySerial << "r axis" << axis << ".trap_traj.config.vel_limit\n";
    return ODrive::readFloat();
}

// state/controller write functions
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
    bool success =  ODrive::runState(axis, 1, true, 10.0f);

    if (success) {
        snprintf(sentData, sizeof(sentData), "Axis %d in idle state.\n", axis+1);
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Axis %d failed to enter idle state.\n", axis+1);
        myUSBSerial.print(sentData);
    }
    return success;
}

bool ODrive::runFullCalibration(uint8_t axis) {
    bool success = ODrive::runState(axis, 3, true, 10.0f);
    
    if (success) {
        snprintf(sentData, sizeof(sentData), "Axis %d calibrated successfully.\n", axis+1);
        myUSBSerial.print(sentData);

        mySerial << "w axis" << axis << ".motor.config.pre_calibrated " << 1 << '\n';
        mySerial << "w axis" << axis << ".encoder.config.pre_calibrated " << 1 << '\n';

    } else {
        snprintf(sentData, sizeof(sentData), "Axis %d failed to calibrate.\n", axis+1);
        myUSBSerial.print(sentData);
    }
    return success;
}

bool ODrive::runClosedLoopControl(uint8_t axis) {
    bool success = ODrive::runState(axis, 8, false);
    
    if (success) {
        snprintf(sentData, sizeof(sentData), "Axis %d in closed-loop control.\n", axis+1);
        myUSBSerial.print(sentData);
    } else {
        snprintf(sentData, sizeof(sentData), "Axis %d failed to enter closed-loop control.\n", axis+1);
        myUSBSerial.print(sentData);
    }
    return success;
}

bool ODrive::runHoming(uint8_t axis, float homingVelocity, float homingOffset) {
    // return ODrive::runState(axis, 11, true, 30.0f);      // ODrive's homing feature requires limit sensors
    snprintf(sentData, sizeof(sentData), "Homing axis %d...\n", axis+1);
    myUSBSerial.print(sentData);

    ODrive::switchToVelocityControl(axis);  // switch to velocity control mode for homing

    if (ODrive::getCurrentState(axis) != 8) {
        ODrive::runClosedLoopControl(axis);
    }

    if (ODrive::getCurrentState(axis) == 8) {
        ODrive::setVelocity(axis, homingVelocity);      // move slowly towards the joint limit
        snprintf(sentData, sizeof(sentData), "Moving to joint limit...\n\n");
        myUSBSerial.print(sentData);
        delay(300);                                     // don't check endstop condition when the motor starts
        while (true) {
            if (abs(ODrive::getVelocity(axis)) < 0.05) { // if the motor has slowed due to the joint limit
                myUSBSerial.print("At joint limit.\n");
                ODrive::setVelocity(axis, 0);           // stop the motor
                ODrive::switchToPositionControl(axis);

                myUSBSerial.print("Moving to homing offset...\n\n");
                snprintf(sentData, sizeof(sentData), "Currently at position: %f\n", ODrive::getPosition(axis));
                myUSBSerial.print(sentData);
                float newPos = ODrive::getPosition(axis) + homingOffset;
                snprintf(sentData, sizeof(sentData), "Moving to: %f\n", newPos);
                myUSBSerial.print(sentData);

                ODrive::setPosition(axis, newPos);    // move to the homing offset position
                delay(200);
                while (abs(ODrive::getPosition(axis) - newPos) > 0.15) {    // wait for the motor to move to the offset
                    delay(10);
                }
                ODrive::runIdle(axis);  // enter idle mode after homing

                snprintf(sentData, sizeof(sentData), "Axis %d successfully homed.\n\n", axis+1);
                myUSBSerial.print(sentData);
                return true;
            }
            delay(10);  // querying too often can miss the data
        }
    } else{
        return false;
    }
    


}

void ODrive::clearErrors() {                     // clears any ODrive errors
    mySerial << "w axis" << 0 << "error " << 0 << '\n';
    mySerial << "w axis" << 1 << "error " << 0 << '\n';
}

void ODrive::switchToPositionControl(uint8_t axis) {
    ODrive::setControlMode(axis, 3);
    ODrive::setInputMode(axis, 5);
    snprintf(sentData, sizeof(sentData), "Control mode set to position control.\nInput mode set to trapezoidal trajectory.\n\n");
    myUSBSerial.print(sentData);
}

void ODrive::switchToVelocityControl(uint8_t axis) {
    ODrive::setControlMode(axis, 2);
    ODrive::setInputMode(axis, 1);
    snprintf(sentData, sizeof(sentData), "Control mode set to velocity control.\nInput mode set to passthrough.\n\n");
    myUSBSerial.print(sentData);
}

void ODrive::switchToTorqueControl(uint8_t axis) {
    ODrive::setControlMode(axis, 1);
    ODrive::setInputMode(axis, 1);
    snprintf(sentData, sizeof(sentData), "Control mode set to torque control.\nInput mode set to passthrough.\n\n");
    myUSBSerial.print(sentData);
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
    mySerial << "w axis" << axis << ".controller.config.vel_limit " << vel << '\n';
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

void ODrive::setTrapAccelLim(uint8_t axis, float accel) {
    mySerial << "w axis" << axis << ".trap_traj.config.accel_limit " << accel << '\n';
}

void ODrive::setTrapDecelLim(uint8_t axis, float decel) {
    mySerial << "w axis" << axis << ".trap_traj.config.decel_limit " << decel << '\n';
}

void ODrive::setTrapVelLim(uint8_t axis, float vel) {
    mySerial << "w axis" << axis << ".trap_traj.config.vel_limit " << vel << '\n';
}