#ifndef Actuator_h
#define Actuator_h

#include "ODrive.h"

class Actuator {

  private:
    ODrive myODrive;        // ODrive object to call member functions
    uint8_t axis;           // index of the control axis (0 or 1) on the ODrive
    usb_serial_class & myUSBSerial = Serial;
    char sentData[128] = "";

    float position;         // motor position (turns)
    float velocity;         // motor velocity (turns/s)
    float current;          // motor current draw (amps)
    float torque;           // motor torque (Nm)

    uint32_t error;         // ODrive error

  public:
    // alternate constructor
    Actuator(const ODrive & givenODrive, uint8_t givenAxis, usb_serial_class & usbSerial);

    // actuator control functions
    void setPosition(float pos);  // sends a position command to the ODrive
    void setVelocity(float vel);  // sends a velocity command to the ODrive
    void setTorque(float torque); // sends a torque command to the ODrive

    void startPositionControl();  // sets control and input modes for position control
    void startVelocityControl();  // sets control and input modes for velocity control
    void startTorqueControl();    // sets control and input modes for torque control

    // actuator read functions
    float getPosition();          // returns the motor position
    float getVelocity();          // returns the motor velocity
    float getTorque();            // returns the motor torque
    float getCurrent();           // returns the motor current draw

    // actuator system functions
    uint32_t getError();          // returns the axis error if any
};

#endif