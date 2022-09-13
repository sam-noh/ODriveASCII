#ifndef Actuator_h
#define Actuator_h

#include "ODrive.h"

class Actuator {

  private:
    ODrive myODrive;        // ODrive object to call member functions
    uint8_t axis;           // index of the control axis (0 or 1) on the ODrive
    float position;         // position of the motor in encoder counts
    float velocity;         // velocity of the motor in encoder counts/s
    float current;          // currrent draw of the motor in amps
    bool calibrated;        // (ODrive.axis.motor.config.pre_calibrated && ODrive.axis.encoder.config.pre_calibrated)
    bool ready;             // true if calibrated and all configs set successfully, false if else

  public:
    //Alternate constructor with given parameters
    Actuator(ODrive & givenODrive, uint8_t givenAxis);

    //Destructor
    ~Actuator();             

    bool initActuator();

    float getPosition();              // returns the motor position
    float getVelocity();              // returns the motor velocity
    float getCurrent();               // returns the motor current draw
    bool isCalibrated() ;             // returns true if the motor and encoder are calibrated
    void setPosition(float pos);      // sends a position command to the ODrive
    void setVelocity(float vel);      // sends a velocity command to the ODrive
    void moveToHome() const;          // moves to the zero position
};

#endif