#ifndef Actuator_h
#define Actuator_h

#include "ODrive.h"
#include "float.h"

class Actuator {

  private:
    ODrive myODrive;                          // ODrive object to call member functions
    uint8_t axis;                             // index of the control axis (0 or 1) on the ODrive
    usb_serial_class & myUSBSerial = Serial;  // Serial object for printing debug info
    char sentData[128] = "";

    // the ODrive only knows pos_abs
    // all position commands are converted to pos_abs by subtracting pos_home (when not homed, pos_home = 0)
    // if pos_home is -3 and the position command is 50, the actuator will move to pos_abs = 53
    float pos_abs;          // motor position reported by the ODrive after powerup (turns)
    float pos_rel;          // pos_rel = pos_abs - pos_home (turns)
    float pos_home;         // pos_abs at home; call setHomePosition()
    float pos_min;          // minimum allowed pos_rel
    float pos_max;          // maximum allowed pos_rel
    float velocity;         // motor velocity (turns/s)
    float current;          // motor current draw (amps)
    float torque;           // motor torque (Nm)

    bool homed;             // false initially; calling setHomePosition() sets it to true
    uint32_t error;         // ODrive error

  public:
    // alternate constructor
    Actuator(const ODrive & givenODrive, uint8_t givenAxis, usb_serial_class & usbSerial);

    bool enable();                // enters closed-loop control
    bool disable();               // enters idle state

    // actuator control functions
    void setPosition(float pos);  // takes pos_rel, converts to pos_abs, and sends pos_abs as the position command to the ODrive
    void setVelocity(float vel);  // sends a velocity command to the ODrive
    void setTorque(float torque); // sends a torque command to the ODrive

    void startPositionControl();  // sets control and input modes for position control
    void startVelocityControl();  // sets control and input modes for velocity control
    void startTorqueControl();    // sets control and input modes for torque control

    void setHome();               // sets the current pos_abs as pos_home
    void setMinPos(float pos);    // sets pos_min
    void setMaxPos(float pos);    // sets pos_max

    // actuator read functions
    float getPosition();          // queries pos_abs from ODrive, computes pos_rel, and returns pos_rel
    float getVelocity();          // returns the motor velocity
    float getTorque();            // returns the motor torque
    float getCurrent();           // returns the motor current draw

    float getMinPos();            // returns pos_min
    float getMaxPos();            // returns pos_max

    // actuator system functions
    uint32_t getError();          // returns the axis error if any
};

#endif