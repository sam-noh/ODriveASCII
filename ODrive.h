#ifndef ODrive_h
#define ODrive_h

#include "Arduino.h"
#include "ODriveEnums.h"

class ODrive {

    private:
        usb_serial_class & myUSBSerial;
        HardwareSerial & mySerial;
        char sentData[128] = "";

        String readString();

    public:

        ODrive(usb_serial_class & usbSerial, HardwareSerial & serial, uint32_t baud);

        ~ODrive();

        float readFloat();
        int32_t readInt();

        float getPosition(uint8_t axis);        // returns the motor position
        float getVelocity(uint8_t axis);        // returns the motor velocity
        float getCurrent(uint8_t axis);         // returns the motor current draw
        uint8_t getCurrentState(uint8_t axis);  // returns the current axis state
        
        bool isCalibrated(uint8_t axis) ;       // returns true if the motor and encoder are calibrated
        uint32_t getAxisError(uint8_t axis);    // returns the axis error if any
        void clearErrors();                     // clears errors if any

        bool runState(uint8_t axis, uint8_t requested_state, bool wait_for_idle, float timeout = 10.0f);
        bool runIdle(uint8_t axis);
        bool runFullCalibration(uint8_t axis);
        bool runClosedLoopControl(uint8_t axis);
        bool runHoming(uint8_t axis);

        uint8_t getControlMode(uint8_t axis);
        uint8_t getInputMode(uint8_t axis);

        void setControlMode(uint8_t axis, uint8_t mode);
        void setInputMode(uint8_t axis, uint8_t mode);
        void setCurrentLim(uint8_t axis, float current);
        void setVelLim(uint8_t axis, float vel);
        void setPosGain(uint8_t axis, float gain);
        void setVelGain(uint8_t axis, float gain);
        void setVelIntGain(uint8_t axis, float gain);

        void setPosition(uint8_t axis, float pos) const;            // sends a position command to the ODrive
        void setVelocity(uint8_t axis, float vel) const;            // sends a velocity command to the ODrive
};

#endif