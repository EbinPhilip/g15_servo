#ifndef __ARDUINO_HAL_H__
#define __ARDUINO_HAL_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "G15_HAL.h"

class Arduino_HAL : public G15_HAL
{
public:
    Arduino_HAL(uint8_t ctrlpin, Driver_Mode::Mode driver_mode = Driver_Mode::Mode::B);
    ~Arduino_HAL();

    virtual void setRxMode() override;
    virtual void setTxMode() override;

    virtual void setDriverMode(Driver_Mode::Mode mode) override;

    virtual void delayMilliseconds(unsigned long ms) override;

protected:
    uint8_t ctrlpin_;
    SoftwareSerial *serial_;
    Driver_Mode driver_mode_;
};

#endif