#ifndef __ARDUINO_HAL_H__
#define __ARDUINO_HAL_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "G15_HAL.h"

class Arduino_HAL : public G15_HAL
{
public:
    Arduino_HAL(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, Driver_Mode::Mode driver_mode = Driver_Mode::Mode::B);

    virtual void begin(uint32_t baudrate, uint32_t timeout) override;
    virtual void end() override;

    virtual void setRxMode() override;
    virtual void setTxMode() override;

    virtual void setDriverMode(Driver_Mode::Mode mode) override;

    virtual void delayMilliseconds(unsigned long ms) override;

    virtual void writeData(uint8_t data[], uint8_t length) override;
    virtual void writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length) override;
    virtual uint8_t readData(uint8_t data[], uint8_t length) override;
    virtual uint8_t readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length) override;

protected:
    void _clearRxBuffer();
    void _listenSerial();
    uint8_t txpin_, rxpin_, ctrlpin_;
    SoftwareSerial *serial_;
    Driver_Mode driver_mode_;
};

#endif