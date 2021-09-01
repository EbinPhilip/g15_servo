#ifndef __ARDUINO_PASSTHROUGH_HAL_H__
#define __ARDUINO_PASSTHROUGH_HAL_H__

#include "Arduino_HAL.h"

class Arduino_Passthrough_HAL : public G15_HAL
{
public:
    Arduino_Passthrough_HAL(Arduino_HAL &hal)
        : hal_(hal)
    {
    }

    virtual void begin(uint32_t baudrate, uint32_t timeout) override
    {
        hal_.begin(baudrate, timeout);
    }

    virtual void end() override
    {
        hal_.end();
    }

    virtual void setRxMode() override
    {
        hal_.setRxMode();
    }

    virtual void setTxMode() override
    {
        hal_.setTxMode();
    }

    virtual void setDriverMode(Driver_Mode::Mode mode) override
    {
        hal_.setDriverMode(mode);
    }

    virtual void delayMilliseconds(unsigned long ms) override
    {
        hal_.delayMilliseconds(ms);
    }

    virtual void writeData(uint8_t data[], uint8_t length) override
    {
        hal_.setTxMode();
        hal_.writeData(data, length);
    }

    virtual uint8_t readData(uint8_t data[], uint8_t length) override
    {
        hal_.setRxMode();
        uint8_t ret = hal_.readData(data, length);
        hal_.setTxMode();
        return ret;
    }

    virtual void writeData(uint8_t header[], uint8_t data[], const uint8_t &checksum, const uint8_t &header_length, const uint8_t &data_length) override
    {
        hal_.setTxMode();
        hal_.writeData(header, data, checksum, header_length, data_length);
    }

    virtual uint8_t readData(uint8_t header[], uint8_t data[], uint8_t &checksum, const uint8_t &data_length) override
    {
        hal_.setRxMode();
        uint8_t ret = hal_.readData(header, data, checksum, data_length);
        hal_.setTxMode();
        return ret;
    }

protected:
    Arduino_HAL &hal_;
};

#endif