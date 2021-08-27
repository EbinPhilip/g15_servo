#ifndef __ARDUINO_SERIAL_HAL_H__
#define __ARDUINO_SERIAL_HAL_H__

#include <HardwareSerial.h>

#include "Arduino_HAL.h"

class Arduino_Serial_HAL : public Arduino_HAL
{
public:
    Arduino_Serial_HAL(HardwareSerial& serial, uint8_t ctrlpin, Driver_Mode::Mode driver_mode = Driver_Mode::Mode::B);
    ~Arduino_Serial_HAL();

    virtual void begin(uint32_t baudrate, uint32_t timeout) override;
    virtual void end() override;

    virtual void writeData(uint8_t data[], uint8_t length) override;
    virtual void writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length) override;
    virtual uint8_t readData(uint8_t data[], uint8_t length) override;
    virtual uint8_t readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length) override;
protected:
    void _clearRxBuffer();
    void _listenSerial();
    HardwareSerial& serial_;
};

#endif