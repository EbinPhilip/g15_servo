#ifndef __LINUX_HAL__
#define __LINUX_HAL__

#include <string>

#include "serial/serial.h"
#include "g15_servo/G15_HAL.h"

class Linux_HAL : public G15_HAL
{
public:
    Linux_HAL(const std::string& serial_port);
    ~Linux_HAL();

    virtual void begin(uint32_t baudrate, uint32_t timeout);
    virtual void end();

    virtual void delayMilliseconds(unsigned long ms) override;

    virtual void writeData(uint8_t data[], uint8_t length) override;
    virtual void writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length) override;
    virtual uint8_t readData(uint8_t data[], uint8_t length) override;
    virtual uint8_t readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length) override;

    protected:
    serial::Serial serial_;
    serial::Timeout serial_timeout_;
};

#endif