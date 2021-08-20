#ifndef __G15_HAL_H__
#define __G15_HAL_H__

#include <stdint.h>

#define G15_HIGH 1
#define G15_LOW 0

#define G15_RESPONSE_PACKET_HEADER_LENGTH 5
#define G15_RESPONSE_PACKET_CHECKSUM_LENGTH 1

struct Driver_Mode
{
    enum class Mode : uint8_t
    {
        A,
        B
    };

    bool rx_mode;
    bool tx_mode;

    Driver_Mode(Mode mode = Mode::B)
    {
        setMode(mode);
    }

    void setMode(Mode mode)
    {
        if (mode == Mode::A)
        {
            tx_mode = G15_HIGH;
            rx_mode = G15_LOW;
        }
        else
        {
            tx_mode = G15_LOW;
            rx_mode = G15_HIGH;
        }
    }
};

class G15_HAL
{
public:
    virtual void begin(uint32_t baudrate, uint32_t timeout) = 0;
    virtual void end() = 0;

    virtual void setRxMode() = 0;
    virtual void setTxMode() = 0;

    virtual void setDriverMode(Driver_Mode::Mode mode) = 0;

    virtual void delayMilliseconds(unsigned long ms) = 0;

    virtual void writeData(uint8_t data[], uint8_t length) = 0;
    virtual void writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length) = 0;
    virtual uint8_t readData(uint8_t data[], uint8_t length) = 0;
    virtual uint8_t readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length) = 0;
};

#endif