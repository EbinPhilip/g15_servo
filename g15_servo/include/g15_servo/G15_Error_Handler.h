#ifndef __G15_ERROR_HANDLER_H__
#define __G15_ERROR_HANDLER_H__

class G15_Error_Handler
{
public:
    virtual void registerSentPacket(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t& header_length, const uint8_t &length)
    {
    }
    virtual void registerReceivedPacket(const uint8_t& servoID, uint8_t header[], const uint8_t &checksum,
                                        uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                                        const uint8_t &actual_checksum, const uint16_t &error)
    {
    }
    virtual void handleErrors()
    {
    }
};

#endif