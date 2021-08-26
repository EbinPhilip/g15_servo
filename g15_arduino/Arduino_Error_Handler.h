#ifndef __ARDUINO_ERROR_HANDLER_H__
#define __ARDUINO_ERROR_HANDLER_H__

#include <Arduino.h>

#include "G15_Error_Handler.h"
#include "G15_Error_Handler_Utils.h"

class Arduino_Error_Handler : public G15_Error_Handler, public G15_Error_Check
{
public:
    Arduino_Error_Handler(const bool& print_packets = false);

    virtual void registerSentPacket(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t& header_length, const uint8_t &length) override;
    virtual void registerReceivedPacket(const uint8_t& servoID, uint8_t header[], const uint8_t &checksum,
                                        uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                                        const uint8_t &actual_checksum, const uint16_t &error) override;
    virtual void handleErrors() override;
protected:
    bool print_packets_;

    Sent_Packet_Config sent_packet_;
    Received_Packet_Config received_packet_;
};

#endif