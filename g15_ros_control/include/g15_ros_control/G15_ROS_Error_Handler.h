#ifndef __G15_ROS_ERROR_HANDLER_H__
#define __G15_ROS_ERROR_HANDLER_H__

#include <stdint.h>
#include <g15_servo/G15_Error_Handler.h>
#include <g15_servo/G15_Error_Handler_Utils.h>

#include <string>

class G15_ROS_Error_Handler : public G15_Error_Handler, public G15_Error_Check
{
public:
    virtual void registerSentPacket(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t& header_length, const uint8_t &length) override;
    virtual void registerReceivedPacket(const uint8_t& servoID, uint8_t header[], const uint8_t &checksum,
                                        uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                                        const uint8_t &actual_checksum, const uint16_t &error) override;
    virtual void handleErrors() override;
    virtual std::string getErrorDetails();
protected:
    Sent_Packet_Config sent_packet_;
    Received_Packet_Config received_packet_;

    std::string error_msg_;
};

#endif