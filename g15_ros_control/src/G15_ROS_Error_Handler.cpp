#include "G15_ROS_Error_Handler.h"

#include <cstdio>

void G15_ROS_Error_Handler::registerSentPacket(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t &header_length, const uint8_t &length)
{
    sent_packet_.configure(header, checksum, data, header_length, length);
}

void G15_ROS_Error_Handler::registerReceivedPacket(const uint8_t &servoID, uint8_t header[], const uint8_t &checksum,
                                                   uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                                                   const uint8_t &actual_checksum, const uint16_t &error)
{
    received_packet_.configure(servoID, header, checksum, data, expected_size, actual_size, actual_checksum, error);
}

void G15_ROS_Error_Handler::handleErrors()
{
    if (received_packet_.error_)
    {
        error_msg_ = "g15 " + std::to_string(received_packet_.servoID_) + " error:- ";
        if (checkNoResponseError(received_packet_.error_))
        {
            error_msg_.append("\nNoResponse");
        }
        else
        {
            if (checkAngleLimitError(received_packet_.error_))
            {
                error_msg_.append("\nAngleLimit");
            }
            if (checkInstructionError(received_packet_.error_))
            {
                error_msg_.append("\nInstruction");
            }

            if (checkOverheatError(received_packet_.error_))
            {
                error_msg_.append("\nOverheat");
            }
            if (checkOverloadError(received_packet_.error_))
            {
                error_msg_.append("\nOverload");
            }
            if (checkRangeError(received_packet_.error_))
            {
                error_msg_.append("\nRange");
            }
            if (checkReceivedChecksumError(received_packet_.error_))
            {
                error_msg_.append("\nChecksum|Actual:" + std::to_string(received_packet_.checksum_)
                + "|" + std::to_string(received_packet_.actual_checksum_));
            }
            if (checkReceivedHeaderError(received_packet_.error_))
            {
                error_msg_.append("\nHeader:" + std::to_string(received_packet_.header_[0])
                + "," + std::to_string(received_packet_.header_[1]));
            }
            if (checkReceivedIDError(received_packet_.error_))
            {
                error_msg_.append("\nID|RecvID:" + std::to_string(received_packet_.servoID_)
                + "|" + std::to_string(received_packet_.header_[2]));
            }
            if (checkReceivedLengthError(received_packet_.error_))
            {
                error_msg_.append("\nLength|Actual:" + std::to_string(received_packet_.expected_size_)
                + "|" + std::to_string(received_packet_.actual_size_));
            }
            if (checkSentChecksumError(received_packet_.error_))
            {
                error_msg_.append("\nSentChecksum");
            }
            if (checkVoltageError(received_packet_.error_))
            {
                error_msg_.append("\nVoltage");
            }
        }
    }

    sent_packet_.unConfigure();
    received_packet_.unConfigure();
}

std::string G15_ROS_Error_Handler::getErrorDetails()
{
    return error_msg_;
}