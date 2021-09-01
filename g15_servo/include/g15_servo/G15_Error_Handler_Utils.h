#ifndef __G15_ERROR_HANDLER_UTILS_H__
#define __G15_ERROR_HANDLER_UTILS_H__

#include <stdint.h>

#include "G15_Constants.h"

class Packet_Config
{
public:
    Packet_Config()
        : configured_(false)
    {
    }
    void unConfigure()
    {
        configured_ = false;
    }
    bool isConfigured()
    {
        return configured_;
    }

protected:
    bool configured_;
};

struct Sent_Packet_Config : public Packet_Config
{
    Sent_Packet_Config()
    {
    }

    void configure(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t& header_length, const uint8_t &length)
    {
        checksum_ = checksum;
        data_ = data;
        data_length_ = length;
        header_length_ = header_length;
        for (uint8_t i = 0; i < header_length; ++i)
        {
            header_[i] = header[i];
        }
        configured_ = true;
    }

    uint8_t header_[6];
    uint8_t header_length_;
    uint8_t checksum_;
    uint8_t *data_;
    uint8_t data_length_;
};

struct Received_Packet_Config : public Packet_Config
{
    Received_Packet_Config()
        : error_(0)
    {
    }

    void configure(const uint8_t &servoID, uint8_t header[], const uint8_t &checksum,
                   uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                   const uint8_t &actual_checksum, const uint16_t &error)
    {
        servoID_ = servoID;
        checksum_ = checksum;
        data_ = data;
        expected_size_ = expected_size;
        actual_size_ = actual_size;
        actual_checksum_ = actual_checksum;
        error_ = error;
        for (uint8_t i = 0; i < 5 && i < actual_size_; ++i)
        {
            header_[i] = header[i];
        }

        configured_ = true;
    }

    uint8_t servoID_;
    uint8_t header_[5];
    uint8_t checksum_;
    uint8_t *data_;
    uint8_t expected_size_;
    uint8_t actual_size_;
    uint8_t actual_checksum_;
    uint16_t error_;
};

struct G15_Error_Check
{
    virtual bool checkError(uint16_t &error, uint16_t code)
    {
        return error & code;
    }
    virtual bool checkInstructionError(uint16_t &error)
    {
        return checkError(error, ALARM_INST);
    }
    virtual bool checkOverloadError(uint16_t &error)
    {
        return checkError(error, ALARM_OVERLOAD);
    }
    virtual bool checkSentChecksumError(uint16_t &error)
    {
        return checkError(error, ALARM_CHECKSUM);
    }
    virtual bool checkRangeError(uint16_t &error)
    {
        return checkError(error, ALARM_RANGE);
    }
    virtual bool checkOverheatError(uint16_t &error)
    {
        return checkError(error, ALARM_OVERHEAT);
    }
    virtual bool checkAngleLimitError(uint16_t &error)
    {
        return checkError(error, ALARM_ANGLELIMIT);
    }
    virtual bool checkVoltageError(uint16_t &error)
    {
        return checkError(error, ALARM_VOLTAGE);
    }
    virtual bool checkNoResponseError(uint16_t &error)
    {
        return (error == NO_RESPONSE_ERROR);
    }
    virtual bool checkReceivedLengthError(uint16_t &error)
    {
        return checkError(error, RECEIVED_LENGTH_ERROR);
    }
    virtual bool checkReceivedHeaderError(uint16_t &error)
    {
        return checkError(error, RECEIVED_HEADER_ERROR);
    }
    virtual bool checkReceivedIDError(uint16_t &error)
    {
        return checkError(error, RECEIVED_ID_ERROR);
    }
    virtual bool checkReceivedChecksumError(uint16_t &error)
    {
        return checkError(error, RECEIVED_CHECKSUM_ERROR);
    }
};

#endif