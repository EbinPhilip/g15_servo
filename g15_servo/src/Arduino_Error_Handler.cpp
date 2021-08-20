
#include "Arduino_Error_Handler.h"

Arduino_Error_Handler::Arduino_Error_Handler(const bool &print_packets)
    : print_packets_(print_packets)
{
}

void Arduino_Error_Handler::registerSentPacket(uint8_t header[], const uint8_t &checksum, uint8_t data[], const uint8_t &header_length, const uint8_t &length)
{
    sent_packet_.configure(header, checksum, data, header_length, length);
}

void Arduino_Error_Handler::registerReceivedPacket(const uint8_t &servoID, uint8_t header[], const uint8_t &checksum,
                                                   uint8_t data[], const uint8_t &expected_size, const uint8_t &actual_size,
                                                   const uint8_t &actual_checksum, const uint16_t &error)
{
    received_packet_.configure(servoID, header, checksum, data, expected_size, actual_size, actual_checksum, error);
}

void Arduino_Error_Handler::handleErrors()
{
    if (print_packets_)
    {
        if (sent_packet_.isConfigured())
        {
            // Sent packet
            Serial.println("Sent:");
            for (uint8_t i = 0; i < sent_packet_.header_length_; i++)
            {
                Serial.print(sent_packet_.header_[i], HEX);
                Serial.print(" ");
            }
            for (uint8_t i = 0; i < sent_packet_.data_length_; i++)
            {
                Serial.print(sent_packet_.data_[i], HEX);
                Serial.print(" ");
            }
            Serial.print(sent_packet_.checksum_, HEX);
            Serial.println();
        }

        if (received_packet_.isConfigured())
        {
            // Received packet
            Serial.println("Received:");
            uint8_t count = 0;
            for (uint8_t i = 0; (i < 5) && (count < received_packet_.actual_size_); i++, count++)
            {
                Serial.print(received_packet_.header_[i], HEX);
                Serial.print(" ");
            }
            for (uint8_t i = 0; (i < (received_packet_.expected_size_ - 6)) && (count < received_packet_.actual_size_); i++, count++)
            {
                Serial.print(received_packet_.data_[i], HEX);
                Serial.print(" ");
            }
            Serial.print(received_packet_.checksum_, HEX);
            Serial.println();
        }
    }

    if (received_packet_.error_)
    {
        Serial.println();
        Serial.println("Errors:");
        Serial.println("^^^^^^^");

        if (checkNoResponseError(received_packet_.error_))
        {
            Serial.println("NoResponse");
        }
        else
        {
            if (checkAngleLimitError(received_packet_.error_))
            {
                Serial.println("AngleLimit");
            }
            if (checkInstructionError(received_packet_.error_))
            {
                Serial.println("Instruction");
            }

            if (checkOverheatError(received_packet_.error_))
            {
                Serial.println("Overheat");
            }
            if (checkOverloadError(received_packet_.error_))
            {
                Serial.println("Overload");
            }
            if (checkRangeError(received_packet_.error_))
            {
                Serial.println("Range");
            }
            if (checkReceivedChecksumError(received_packet_.error_))
            {
                Serial.print("Checksum|Actual:");
                Serial.print(received_packet_.checksum_, HEX);
                Serial.print("|");
                Serial.println(received_packet_.actual_checksum_, HEX);
            }
            if (checkReceivedHeaderError(received_packet_.error_))
            {
                Serial.print("Header:");
                Serial.print(received_packet_.header_[0], HEX);
                Serial.println(received_packet_.header_[1], HEX);
            }
            if (checkReceivedIDError(received_packet_.error_))
            {
                Serial.print("ID|RecvID:");
                Serial.print(received_packet_.servoID_, HEX);
                Serial.print("|");
                Serial.println(received_packet_.header_[2], HEX);
            }
            if (checkReceivedLengthError(received_packet_.error_))
            {
                Serial.print("Length|Actual:");
                Serial.print(received_packet_.expected_size_, HEX);
                Serial.print("|");
                Serial.println(received_packet_.actual_size_, HEX);
            }
            if (checkSentChecksumError(received_packet_.error_))
            {
                Serial.println("SentChecksum");
            }
            if (checkVoltageError(received_packet_.error_))
            {
                Serial.println("Voltage");
            }
        }
    }

    if (print_packets_ || received_packet_.error_)
    {
        Serial.println();
        Serial.println("-------------------------------------------------");
        Serial.println();
    }

    sent_packet_.unConfigure();
    received_packet_.unConfigure();
}