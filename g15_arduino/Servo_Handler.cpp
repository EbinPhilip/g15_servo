#include <Arduino.h>

#include "Servo_Handler.h"

Servo_Handler::Servo_Handler(Arduino_Passthrough_HAL& hal, uint32_t baudrate, uint32_t servo_timeout = SerialTimeOut)
    : hal_(hal)
{
    hal_.begin(baudrate, servo_timeout);
}

void Servo_Handler::handle_packet(Passthrough_Packet& packet)
{
    if (_check_handling_criteria(packet))
    {
        if (Serial.readBytes(packet.data+packet.current_length, 2) == 2)
        {
            packet.current_length += 2;
            if ( Serial.readBytes(packet.data+packet.current_length, packet.data[sent_packet::length]) ==  packet.data[sent_packet::length] )
            {
                packet.current_length+=packet.data[sent_packet::length];
                uint8_t read_data_length = RESPONSE_PACKET_LENGTH;
                if (packet.data[sent_packet::instruction] == iREAD_DATA)
                {
                    read_data_length += packet.data[sent_packet::data_length];
                }
                hal_.writeData(packet.data, packet.current_length);
                int read_count = hal_.readData(packet.data, read_data_length);
                for (uint8_t i = 0; i < read_count; i++)
                {
                    Serial.write(packet.data[i]);
                }
                Serial.flush();
            }

        }
    }
    else
    {
        if (next_)
        {
            next_->handle_packet(packet);
        }
    }
}

bool Servo_Handler::_check_handling_criteria(Passthrough_Packet& packet)
{
    if (packet.data[sent_packet::header1] == PACKET_HEADER
    &&  packet.data[sent_packet::header2] == PACKET_HEADER)
    {
        return true;
    }
    else
    {
        return false;
    }
}
