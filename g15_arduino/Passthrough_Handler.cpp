#include <Arduino.h>

#include "Passthrough_Handler.h"
#include "G15_Constants.h"

Passthrough_Handler::Passthrough_Handler(unsigned long timeout)
: serial_timeout_(timeout)
{
}

void Passthrough_Handler::handle_packet(Passthrough_Packet& packet)
{
    uint8_t byte1, byte2;
    if (Serial.available()>=2)
    {
        unsigned long t1 = millis();
        Serial.readBytes(&byte1, 1);
        while(millis()- t1 < serial_timeout_)
        {
            if (Serial.readBytes(&byte2, 1))
            {
                t1 = millis();
            }
            if (byte1 == byte2)
            {
                packet.data[sent_packet::header1] = byte1;
                packet.data[sent_packet::header2] = byte2;
                packet.current_length += 2;
                if (next_)
                {
                    next_->handle_packet(packet);
                    return;
                }
            }
            else
            {
                byte1 = byte2;
            }
        }
    }
}
