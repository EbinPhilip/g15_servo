#ifndef __G15_PASSTHROUGH_H__
#define __G15_PASSTHROUGH_H__

#include <stdint.h>

namespace G15_Passthrough
{
    const int COMMAND_LENGTH = 5;
    typedef uint8_t command_packet[COMMAND_LENGTH];
    const uint8_t HEADER = 0xF1;
    const uint8_t DRIVER_MODE_COMMAND = 0x01;

    void populate_passthrough_command(const uint8_t& command, const uint8_t& value, command_packet& packet)
    {
        packet[0] = packet[1] = HEADER;
        packet[2] = command;
        packet[3] = value;
        packet[4] =  ~( packet[0] + 
                        packet[1] + 
                        packet[2] + 
                        packet[3] );
    }
}

#endif