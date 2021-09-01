#ifndef __PASSTHROUGH_HANDLER_H__
#define __PASSTHROUGH_HANDLER_H__

#include "inttypes.h"
#include "G15_Constants.h"

#define MAX_PASSTHROUGH_PACKET_LENGTH 25

struct Passthrough_Packet
{
    Passthrough_Packet()
    : data({0}),
    current_length(0)
    {
    }
    uint8_t data[MAX_PASSTHROUGH_PACKET_LENGTH];
    uint8_t current_length;
};

class Handler_Interface
{
public:
    virtual void attachNext(Handler_Interface* interface)
    {
        next_ = interface;
    }

    virtual void handle_packet(Passthrough_Packet& packet) = 0;
protected:
    Handler_Interface* next_;
};

class Passthrough_Handler : public Handler_Interface
{
public:
    Passthrough_Handler(unsigned long timeout = SerialTimeOut);
    virtual void handle_packet(Passthrough_Packet& packet) override;
protected:
    unsigned long serial_timeout_;
};

#endif