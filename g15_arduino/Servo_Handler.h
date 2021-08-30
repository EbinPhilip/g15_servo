#ifndef __SERVO_HANDLER_H__
#define __SERVO_HANDLER_H__

#include "Passthrough_Handler.h"
#include "Arduino_Passthrough_HAL.h"
#include "G15_Constants.h"

class Servo_Handler : public Handler_Interface
{
public:
    Servo_Handler(Arduino_Passthrough_HAL& hal, uint32_t baudrate, uint32_t servo_timeout = SerialTimeOut);

    virtual void handle_packet(Passthrough_Packet& packet) override;

protected:
    virtual bool _check_handling_criteria(Passthrough_Packet& packet);

    Arduino_Passthrough_HAL& hal_;
};

#endif