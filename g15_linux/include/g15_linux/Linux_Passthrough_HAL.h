#ifndef __LINUX_PASSTHROUGH_HAL_H__
#define __LINUX_PASSTHROUGH_HAL_H__

#include "Linux_HAL.h"

class Linux_Passthrough_HAL : public Linux_HAL
{
public:
    Linux_Passthrough_HAL(const std::string& serial_port)
        : Linux_HAL(serial_port)
    {
    }

    virtual void setDriverMode(Driver_Mode::Mode mode) override
    {
    }

    virtual void setRxMode() override
    {
    }

    virtual void setTxMode() override
    {
    }
};

#endif