#ifndef __G15_JETSON_NANO_H__
#define __G15_JETSON_NANO_H__

#include <string>
#include <gpiod.h>

#include "g15_servo/Linux_HAL.h"

class Jetson_Nano_HAL : public Linux_HAL
{
public:
    Jetson_Nano_HAL(const std::string &serial_port,
                    uint8_t ctrlpin,
                    const std::string& gpio_chip  = "/dev/gpiochip0",
                    Driver_Mode::Mode driver_mode = Driver_Mode::Mode::B);
    
    ~Jetson_Nano_HAL();

    virtual void setDriverMode(Driver_Mode::Mode mode) override;

    virtual void setRxMode() override;
    virtual void setTxMode() override;

protected:
    gpiod_chip* gpio_chip_;
    gpiod_line* gpio_line_;
    Driver_Mode driver_mode_;
};

#endif
