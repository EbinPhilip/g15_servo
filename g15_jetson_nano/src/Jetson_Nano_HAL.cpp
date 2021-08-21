#include "Jetson_Nano_HAL.h"

Jetson_Nano_HAL::Jetson_Nano_HAL(const std::string &serial_port,
                                 uint8_t ctrlpin,
                                 Driver_Mode::Mode driver_mode)
                                 : Linux_HAL(serial_port),
                                 ctrlpin_(ctrlpin),
                                 driver_mode_(driver_mode)
{
    GPIO::setmode(GPIO::BCM);
    GPIO::setup(ctrlpin_, GPIO::OUT);
}

Jetson_Nano_HAL::~Jetson_Nano_HAL()
{
    GPIO::cleanup(ctrlpin_); 
}

void Jetson_Nano_HAL::setDriverMode(Driver_Mode::Mode mode)
{
    driver_mode_.setMode(mode);
}

void Jetson_Nano_HAL::setRxMode()
{
    GPIO::output(ctrlpin_, driver_mode_.rx_mode);
}

void Jetson_Nano_HAL::setTxMode()
{
    GPIO::output(ctrlpin_, driver_mode_.tx_mode);
}

