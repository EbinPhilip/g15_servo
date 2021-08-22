
#include "Jetson_Nano_HAL.h"

const std::string g15_jetson_nano_hal_string = "g15_jetson_nano_hal";

Jetson_Nano_HAL::Jetson_Nano_HAL(const std::string &serial_port,
                                 uint8_t ctrlpin,
                                 const std::string &gpio_chip,
                                 Driver_Mode::Mode driver_mode)
    : Linux_HAL(serial_port),
      driver_mode_(driver_mode)
{
    gpio_chip_ = gpiod_chip_open("/dev/gpiochip0");
    gpio_line_ = gpiod_chip_get_line(gpio_chip_, ctrlpin);
    int ret = gpiod_line_request_output(gpio_line_, g15_jetson_nano_hal_string.c_str(), driver_mode_.tx_mode);
}

Jetson_Nano_HAL::~Jetson_Nano_HAL()
{
    gpiod_chip_close(gpio_chip_);
}

void Jetson_Nano_HAL::setDriverMode(Driver_Mode::Mode mode)
{
    driver_mode_.setMode(mode);
}

void Jetson_Nano_HAL::setRxMode()
{
    gpiod_line_set_value(gpio_line_, driver_mode_.rx_mode);
}

void Jetson_Nano_HAL::setTxMode()
{
    gpiod_line_set_value(gpio_line_, driver_mode_.rx_mode);
}
