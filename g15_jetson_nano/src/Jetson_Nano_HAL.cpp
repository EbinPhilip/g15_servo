
#include "Jetson_Nano_HAL.h"

const std::string g15_jetson_nano_hal_string = "g15_jetson_nano_hal";

Jetson_Nano_HAL::Jetson_Nano_HAL(const std::string &serial_port,
                                 uint8_t ctrlpin,
                                 const std::string &gpio_chip,
                                 Driver_Mode::Mode driver_mode)
    : Linux_HAL(serial_port),
      gpio_chip_(gpio_chip),
      gpio_line_(gpio_chip_.get_line(ctrlpin)),
      driver_mode_(driver_mode)
{
    gpio_line_.request({g15_jetson_nano_hal_string,
                        gpiod::line_request::DIRECTION_OUTPUT,
                        driver_mode_.tx_mode});
}

void Jetson_Nano_HAL::setDriverMode(Driver_Mode::Mode mode)
{
    driver_mode_.setMode(mode);
}

void Jetson_Nano_HAL::setRxMode()
{
    gpio_line_.set_value(driver_mode_.rx_mode);
}

void Jetson_Nano_HAL::setTxMode()
{
    gpio_line_.set_value(driver_mode_.rx_mode);
}
