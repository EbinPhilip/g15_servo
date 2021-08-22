
#include <cstdio>
#include "Jetson_Nano_HAL.h"

const std::string g15_jetson_nano_hal_string = "g15_jetson_nano_hal";

Jetson_Nano_HAL::Jetson_Nano_HAL(const std::string &serial_port,
                                 uint8_t ctrlpin,
                                 Driver_Mode::Mode driver_mode)
    : Linux_HAL(serial_port),
      ctrlpin_(ctrlpin),
      driver_mode_(driver_mode)
{
    FILE *fd = fopen("/sys/class/gpio/export", "w");
    fprintf(fd, "%d", ctrlpin_);
    fclose(fd);

    delayMilliseconds(20);

    char gpio_string[100];

    sprintf(gpio_string, "/sys/class/gpio/gpio%d/direction", ctrlpin_);
    fd = fopen(gpio_string, "w");
    fprintf(fd, "out");
    fclose(fd);

    sprintf(gpio_string, "/sys/class/gpio/gpio%d/value", ctrlpin_);
    gpio_fs_handle_ = fopen(gpio_string, "w");
}

Jetson_Nano_HAL::~Jetson_Nano_HAL()
{
    fclose(gpio_fs_handle_);

    FILE *fd = fopen("/sys/class/gpio/unexport", "w");
    fprintf(fd, "%d", ctrlpin_);
    fclose(fd);
}

void Jetson_Nano_HAL::setDriverMode(Driver_Mode::Mode mode)
{
    driver_mode_.setMode(mode);
}

void Jetson_Nano_HAL::setRxMode()
{
    fprintf(gpio_fs_handle_, "%d", driver_mode_.rx_mode);
}

void Jetson_Nano_HAL::setTxMode()
{
    fprintf(gpio_fs_handle_, "%d", driver_mode_.tx_mode);
}
