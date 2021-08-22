
#include <cstdio>
#include <ios>
#include "Jetson_Nano_HAL.h"

Jetson_Nano_HAL::Jetson_Nano_HAL(const std::string &serial_port,
                                 uint8_t ctrlpin,
                                 Driver_Mode::Mode driver_mode)
    : Linux_HAL(serial_port),
      ctrlpin_(ctrlpin),
      driver_mode_(driver_mode)
{
    FILE *fd = _openFile("/sys/class/gpio/export", "w");
    fprintf(fd, "%d", ctrlpin_);
    fclose(fd);

    delayMilliseconds(20);

    char gpio_string[100];

    sprintf(gpio_string, "/sys/class/gpio/gpio%d/direction", ctrlpin_);
    fd = _openFile(gpio_string, "w");
    fprintf(fd, "out");
    fclose(fd);

    sprintf(gpio_string, "/sys/class/gpio/gpio%d/value", ctrlpin_);
    gpio_fs_handle_ = _openFile(gpio_string, "w");
}

Jetson_Nano_HAL::~Jetson_Nano_HAL()
{
    fclose(gpio_fs_handle_);

    FILE *fd = _openFile("/sys/class/gpio/unexport", "w");
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

FILE* Jetson_Nano_HAL::_openFile(const char* path, const char* mode)
{
    FILE* ret = fopen(path, mode);
    if (!ret)
    {
        throw std::ios_base::failure("file open failed: "+std::string(path));
    }
    else
    {
        return ret;
    }
}
