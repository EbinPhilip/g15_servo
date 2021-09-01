#include "Arduino_HAL.h"

Arduino_HAL::Arduino_HAL(uint8_t ctrlpin, Driver_Mode::Mode driver_mode = Driver_Mode::Mode::B)
    : ctrlpin_(ctrlpin),
      driver_mode_(driver_mode)
{
    pinMode(ctrlpin_, OUTPUT);
}

Arduino_HAL::~Arduino_HAL()
{
    pinMode(ctrlpin_, INPUT);
}

void Arduino_HAL::setRxMode()
{
    digitalWrite(ctrlpin_, driver_mode_.rx_mode);
}

void Arduino_HAL::setTxMode()
{
    digitalWrite(ctrlpin_, driver_mode_.tx_mode);
}

void Arduino_HAL::setDriverMode(Driver_Mode::Mode mode)
{
    driver_mode_ = mode;
}

void Arduino_HAL::delayMilliseconds(unsigned long ms)
{
    delay(ms);
}

