#include "Arduino_HAL.h"

Arduino_HAL::Arduino_HAL(uint8_t rxpin, uint8_t txpin, uint8_t ctrlpin, Driver_Mode::Mode driver_mode)
    : txpin_(txpin),
      rxpin_(rxpin),
      ctrlpin_(ctrlpin),
      serial_(nullptr),
      driver_mode_(driver_mode)
{
}

void Arduino_HAL::begin(uint32_t baudrate, uint32_t timeout)
{
    pinMode(rxpin_, INPUT);
    pinMode(txpin_, OUTPUT);
    serial_ = new SoftwareSerial(rxpin_, txpin_);
    serial_->begin(baudrate);
    serial_->setTimeout(timeout);

    pinMode(ctrlpin_, OUTPUT);
    setTxMode();
}

void Arduino_HAL::end()
{
    pinMode(rxpin_, INPUT);
    pinMode(txpin_, INPUT);
    serial_->end();

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

void Arduino_HAL::writeData(uint8_t data[], uint8_t length)
{
    _listenSerial();
    for (uint8_t i = 0; i < length; i++)
    {
        serial_->write(data[i]);
    }
    serial_->flush();
}

void Arduino_HAL::writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length)
{
    _listenSerial();
    for (uint8_t i = 0; i < header_length; i++)
    {
        serial_->write(header[i]);
    }
    for (uint8_t i = 0; i < data_length; i++)
    {
        serial_->write(data[i]);
    }
    serial_->write(checksum);
    serial_->flush();
}

uint8_t Arduino_HAL::readData(uint8_t data[], uint8_t length)
{
    return serial_->readBytes(data, length);
}

uint8_t Arduino_HAL::readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length)
{
    uint8_t count_read = 0;
    count_read += serial_->readBytes(header, G15_RESPONSE_PACKET_HEADER_LENGTH);
    if (data_length)
    {
        count_read += serial_->readBytes(data, data_length);
    }
    count_read += serial_->readBytes(&checksum, G15_RESPONSE_PACKET_CHECKSUM_LENGTH);
    return count_read;
}

void Arduino_HAL::_clearRxBuffer()
{
    while (serial_->available())
    {
        serial_->read();
    }
}

void Arduino_HAL::_listenSerial()
{
    if (!serial_->listen())
    {
        _clearRxBuffer();
    }
}
