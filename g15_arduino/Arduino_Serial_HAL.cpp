#include "Arduino_Serial_HAL.h"

#include "Arduino_Software_Serial_HAL.h"

#define serial_port Serial3

Arduino_Serial_HAL::Arduino_Serial_HAL(HardwareSerial& serial, uint8_t ctrlpin, Driver_Mode::Mode driver_mode)
    : Arduino_HAL(ctrlpin, driver_mode),
      serial_(serial)
{
}

Arduino_Serial_HAL::~Arduino_Serial_HAL()
{
    end();
}

void Arduino_Serial_HAL::begin(uint32_t baudrate, uint32_t timeout)
{
    serial_.begin(baudrate);
    serial_.setTimeout(timeout);
    setTxMode();
}

void Arduino_Serial_HAL::end()
{
    serial_.end();
}

void Arduino_Serial_HAL::writeData(uint8_t data[], uint8_t length)
{
    _clearRxBuffer();
    for (uint8_t i = 0; i < length; i++)
    {
        serial_.write(data[i]);
    }
    serial_.flush();
}

void Arduino_Serial_HAL::writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length)
{
    _clearRxBuffer();
    for (uint8_t i = 0; i < header_length; i++)
    {
        serial_.write(header[i]);
    }
    for (uint8_t i = 0; i < data_length; i++)
    {
        serial_.write(data[i]);
    }
    serial_.write(checksum);
    serial_.flush();
}

uint8_t Arduino_Serial_HAL::readData(uint8_t data[], uint8_t length)
{
    return serial_.readBytes(data, length);
}

uint8_t Arduino_Serial_HAL::readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length)
{
    uint8_t count_read = 0;
    count_read += serial_.readBytes(header, G15_RESPONSE_PACKET_HEADER_LENGTH);
    if (data_length)
    {
        count_read += serial_.readBytes(data, data_length);
    }
    count_read += serial_.readBytes(&checksum, G15_RESPONSE_PACKET_CHECKSUM_LENGTH);
    return count_read;
}

void Arduino_Serial_HAL::_clearRxBuffer()
{
    while (serial_.available())
    {
        serial_.read();
    }
}