#include <thread>
#include <chrono>

#include "Linux_HAL.h"

Linux_HAL::Linux_HAL(const std::string& serial_port)
:serial_(serial_port)
{
}

Linux_HAL::~Linux_HAL()
{
    end();
}

void Linux_HAL::begin(uint32_t baudrate, uint32_t timeout)
{
    serial_.setBaudrate(baudrate);
    serial_timeout_ = serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(serial_timeout_);
}

void Linux_HAL::end()
{
    serial_.close();
}

void Linux_HAL::delayMilliseconds(unsigned long ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void Linux_HAL::writeData(uint8_t data[], uint8_t length)
{
    serial_.flushInput();
    serial_.write(data, length);
    serial_.flushOutput();
}

void Linux_HAL::writeData(uint8_t header[], uint8_t data[], const uint8_t& checksum,  const uint8_t& header_length, const uint8_t& data_length)
{
    serial_.flushInput();
    serial_.write(header, header_length);
    serial_.write(data, data_length);
    serial_.write(&checksum, 1);
    serial_.flushOutput();
}

uint8_t Linux_HAL::readData(uint8_t data[], uint8_t length)
{
    return serial_.read(data, length);
}

uint8_t Linux_HAL::readData(uint8_t header[], uint8_t data[], uint8_t& checksum, const uint8_t& data_length)
{
    uint8_t count_read = 0;
    count_read += serial_.read(header, G15_RESPONSE_PACKET_HEADER_LENGTH);
    if (data_length)
    {
        count_read += serial_.read(data, data_length);
    }
    count_read += serial_.read(&checksum, G15_RESPONSE_PACKET_CHECKSUM_LENGTH);
    return count_read;
}