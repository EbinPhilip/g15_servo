#include <iostream>
#include <chrono>

#include <signal.h>

#include "Linux_HAL.h"
#include "Linux_Passthrough_HAL.h"
#include "G15_Servo.h"

bool stop_execution = false;

void signalHandler(int s)
{
    stop_execution = true;
}

uint16_t position1 = ConvertAngleToPos(90);
uint16_t position2 = ConvertAngleToPos(180);

uint16_t speed1 = ConvertRPMToVal(50);
uint16_t speed2 = ConvertRPMToVal(30);

uint8_t packet1[] = 
{
    (uint8_t)(position1 & 0xff),
    (uint8_t)(position1 >> 8),
    (uint8_t)(speed1 & 0xff),
    (uint8_t)(speed1 >> 8),
};

uint8_t packet2[] = 
{
    (uint8_t)(position2 & 0xff),
    (uint8_t)(position2 >> 8),
    (uint8_t)(speed2 & 0xff),
    (uint8_t)(speed2 >> 8),
};


void waitAndRead(G15_Servo& servo, unsigned long wait_ms)
{
    
    auto t1 = std::chrono::steady_clock::now();

    uint8_t read_bytes[4] = {0};
    uint16_t value = 0;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
    -t1).count() < wait_ms && !stop_execution)
    {
        for (int i = 1; i<=3;++i)
        {
            std::cout<<("Servo "+std::to_string(i)+" -> ");
            if ( !servo.readInstruction(i, PRESENT_POSITION_L, read_bytes, 4) )
            {
                value = read_bytes[1]&0x03;
                value = (value << 8) | read_bytes[0];
                std::cout<<("Pos: ");
                std::cout<<(servo.convertFromRegisterValue(value, POSITION_MAX_REGISTER, 360.0));
                std::cout<<(" degrees");
                std::cout<<(" | ");
                value = read_bytes[3]&0x01;
                value = (value << 8) | read_bytes[2];
                std::cout<<("Speed: ");
                std::cout<<(servo.convertFromRegisterValue(value, SPEED_MAX_REGISTER, SPEED_MAX_RPM));
                std::cout<<(" rpm");
            }
            else
            {
                std::cout<<("Error");
            }
            if (i!=3)
                std::cout<<("\t|||\t");
        }
        std::cout<<std::endl;
    }
}

int main(void)
{
    signal(SIGINT, signalHandler);

    Linux_Passthrough_HAL hal("/dev/ttyUSB0");
    G15_Servo servo(hal);

    servo.begin(115200);
    hal.delayMilliseconds(2000);

     // Make sure servo in position mode
    servo.exitWheelMode(1);
    servo.exitWheelMode(2);
    servo.exitWheelMode(3);

    while (!stop_execution)
    {
        // servo.setLED(2, 1);
        // hal.delayMilliseconds(1000);
        // servo.setLED(2,0);
        // hal.delayMilliseconds(1000);

        uint8_t on = true;
        uint8_t off = false;

        servo.writeInstruction(BROADCAST_ID, LED, &on, 1);
        servo.regWriteInstruction(1, GOAL_POSITION_L, packet1, 4);
        servo.regWriteInstruction(2, GOAL_POSITION_L, packet2, 4);
        servo.regWriteInstruction(3, GOAL_POSITION_L, packet1, 4);
        servo.actionInstruction();
        // hal.delayMilliseconds(2000);
        waitAndRead(servo, 2000);

        servo.writeInstruction(BROADCAST_ID, LED, &off, 1);
        servo.regWriteInstruction(1, GOAL_POSITION_L, packet2, 4);
        servo.regWriteInstruction(2, GOAL_POSITION_L, packet1, 4);
        servo.regWriteInstruction(3, GOAL_POSITION_L, packet2, 4);
        servo.actionInstruction();

        // hal.delayMilliseconds(2000);
        waitAndRead(servo, 2000);
    }

    servo.end();
}