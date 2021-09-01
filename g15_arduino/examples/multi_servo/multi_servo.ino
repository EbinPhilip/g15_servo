#include <SoftwareSerial.h>

#include <Arduino.h>
#include "G15_Servo.h"
#include "Arduino_HAL.h"
#include "Arduino_Software_Serial_HAL.h"
#include "Arduino_Serial_HAL.h"
#include "Arduino_Passthrough_HAL.h"
#include "G15_Constants.h"

#include "Arduino_Error_Handler.h"


Arduino_Software_Serial_HAL hal(2, 3, 8, Driver_Mode::Mode::B);
// Arduino_Serial_HAL hal(Serial3, 8, Driver_Mode::Mode::B);
// Arduino_Passthrough_HAL hal_p(hal);
Arduino_Error_Handler error_handler(false);
G15_Servo g15(hal &error_handler);

uint16_t position1 = ConvertAngleToPos(90);
uint16_t position2 = ConvertAngleToPos(180);

uint16_t speed1 = ConvertRPMToVal(50);
uint16_t speed2 = ConvertRPMToVal(30);

uint8_t packet1[] = 
{
    position1 & 0xff,
    position1 >> 8,
    speed1 & 0xff,
    speed1 >> 8,
};

uint8_t packet2[] = 
{
    position2 & 0xff,
    position2 >> 8,
    speed2 & 0xff,
    speed2 >> 8,
};

void setup()
{
    pinMode(2, INPUT);
    pinMode(3, OUTPUT);

    g15.begin(19200);

    Serial.begin(115200);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    delay(1000);
    digitalWrite(LED, HIGH);

    // Make sure G15 in position mode
    g15.exitWheelMode(1);
    g15.exitWheelMode(2);
    g15.exitWheelMode(3);
}

void loop()
{
    uint8_t on = true;
    uint8_t off = false;

    g15.writeInstruction(BROADCAST_ID, LED, &on, 1);
    g15.regWriteInstruction(1, GOAL_POSITION_L, packet1, 4);
    g15.regWriteInstruction(2, GOAL_POSITION_L, packet2, 4);
    g15.regWriteInstruction(3, GOAL_POSITION_L, packet1, 4);
    g15.actionInstruction();

    // delay(2000);

    waitAndRead(2000);

    g15.writeInstruction(BROADCAST_ID, LED, &off, 1);
    g15.regWriteInstruction(1, GOAL_POSITION_L, packet2, 4);
    g15.regWriteInstruction(2, GOAL_POSITION_L, packet1, 4);
    g15.regWriteInstruction(3, GOAL_POSITION_L, packet2, 4);
    g15.actionInstruction();

    // delay(2000);

    waitAndRead(2000);

}

void waitAndRead(unsigned long wait_ms)
{
    
    unsigned long t1 = millis();

    uint8_t read_bytes[4] = {0};
    uint16_t value = 0;

    while ((millis()-t1)< wait_ms)
    {
        for (int i = 1; i<=3;++i)
        {
            Serial.print("Servo "+String(i)+" -> ");
            if ( !g15.readInstruction(i, PRESENT_POSITION_L, read_bytes, 4) )
            {
                value = read_bytes[1]&0x03;
                value = (value << 8) | read_bytes[0];
                Serial.print("Pos: ");
                Serial.print(g15.convertFromRegisterValue(value, POSITION_MAX_REGISTER, 360.0), 2);
                Serial.print(" degrees");
                Serial.print(" | ");
                value = read_bytes[3]&0x01;
                value = (value << 8) | read_bytes[2];
                Serial.print("Speed: ");
                Serial.print(g15.convertFromRegisterValue(value, SPEED_MAX_REGISTER, SPEED_MAX_RPM), 2);
                Serial.print(" rpm");
            }
            else
            {
                Serial.print("Error");
            }
            if (i!=3)
                Serial.print("\t|||\t");
        }
        Serial.println();
    }
}


