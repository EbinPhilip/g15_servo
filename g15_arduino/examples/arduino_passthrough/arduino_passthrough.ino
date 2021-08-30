#include <Arduino.h>
#include "G15_Servo.h"
#include "Arduino_HAL.h"
#include "Arduino_Software_Serial_HAL.h"
#include "Arduino_Serial_HAL.h"
#include "Arduino_Passthrough_HAL.h"
#include "G15_Constants.h"

#include "Servo_Handler.h"

Arduino_Serial_HAL hal(Serial3, 8, Driver_Mode::Mode::B);
Arduino_Passthrough_HAL hal_p(hal);

Passthrough_Handler passthrough_handler(5U);
Servo_Handler servo_handler(hal_p, 19200, 5U);

void setup()
{
    Serial.begin(115200);
    passthrough_handler.attachNext(&servo_handler);
}

void loop()
{
    Passthrough_Packet packet;
    passthrough_handler.handle_packet(packet);
}




