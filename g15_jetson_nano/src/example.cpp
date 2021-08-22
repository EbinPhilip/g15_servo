#include <signal.h>

#include "Jetson_Nano_HAL.h"
#include "g15_servo/G15_Servo.h"

bool stop_execution = false;

void signalHandler(int s)
{
    stop_execution = true;
}

int main(void)
{
    signal(SIGINT, signalHandler);

    Jetson_Nano_HAL hal("/dev/ttyUSB0", 79);
    G15_Servo servo(hal);

    servo.begin(19200);

    while (!stop_execution)
    {
        servo.setLED(1, 1);
        hal.delayMilliseconds(1000);
        servo.setLED(1,0);
        hal.delayMilliseconds(1000);
    }

    servo.end();
}
