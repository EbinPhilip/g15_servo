#!/bin/bash

g15_servo_path="./.."
SCRIPT_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ "$1" != "" ]; then
    g15_servo_path="$1"
fi

cp "$g15_servo_path/g15_servo/src/G15_Servo.cpp" "$SCRIPT_PATH"
cp $g15_servo_path"/g15_servo/include/g15_servo/G15_HAL.h" "$SCRIPT_PATH"
cp $g15_servo_path"/g15_servo/include/g15_servo/G15_Servo.h" "$SCRIPT_PATH"
cp $g15_servo_path"/g15_servo/include/g15_servo/G15_Constants.h" "$SCRIPT_PATH"
cp $g15_servo_path"/g15_servo/include/g15_servo/G15_Error_Handler.h" "$SCRIPT_PATH"
cp $g15_servo_path"/g15_servo/include/g15_servo/G15_Error_Handler_Utils.h" "$SCRIPT_PATH"




