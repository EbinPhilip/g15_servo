# G15 Servo

This package contains the firmware and ROS control implementations for the [G15 Servo](https://www.cytron.io/p-cube-servo). 

- g15_servo: protocol implementation and HAL interface.
- g15_arduino: uses the protocol implementation over Arduino HAL.
- g15_linux: linux implementation of the G15 Hal. Requires arduino as passthrough.
- g15_ros_control: implementation of Actuator and Actuator Controller interfaces from [Configurable Control Hardware](https://github.com/EbinPhilip/configurable_control_hw/). 