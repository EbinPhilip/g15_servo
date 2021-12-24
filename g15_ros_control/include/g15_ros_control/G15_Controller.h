#ifndef __G15_CONTROLLER_H__
#define __G15_CONTROLLER_H__

#include <configurable_control_hw/Actuator_Controller.h>
#include "G15_Actuator_Properties.h"

#include <g15_servo/G15_Servo.h>
#include <g15_linux/Linux_Passthrough_HAL.h>

#include <rotational_units/Rotational_Units.h>

#include <map>
#include <string>

class G15_Controller : public Actuator_Controller
{
public:
    G15_Controller(const std::string& controller_name, const std::string& serial_port);

    void addServo(G15_Actuator_Properties_Ptr actuator);

    virtual void readState() override;
    virtual void writeCommand() override;

    virtual Actuator_Properties_Ptr getActuator(const std::string&) override;
    virtual void getActuatorNames(std::vector<std::string>&) override;

protected:
    std::string controller_name_;
    std::string serial_port_;

    G15_Servo servo_controller_;
    Linux_Passthrough_HAL hal_;

    std::map<std::string, G15_Actuator_Properties_Ptr> servo_map_;
};

#endif