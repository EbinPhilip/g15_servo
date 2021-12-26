#ifndef __G15_CONTROLLER_H__
#define __G15_CONTROLLER_H__

#include <configurable_control_hw/Actuator_Controller.h>
#include "G15_Actuator_Properties.h"
#include "G15_ROS_Error_Handler.h"

#include <g15_servo/G15_Servo.h>
#include <g15_linux/Linux_Passthrough_HAL.h>

#include <rotational_units/Rotational_Units.h>

#include <map>
#include <string>

class G15_Controller : public Actuator_Controller
{
public:
    G15_Controller(const std::string& controller_name, const std::string& serial_port, bool& stop_flag);
    ~G15_Controller();

    void addServo(G15_Actuator_Properties_Ptr actuator);

    virtual void readState() override;
    virtual void writeCommand() override;

    virtual void enableActuators() override;
    virtual void disableActuators() override;

    virtual bool getErrorDetails(std::string& error_msg) override;

    virtual Actuator_Properties_Ptr getActuator(const std::string&) override;
    virtual void getActuatorNames(std::vector<std::string>&) override;

protected:
    void _enableActuators();
    uint8_t _checkResponse(uint8_t error, bool throw_exception=false);
    uint8_t _handleErrorResponse(uint8_t error);

    std::string controller_name_;
    std::string serial_port_;

    bool perform_actuator_enable_;
    bool actuator_enabled_status_;

    bool& stop_flag_;
    bool error_status_;
    uint8_t com_error_count_;
    std::string error_description_;

    Linux_Passthrough_HAL hal_;
    G15_ROS_Error_Handler error_handler_;
    G15_Servo servo_controller_;

    std::map<std::string, G15_Actuator_Properties_Ptr> servo_map_;
};

#endif