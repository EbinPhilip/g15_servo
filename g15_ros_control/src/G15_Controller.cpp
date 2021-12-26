#include "G15_Controller.h"
#include <rotational_units/Rotational_Units.h>

#include <ros/console.h>

#include <stdexcept>
#include <cmath>

G15_Controller::G15_Controller(const std::string &controller_name,
                            const std::string &serial_port, bool& stop_flag)
    : controller_name_(controller_name),
      serial_port_(serial_port),
      perform_actuator_enable_(false),
      actuator_enabled_status_(false),
      stop_flag_(stop_flag),
      error_status_(false),
      hal_(serial_port),
      servo_controller_(hal_, &error_handler_)
{
    servo_controller_.begin(115200);
    hal_.delayMilliseconds(2000);
}

G15_Controller::~G15_Controller()
{
    if (actuator_enabled_status_)
    {
        try
        {
            disableActuators();
        }
        catch(...)
        {
            ROS_ERROR("%s: unexpected error in destructor", controller_name_.c_str());
        }
    }
}

void G15_Controller::addServo(G15_Actuator_Properties_Ptr actuator)
{
    if (servo_controller_.ping(actuator->servo_id))
    {
        throw std::runtime_error("ping failed: " + actuator->actuator_name +
                                 " ID:" + std::to_string(actuator->servo_id));
    }
    else
    {
        uint16_t cw_limit = servo_controller_.convertToRegisterValue(
            actuator->cw_limit_deg.Value(),POSITION_MAX_DEGREES, POSITION_MAX_REGISTER);
        uint16_t ccw_limit = servo_controller_.convertToRegisterValue(
            actuator->ccw_limit_deg.Value(),POSITION_MAX_DEGREES, POSITION_MAX_REGISTER);
        _handleErrorResponse(servo_controller_.setAngleLimit(actuator->servo_id, cw_limit, ccw_limit));

        servo_map_.insert(std::make_pair(actuator->actuator_name, actuator));
    }
}

void G15_Controller::readState()
{
    uint8_t read_bytes[6] = {0};
    uint16_t value = 0;

    for (auto servo : servo_map_)
    {
        if (stop_flag_ || error_status_)
        {
            return;
        }

        if ( !_checkResponse(servo_controller_.readInstruction(servo.second->servo_id, PRESENT_POSITION_L, read_bytes, 6)) )
        {
            value = read_bytes[3] & 0x01;
            value = (value << 8) | read_bytes[2];
            double speed =
                servo_controller_.convertFromRegisterValue(value, SPEED_MAX_REGISTER, SPEED_MAX_RADIANS);

            value = read_bytes[1] & 0x03;
            value = (value << 8) | read_bytes[0];
            RUnits::Degrees pos_deg = servo_controller_.convertFromRegisterValue(value, POSITION_MAX_REGISTER, POSITION_MAX_DEGREES);
            if (pos_deg.Value()>servo.second->ccw_limit_deg.Value())
            {
                pos_deg = pos_deg.Value() - 360.0;
            }
            double position = static_cast<RUnits::Radians>(pos_deg).Value();

            if ((position - servo.second->state.position) < 0)
            {
                speed*=-1;
            }
            servo.second->state.velocity = speed;
            servo.second->state.position = position;
            
            value = read_bytes[5] & 0x01;
            value = (value << 8) | read_bytes[4];
            servo.second->state.effort =
                servo_controller_.convertFromRegisterValue(value, 1023, 1);
        }
    }
}

void G15_Controller::writeCommand()
{
     if (perform_actuator_enable_)
    {
        _enableActuators();
    }


    for (auto servo : servo_map_)
    {
        if (stop_flag_ || error_status_ || !actuator_enabled_status_)
        {
            return;
        }

        RUnits::Radians pos_rad = servo.second->command.position;
        RUnits::Degrees pos_max = 360.0;
        if (pos_rad.Value()<0)
        {
            pos_rad = static_cast<RUnits::Radians>(pos_max).Value() + pos_rad.Value();
        }
        uint16_t position = servo_controller_.convertToRegisterValue(pos_rad.Value(), 2*M_PI, POSITION_MAX_REGISTER);
        uint16_t velocity = servo_controller_.convertToRegisterValue(fabs(servo.second->command.velocity), SPEED_MAX_RADIANS, SPEED_MAX_REGISTER);

        uint8_t packet[] = 
            {
                (uint8_t)(position & 0xff),
                (uint8_t)(position >> 8),
                (uint8_t)(velocity & 0xff),
                (uint8_t)(velocity >> 8),
            };
        
        _checkResponse(servo_controller_.writeInstruction(servo.second->servo_id, GOAL_POSITION_L, packet, 4));
    }
}

void G15_Controller::enableActuators()
{
    if (!actuator_enabled_status_)
        perform_actuator_enable_ = true;
}

void G15_Controller::disableActuators()
{
    if (actuator_enabled_status_)
    {
        try
        {
            for (auto& servo : servo_map_)
            {   
                _handleErrorResponse(servo_controller_.setTorqueOnOff(servo.second->servo_id, 0));
                _handleErrorResponse(servo_controller_.setLED(servo.second->servo_id, 0));
            }
        }
        catch(const std::exception& e)
        {
            std::string error_msg = e.what();
            ROS_ERROR("%s", error_msg.c_str());
        }
        catch (...)
        {
            ROS_ERROR("%s: unexpected error, disable failed", controller_name_.c_str());
        }    
        
        actuator_enabled_status_ = false;
    }
}

bool G15_Controller::getErrorDetails(std::string& error_msg)
{
    if (error_status_)
        error_msg = error_handler_.getErrorDetails();
    return error_status_;
}

Actuator_Properties_Ptr G15_Controller::getActuator(const std::string& name)
{
    auto it = servo_map_.find(name);
    if (it != servo_map_.end())
    {
        return (it->second);
    }
    else
    {
        return nullptr;
    }
}

void G15_Controller::getActuatorNames(std::vector<std::string>& names)
{
    for (auto it : servo_map_)
    {
        names.push_back(it.first);
    }
}

void G15_Controller::_enableActuators()
{
    if (stop_flag_ || error_status_)
    {
        return;
    }

    if (actuator_enabled_status_)
    {
        perform_actuator_enable_ = false;
        return;
    }

    for (auto& servo : servo_map_)
    {   
        _handleErrorResponse(servo_controller_.setTorqueOnOff(servo.second->servo_id, 1));
        _handleErrorResponse(servo_controller_.setLED(servo.second->servo_id, 1));
    }

    actuator_enabled_status_ = true;
}

uint8_t G15_Controller::_checkResponse(uint8_t error, bool throw_exception)
{
    if (error)
    {
        stop_flag_ = true;
        error_status_ = true;
        error_description_ = error_handler_.getErrorDetails();

        if (throw_exception)
        {
            throw std::runtime_error(error_description_);
        }
    }

    return error;
}

uint8_t G15_Controller::_handleErrorResponse(uint8_t error)
{
    return _checkResponse(error);
}