#include <pluginlib/class_list_macros.h>

#include "G15_Actuator_Config_Parser.h"
#include "G15_Actuator_Properties.h"
#include "G15_Controller.h"

#include <memory>
#include <stdexcept>
#include <algorithm>

using namespace XmlRpc;

void G15_Actuator_Config_Parser::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) 
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("actuator config parsing failed!");
    }
    
    for(auto it = config.begin(); it != config.end(); ++it)
    {
        if (it->second.getType() != XmlRpcValue::Type::TypeStruct)
        {
            throw std::runtime_error("actuator config parsing failed!");
        }
    
        std::string actuator_name = it->first;
        uint8_t actuator_id = (uint8_t)static_cast<int>(it->second["servo_id"]);
        std::string actuator_controller = static_cast<std::string>(it->second["controller_name"]);

        G15_Actuator_Properties_Ptr actuator = std::make_shared<G15_Actuator_Properties>
        (actuator_name, actuator_id);
        actuator->ccw_limit_deg = static_cast<double>(it->second["ccw_limit_deg"]);
        actuator->cw_limit_deg = static_cast<double>(it->second["cw_limit_deg"]);
        
        auto controller_it = controller_map->find(actuator_controller);
        if (controller_it == controller_map->end())
        {
            throw std::runtime_error("actuator controller config missing: " + actuator_controller);
        }
        auto controller = std::dynamic_pointer_cast<G15_Controller>(controller_it->second);
        if (!controller)
        {
            throw std::bad_cast();
        }
        controller->addServo(actuator);
    }
}

PLUGINLIB_EXPORT_CLASS(G15_Actuator_Config_Parser, Actuator_Config_Parser)