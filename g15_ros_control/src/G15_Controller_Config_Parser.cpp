#include <pluginlib/class_list_macros.h>

#include "G15_Controller_Config_Parser.h"
#include <configurable_control_hw/Actuator_Controller.h>
#include "G15_Controller.h"

#include <memory>
#include <stdexcept>
#include <algorithm>

using namespace XmlRpc;

void G15_Controller_Config_Parser::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("controller config parsing failed!");
    }
    
    for(auto it = config.begin(); it != config.end(); ++it)
    {
        if (it->second.getType() != XmlRpcValue::Type::TypeStruct)
        {
            throw std::runtime_error("actuator config parsing failed!");
        }
    
        std::string controller_name = it->first;
        uint8_t controller_id = (uint8_t)static_cast<int>(it->second["actuator_id"]);
        std::string controller_port = static_cast<std::string>(it->second["port"]);
        unsigned int controller_baud_rate = (unsigned int)static_cast<int>(it->second["baud_rate"]);

        Actuator_Controller_Ptr controller_ptr = std::make_shared<G15_Controller>(controller_name, controller_port, *stop_flag_ptr_);
        controller_map->insert(std::make_pair(controller_name, controller_ptr));
    }
}

PLUGINLIB_EXPORT_CLASS(G15_Controller_Config_Parser, Actuator_Config_Parser)