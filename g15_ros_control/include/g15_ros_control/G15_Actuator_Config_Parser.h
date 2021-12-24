#ifndef __G15_CONFIG_PARSER_H__
#define __G15_CONFIG_PARSER_H__

#include <configurable_control_hw/Actuator_Config_Parser.h>

#include <cstdint>
#include <string>

class G15_Actuator_Config_Parser : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
};

#endif