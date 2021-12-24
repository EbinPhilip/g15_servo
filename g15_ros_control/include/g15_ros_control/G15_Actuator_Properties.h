#ifndef __G15_ACTUATOR_PROPERTIES_H__
#define __G15_ACTUATOR_PROPERTIES_H__

#include <configurable_control_hw/Actuator_Properties.h>
#include <rotational_units/Rotational_Units.h>

#include <memory>
#include <string>

const std::string G15_ACTUATOR_NAME_STR = "g15_servo";

struct G15_Actuator_Properties : public Actuator_Properties
{
    G15_Actuator_Properties(const std::string& name, uint8_t id_number)
        : servo_id(id_number),
          last_sent(0),
          error_code(0),
          error_count(0)
    {
        actuator_name =  name;
        actuator_type = G15_ACTUATOR_NAME_STR;
    }

    uint8_t servo_id;
    RUnits::Degrees cw_limit_deg;
    RUnits::Degrees ccw_limit_deg;

    unsigned long last_sent;
    bool error_status;
    uint8_t error_code;
    uint8_t error_count;
};

typedef std::shared_ptr<G15_Actuator_Properties> G15_Actuator_Properties_Ptr;

#endif