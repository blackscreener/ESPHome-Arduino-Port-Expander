import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_PIN, CONF_ID, CONF_INTERLOCK
from . import ArduinoPortExpander, arduino_port_expander_ns

DEPENDENCIES = ["arduino_port_expander"]
CONF_ARDUINO_PORT_EXPANDER = "arduino_port_expander"
CONF_INTERLOCK_WAIT_TIME = "interlock_wait_time"

ApeSwitch = arduino_port_expander_ns.class_("ApeSwitch", switch.Switch)

CONFIG_SCHEMA = switch.switch_schema(ApeSwitch).extend({
    cv.Required(CONF_PIN): cv.uint8_t,
    cv.Required(CONF_ARDUINO_PORT_EXPANDER): cv.use_id(ArduinoPortExpander),
    cv.Optional(CONF_INTERLOCK): cv.ensure_list(cv.use_id(ApeSwitch)),
    cv.Optional(CONF_INTERLOCK_WAIT_TIME, default="0ms"): cv.positive_time_period_milliseconds,
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_ARDUINO_PORT_EXPANDER])
    var = cg.new_Pvariable(config[CONF_ID], hub, config[CONF_PIN])
    await switch.register_switch(var, config)
    
    # Explicitly register the switch with the hub
    cg.add(hub.register_switch(var))
    
    if CONF_INTERLOCK in config:
        interlock_switches = []
        for other_id in config[CONF_INTERLOCK]:
            other = await cg.get_variable(other_id)
            interlock_switches.append(other)
        cg.add(var.set_interlock_switches(interlock_switches))
    
    if CONF_INTERLOCK_WAIT_TIME in config:
        cg.add(var.set_interlock_wait_time(config[CONF_INTERLOCK_WAIT_TIME]))