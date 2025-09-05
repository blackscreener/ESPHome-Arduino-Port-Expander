import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_PIN, CONF_ID, CONF_DEVICE_CLASS, CONF_INVERTED
from . import ArduinoPortExpander, arduino_port_expander_ns

DEPENDENCIES = ["arduino_port_expander"]
CONF_ARDUINO_PORT_EXPANDER = "arduino_port_expander"

ApeBinarySensor = arduino_port_expander_ns.class_("ApeBinarySensor", binary_sensor.BinarySensor)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(ApeBinarySensor),
        cv.Required(CONF_PIN): cv.uint8_t,
        cv.Required(CONF_ARDUINO_PORT_EXPANDER): cv.use_id(ArduinoPortExpander),
        cv.Optional(CONF_DEVICE_CLASS): cv.string,
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_ARDUINO_PORT_EXPANDER])
    var = cg.new_Pvariable(config[CONF_ID], hub, config[CONF_PIN])
    await binary_sensor.register_binary_sensor(var, config)
    
    if CONF_DEVICE_CLASS in config:
        cg.add(var.set_device_class(config[CONF_DEVICE_CLASS]))
    
    if CONF_INVERTED in config:
        cg.add(var.set_inverted(config[CONF_INVERTED]))
    
    cg.add(
        cg.RawExpression(
            f'ESP_LOGI("ape.binary_sensor", "Created binary sensor for pin {config[CONF_PIN]} with invert={config[CONF_INVERTED]}")'
        )
    )