import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c

DEPENDENCIES = ["i2c"]
CODEOWNERS = ["@esphome"]

arduino_port_expander_ns = cg.esphome_ns.namespace("arduino_port_expander")
ArduinoPortExpander = arduino_port_expander_ns.class_(
    "ArduinoPortExpander", cg.Component, i2c.I2CDevice
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ArduinoPortExpander),
        cv.Optional("vref_default", default=False): cv.boolean,
    }
).extend(i2c.i2c_device_schema(0x08))

async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    if "vref_default" in config:
        cg.add(var.set_vref_default(config["vref_default"]))