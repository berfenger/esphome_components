import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

DEPENDENCIES = ["i2c"]
MULTI_CONF = True
CONF_EEPROM = "eeprom"

mcp4728_ns = cg.esphome_ns.namespace("mcp4728")
MCP4728Output = mcp4728_ns.class_("MCP4728Output", cg.Component, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MCP4728Output),
            cv.Optional(CONF_EEPROM, default=False): cv.boolean
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x60))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_EEPROM])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
