import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_CHANNEL, CONF_ID
from . import MCP4728Output, mcp4728_ns

DEPENDENCIES = ["mcp4728"]

MCP4728Channel = mcp4728_ns.class_("MCP4728Channel", output.FloatOutput)
CONF_MCP4728_ID = "mcp4728_id"
CONF_VDD = "vdd"
CONF_GAIN = "gain"

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(MCP4728Channel),
        cv.GenerateID(CONF_MCP4728_ID): cv.use_id(MCP4728Output),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=3),
        cv.Optional(CONF_VDD, default=True): cv.boolean,
        cv.Optional(CONF_GAIN, default=1): cv.int_range(min=1, max=2)
    }
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_MCP4728_ID])
    rhs = paren.create_channel(config[CONF_CHANNEL], config[CONF_VDD], config[CONF_GAIN])
    var = cg.Pvariable(config[CONF_ID], rhs)
    await output.register_output(var, config)