import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_CHANNEL, CONF_ID, CONF_GAIN
from . import MCP4728Output, mcp4728_ns

DEPENDENCIES = ["mcp4728"]

MCP4728Channel = mcp4728_ns.class_("MCP4728Channel", output.FloatOutput)
CONF_MCP4728_ID = "mcp4728_id"
CONF_VREF = "vref"

MCP4728Vref = mcp4728_ns.enum("MCP4728Vref")
VREF_OPTIONS = {
    "vdd": MCP4728Vref.MCP4728_VREF_VDD,
    "internal": MCP4728Vref.MCP4728_VREF_INTERNAL_2_8V
}

MCP4728Gain = mcp4728_ns.enum("MCP4728Gain")
GAIN_OPTIONS = {
    "X1": MCP4728Gain.MCP4728_GAIN_X1,
    "X2": MCP4728Gain.MCP4728_GAIN_X2
}

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(MCP4728Channel),
        cv.GenerateID(CONF_MCP4728_ID): cv.use_id(MCP4728Output),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=3),
        cv.Optional(CONF_VREF, default="vdd"): cv.enum(
            VREF_OPTIONS, upper=False
        ),
        cv.Optional(CONF_GAIN, default="X1"): cv.enum(
            GAIN_OPTIONS, upper=True
        ),
    }
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_MCP4728_ID])
    rhs = paren.create_channel(config[CONF_CHANNEL], config[CONF_VREF], config[CONF_GAIN])
    var = cg.Pvariable(config[CONF_ID], rhs)
    await output.register_output(var, config)