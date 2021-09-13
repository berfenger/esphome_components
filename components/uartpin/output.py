import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_CHANNEL, CONF_ID
from . import UARTPINComponent, uartpin_ns

DEPENDENCIES = ["uartpin"]

UARTPINChannel = uartpin_ns.class_("UARTPINChannel", output.FloatOutput)
CONF_UARTPIN_ID = "uartpin_id"
CONF_DATA_HIGH = "data_high"
CONF_DATA_LOW = "data_low"

def validate_raw_data(value):
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid(
        "data must either be a string wrapped in quotes or a list of bytes"
    )

CONFIG_SCHEMA = output.BINARY_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(UARTPINChannel),
        cv.GenerateID(CONF_UARTPIN_ID): cv.use_id(UARTPINComponent),
        cv.Required(CONF_DATA_HIGH): validate_raw_data,
        cv.Required(CONF_DATA_LOW): validate_raw_data,
    }
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_UARTPIN_ID])
    datah = config[CONF_DATA_HIGH]
    if isinstance(datah, bytes):
        datah = [HexInt(x) for x in datah]
    datal = config[CONF_DATA_LOW]
    if isinstance(datal, bytes):
        datal = [HexInt(x) for x in datal]
    rhs = paren.create_channel()
    var = cg.Pvariable(config[CONF_ID], rhs)
    cg.add(var.set_data_high(datah))
    cg.add(var.set_data_low(datal))
    await output.register_output(var, config)