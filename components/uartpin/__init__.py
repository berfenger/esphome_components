import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_DELAY

DEPENDENCIES = ["uart"]
MULTI_CONF = True
CONF_INIT_DATA = "init_data"

uartpin_ns = cg.esphome_ns.namespace("uartpin")
UARTPINComponent = uartpin_ns.class_("UARTPINComponent", cg.Component, uart.UARTDevice)

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

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UARTPINComponent),
            cv.Optional(CONF_DELAY): cv.update_interval,
            cv.Optional(CONF_INIT_DATA): validate_raw_data,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    if CONF_DELAY in config:
        cg.add(var.set_init_delay(config[CONF_DELAY]))
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if CONF_INIT_DATA in config:
        idata = config[CONF_INIT_DATA]
        if idata and isinstance(idata, bytes):
            idata = [HexInt(x) for x in idata]
        cg.add(var.set_init_data(idata))
