import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
    CONF_STATE_CLASS,
    ICON_EMPTY
)

DEPENDENCIES = ["i2c"]

max44009_ns = cg.esphome_ns.namespace("max44009")
MAX44009Sensor = max44009_ns.class_(
    "MAX44009Sensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (sensor.sensor_schema(
        UNIT_LUX, ICON_EMPTY, 0, DEVICE_CLASS_ILLUMINANCE
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(MAX44009Sensor),
            cv.Optional(CONF_STATE_CLASS, default=STATE_CLASS_MEASUREMENT): sensor.validate_state_class
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x4A))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    await sensor.register_sensor(var, config)
