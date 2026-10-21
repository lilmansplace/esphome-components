import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_FREQUENCY,
    DEVICE_CLASS_FREQUENCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_SIGNAL,
    STATE_CLASS_MEASUREMENT,
)
from esphome.types import ConfigType

from . import CONF_SI4703_ID, Si4703Component

DEPENDENCIES = ["si4703"]

CONF_RSSI = "rssi"
UNIT_MEGAHERTZ = "MHz"
UNIT_DECIBEL_MICROVOLT = "dBµV"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_MEGAHERTZ,
            device_class=DEVICE_CLASS_FREQUENCY,
            accuracy_decimals=2,
            icon="mdi:radio",
        ),
        cv.Optional(CONF_RSSI): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MICROVOLT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon=ICON_SIGNAL,
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    if frequency_config := config.get(CONF_FREQUENCY):
        sens = await sensor.new_sensor(frequency_config)
        cg.add(parent.set_frequency_sensor(sens))
    if rssi_config := config.get(CONF_RSSI):
        sens = await sensor.new_sensor(rssi_config)
        cg.add(parent.set_rssi_sensor(sens))
