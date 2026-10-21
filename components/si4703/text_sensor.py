import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import CONF_SI4703_ID, Si4703Component

DEPENDENCIES = ["si4703"]

CONF_STATION_NAME = "station_name"
CONF_RADIO_TEXT = "radio_text"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_STATION_NAME): text_sensor.text_sensor_schema(
            icon="mdi:radio-tower",
        ),
        cv.Optional(CONF_RADIO_TEXT): text_sensor.text_sensor_schema(
            icon="mdi:message-text",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    if station_name_config := config.get(CONF_STATION_NAME):
        sens = await text_sensor.new_text_sensor(station_name_config)
        cg.add(parent.set_station_name_text_sensor(sens))
    if radio_text_config := config.get(CONF_RADIO_TEXT):
        sens = await text_sensor.new_text_sensor(radio_text_config)
        cg.add(parent.set_radio_text_text_sensor(sens))
