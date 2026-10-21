import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import CONF_SI4703_ID, Si4703Component

DEPENDENCIES = ["si4703"]

CONF_STEREO = "stereo"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_STEREO): binary_sensor.binary_sensor_schema(
            icon="mdi:surround-sound",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    if stereo_config := config.get(CONF_STEREO):
        sens = await binary_sensor.new_binary_sensor(stereo_config)
        cg.add(parent.set_stereo_binary_sensor(sens))
