import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_FREQUENCY,
    CONF_VOLUME,
    DEVICE_CLASS_FREQUENCY,
    ENTITY_CATEGORY_CONFIG,
)
from esphome.types import ConfigType

from .. import CONF_SI4703_ID, Si4703Component, si4703_ns

FrequencyNumber = si4703_ns.class_("FrequencyNumber", number.Number)
VolumeNumber = si4703_ns.class_("VolumeNumber", number.Number)
SeekThresholdNumber = si4703_ns.class_("SeekThresholdNumber", number.Number)

CONF_SEEK_THRESHOLD = "seek_threshold"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_FREQUENCY): number.number_schema(
            FrequencyNumber,
            unit_of_measurement="MHz",
            device_class=DEVICE_CLASS_FREQUENCY,
            icon="mdi:radio-fm",
        ),
        cv.Optional(CONF_VOLUME): number.number_schema(
            VolumeNumber,
            icon="mdi:volume-high",
        ),
        cv.Optional(CONF_SEEK_THRESHOLD): number.number_schema(
            SeekThresholdNumber,
            unit_of_measurement="dBµV",
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:tune-variant",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    if frequency_config := config.get(CONF_FREQUENCY):
        # Range and step are narrowed to the configured band at runtime
        n = await number.new_number(
            frequency_config, min_value=76.0, max_value=108.0, step=0.05
        )
        await cg.register_parented(n, config[CONF_SI4703_ID])
        cg.add(parent.set_frequency_number(n))
    if volume_config := config.get(CONF_VOLUME):
        n = await number.new_number(volume_config, min_value=0, max_value=15, step=1)
        await cg.register_parented(n, config[CONF_SI4703_ID])
        cg.add(parent.set_volume_number(n))
    if seek_config := config.get(CONF_SEEK_THRESHOLD):
        n = await number.new_number(seek_config, min_value=0, max_value=127, step=1)
        await cg.register_parented(n, config[CONF_SI4703_ID])
        cg.add(parent.set_seek_threshold_number(n))
