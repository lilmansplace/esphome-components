import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG
from esphome.types import ConfigType

from .. import CONF_SI4703_ID, Si4703Component, si4703_ns

MuteSwitch = si4703_ns.class_("MuteSwitch", switch.Switch)
MonoSwitch = si4703_ns.class_("MonoSwitch", switch.Switch)

CONF_MUTE = "mute"
CONF_MONO = "mono"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_MUTE): switch.switch_schema(
            MuteSwitch,
            icon="mdi:volume-off",
        ),
        cv.Optional(CONF_MONO): switch.switch_schema(
            MonoSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:speaker",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    if mute_config := config.get(CONF_MUTE):
        s = await switch.new_switch(mute_config)
        await cg.register_parented(s, config[CONF_SI4703_ID])
        cg.add(parent.set_mute_switch(s))
    if mono_config := config.get(CONF_MONO):
        s = await switch.new_switch(mono_config)
        await cg.register_parented(s, config[CONF_SI4703_ID])
        cg.add(parent.set_mono_switch(s))
