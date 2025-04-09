import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import media_player
from esphome.const import (
    CONF_ID,
    CONF_NAME,
)
from . import si4703_ns, Si4703Component

CONF_SI4703_ID = "si4703_id"

Si4703MediaPlayer = si4703_ns.class_(
    "Si4703MediaPlayer", media_player.MediaPlayer, cg.Component
)

CONFIG_SCHEMA = media_player.MEDIA_PLAYER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(Si4703MediaPlayer),
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        # Add specific media_player config options here if needed
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NAME])
    await cg.register_component(var, config)
    await media_player.register_media_player(var, config)

    parent = await cg.get_variable(config[CONF_SI4703_ID])
    cg.add(var.set_parent(parent))

    # You might need to tell the C++ media player about features supported
    # cg.add(var.set_supported_features(...)) 