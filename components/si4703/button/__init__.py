import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.types import ConfigType

from .. import CONF_SI4703_ID, Si4703Component, si4703_ns

SeekUpButton = si4703_ns.class_("SeekUpButton", button.Button)
SeekDownButton = si4703_ns.class_("SeekDownButton", button.Button)
ScanButton = si4703_ns.class_("ScanButton", button.Button)

CONF_SEEK_UP = "seek_up"
CONF_SEEK_DOWN = "seek_down"
CONF_SCAN = "scan"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SI4703_ID): cv.use_id(Si4703Component),
        cv.Optional(CONF_SEEK_UP): button.button_schema(
            SeekUpButton, icon="mdi:fast-forward"
        ),
        cv.Optional(CONF_SEEK_DOWN): button.button_schema(
            SeekDownButton, icon="mdi:rewind"
        ),
        cv.Optional(CONF_SCAN): button.button_schema(
            ScanButton, icon="mdi:radar"
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_SI4703_ID])
    for key, setter in (
        (CONF_SEEK_UP, parent.set_seek_up_button),
        (CONF_SEEK_DOWN, parent.set_seek_down_button),
        (CONF_SCAN, parent.set_scan_button),
    ):
        if conf := config.get(key):
            b = await button.new_button(conf)
            await cg.register_parented(b, config[CONF_SI4703_ID])
            cg.add(setter(b))
