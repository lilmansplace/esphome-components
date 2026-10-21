from esphome import automation, pins
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_RESET_PIN,
    CONF_TRIGGER_ID,
    CONF_VOLUME,
)
from esphome.core import ID
from esphome.cpp_generator import MockObj, TemplateArgsType
from esphome.types import ConfigType

CODEOWNERS = ["@lilmansplace"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True
DOMAIN = "si4703"

CONF_SI4703_ID = "si4703_id"
CONF_SDIO_PIN = "sdio_pin"
CONF_BAND = "band"
CONF_CHANNEL_SPACING = "channel_spacing"
CONF_DE_EMPHASIS = "de_emphasis"
CONF_REGION = "region"
CONF_ON_SCAN_COMPLETE = "on_scan_complete"
CONF_STATUS_TEXT = "status_text"

si4703_ns = cg.esphome_ns.namespace("si4703")
Si4703Component = si4703_ns.class_("Si4703Component", cg.Component, i2c.I2CDevice)

Band = si4703_ns.enum("Band")
BANDS = {
    "87.5-108MHZ": Band.BAND_87_5_108,
    "76-108MHZ": Band.BAND_76_108,
    "76-90MHZ": Band.BAND_76_90,
}
BAND_LIMITS = {
    "87.5-108MHZ": (87.5, 108.0),
    "76-108MHZ": (76.0, 108.0),
    "76-90MHZ": (76.0, 90.0),
}

ChannelSpacing = si4703_ns.enum("ChannelSpacing")
CHANNEL_SPACINGS = {
    "200KHZ": ChannelSpacing.SPACING_200KHZ,
    "100KHZ": ChannelSpacing.SPACING_100KHZ,
    "50KHZ": ChannelSpacing.SPACING_50KHZ,
}

DE_EMPHASIS = {"75US": False, "50US": True}

# Regional defaults (AN230 table): band, channel spacing, de-emphasis
REGIONS = {
    "AMERICAS": ("87.5-108MHz", "200kHz", "75us"),
    "EUROPE": ("87.5-108MHz", "100kHz", "50us"),
    "AUSTRALIA": ("87.5-108MHz", "200kHz", "50us"),
    "JAPAN": ("76-90MHz", "100kHz", "50us"),
    "JAPAN_WIDE": ("76-108MHz", "100kHz", "50us"),
}

TuneAction = si4703_ns.class_("TuneAction", automation.Action)
SeekAction = si4703_ns.class_("SeekAction", automation.Action)
SetVolumeAction = si4703_ns.class_("SetVolumeAction", automation.Action)
ScanAction = si4703_ns.class_("ScanAction", automation.Action)
ScanCompleteTrigger = si4703_ns.class_(
    "ScanCompleteTrigger", automation.Trigger.template()
)


def _upper_no_space(value):
    return cv.string_strict(value).upper().replace(" ", "")


def _apply_region_defaults(config):
    """Fill band / spacing / de-emphasis from `region` unless set explicitly."""
    if not isinstance(config, dict):
        return config
    config = dict(config)
    region = cv.one_of(*REGIONS, upper=True)(config.get(CONF_REGION, "AMERICAS"))
    config[CONF_REGION] = region
    band, spacing, de_emphasis = REGIONS[region]
    config.setdefault(CONF_BAND, band)
    config.setdefault(CONF_CHANNEL_SPACING, spacing)
    config.setdefault(CONF_DE_EMPHASIS, de_emphasis)
    return config


def _validate_frequency_in_band(config: ConfigType) -> ConfigType:
    lo, hi = BAND_LIMITS[config[CONF_BAND]]
    if not lo <= config[CONF_FREQUENCY] <= hi:
        raise cv.Invalid(
            f"frequency must be within the {config[CONF_BAND]} band ({lo}-{hi} MHz)",
            path=[CONF_FREQUENCY],
        )
    return config


CONFIG_SCHEMA = cv.All(
    _apply_region_defaults,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Si4703Component),
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            # Same GPIO as the I2C bus SDA pin (both need allow_other_uses: true).
            # Held low while RST rises so the chip latches 2-wire (I2C) mode.
            cv.Optional(CONF_SDIO_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_REGION): cv.one_of(*REGIONS, upper=True),
            cv.Required(CONF_BAND): cv.All(_upper_no_space, cv.enum(BANDS)),
            cv.Required(CONF_CHANNEL_SPACING): cv.All(
                _upper_no_space, cv.enum(CHANNEL_SPACINGS)
            ),
            cv.Required(CONF_DE_EMPHASIS): cv.All(
                _upper_no_space, cv.enum(DE_EMPHASIS)
            ),
            cv.Optional(CONF_ON_SCAN_COMPLETE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        ScanCompleteTrigger
                    ),
                }
            ),
            cv.Optional(CONF_FREQUENCY, default=87.5): cv.float_range(
                min=76.0, max=108.0
            ),
            cv.Optional(CONF_VOLUME, default=8): cv.int_range(min=0, max=15),
            # Show the frequency / scan progress in station_name and radio_text until RDS data arrives
            cv.Optional(CONF_STATUS_TEXT, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10)),
    _validate_frequency_in_band,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset_pin))
    if sdio_config := config.get(CONF_SDIO_PIN):
        sdio_pin = await cg.gpio_pin_expression(sdio_config)
        cg.add(var.set_sdio_pin(sdio_pin))

    cg.add(var.set_band(config[CONF_BAND]))
    cg.add(var.set_channel_spacing(config[CONF_CHANNEL_SPACING]))
    cg.add(var.set_de_emphasis_50us(config[CONF_DE_EMPHASIS]))
    cg.add(var.set_initial_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_initial_volume(config[CONF_VOLUME]))
    if config[CONF_STATUS_TEXT]:
        cg.add(var.set_status_text(True))

    for conf in config.get(CONF_ON_SCAN_COMPLETE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


# Actions

TUNE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(Si4703Component),
        cv.Required(CONF_FREQUENCY): cv.templatable(cv.float_range(min=76.0, max=108.0)),
    }
)


@automation.register_action(
    "si4703.tune", TuneAction, TUNE_SCHEMA, synchronous=True
)
async def tune_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_FREQUENCY], args, cg.float_)
    cg.add(var.set_frequency(template_))
    return var


SEEK_SCHEMA = maybe_simple_id({cv.GenerateID(): cv.use_id(Si4703Component)})


@automation.register_action(
    "si4703.seek_up", SeekAction, SEEK_SCHEMA, synchronous=True
)
async def seek_up_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg, True)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "si4703.seek_down", SeekAction, SEEK_SCHEMA, synchronous=True
)
async def seek_down_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg, False)
    await cg.register_parented(var, config[CONF_ID])
    return var


SET_VOLUME_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(Si4703Component),
        cv.Required(CONF_VOLUME): cv.templatable(cv.int_range(min=0, max=15)),
    }
)


@automation.register_action(
    "si4703.set_volume", SetVolumeAction, SET_VOLUME_SCHEMA, synchronous=True
)
async def set_volume_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_VOLUME], args, cg.uint8)
    cg.add(var.set_volume(template_))
    return var


@automation.register_action(
    "si4703.scan", ScanAction, SEEK_SCHEMA, synchronous=True
)
async def scan_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
