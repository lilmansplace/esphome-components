import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor, text_sensor
from esphome.const import CONF_ID, ICON_SIGNAL

CODEOWNERS = ["@lilmansplace"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_RESET_PIN = "reset_pin"
CONF_SDIO_PIN = "sdio_pin"
CONF_START_FREQUENCY = "frequency"
CONF_START_VOLUME = "volume"
CONF_DE_EMPHASIS = "de_emphasis"
CONF_SEEK_THRESHOLD = "seek_threshold"
CONF_FREQUENCY_SENSOR = "frequency_sensor"
CONF_RSSI_SENSOR = "rssi_sensor"
CONF_STATION_NAME = "station_name"
CONF_RADIO_TEXT = "radio_text"

si4703_ns = cg.esphome_ns.namespace("si4703")
Si4703Component = si4703_ns.class_("Si4703Component", cg.Component, i2c.I2CDevice)

DE_EMPHASIS = {"75us": False, "50us": True}  # 75us = Americas, 50us = Europe/Asia

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Si4703Component),
            # RST pin of the Si4703. Required: the chip must be reset with SDIO low
            # to come up in 2-wire (I2C) mode.
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            # The same GPIO as i2c "sda". Used only during reset to hold SDIO low.
            # Both places need "allow_other_uses: true".
            cv.Optional(CONF_SDIO_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_START_FREQUENCY, default=87.5): cv.float_range(
                min=87.5, max=108.0
            ),
            cv.Optional(CONF_START_VOLUME, default=8): cv.int_range(min=0, max=15),
            cv.Optional(CONF_DE_EMPHASIS, default="75us"): cv.enum(DE_EMPHASIS),
            cv.Optional(CONF_SEEK_THRESHOLD, default=25): cv.int_range(
                min=0, max=127
            ),
            cv.Optional(CONF_FREQUENCY_SENSOR): sensor.sensor_schema(
                unit_of_measurement="MHz",
                icon="mdi:radio",
                accuracy_decimals=1,
            ),
            cv.Optional(CONF_RSSI_SENSOR): sensor.sensor_schema(
                unit_of_measurement="dBµV",
                icon=ICON_SIGNAL,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_STATION_NAME): text_sensor.text_sensor_schema(
                icon="mdi:radio-tower"
            ),
            cv.Optional(CONF_RADIO_TEXT): text_sensor.text_sensor_schema(
                icon="mdi:message-text"
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset_pin))

    if CONF_SDIO_PIN in config:
        sdio_pin = await cg.gpio_pin_expression(config[CONF_SDIO_PIN])
        cg.add(var.set_sdio_pin(sdio_pin))

    cg.add(var.set_initial_frequency(config[CONF_START_FREQUENCY]))
    cg.add(var.set_initial_volume(config[CONF_START_VOLUME]))
    cg.add(var.set_de_emphasis_50us(config[CONF_DE_EMPHASIS]))
    cg.add(var.set_seek_threshold(config[CONF_SEEK_THRESHOLD]))

    if CONF_FREQUENCY_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_FREQUENCY_SENSOR])
        cg.add(var.set_frequency_sensor(sens))
    if CONF_RSSI_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_RSSI_SENSOR])
        cg.add(var.set_rssi_sensor(sens))
    if CONF_STATION_NAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATION_NAME])
        cg.add(var.set_station_name_sensor(sens))
    if CONF_RADIO_TEXT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_RADIO_TEXT])
        cg.add(var.set_radio_text_sensor(sens))
