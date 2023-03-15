import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN, CONF_MODE
from esphome import automation

pinscan_ns = cg.esphome_ns.namespace("pinscan")

PinscanComponent = pinscan_ns.class_("Pinscan", cg.Component, cg.PollingComponent)
SetInputAction = pinscan_ns.class_("SetInputAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PinscanComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)


@automation.register_action(
    "pinscan.set_input",
    SetInputAction,
    automation.maybe_simple_id(
        cv.Schema(
            {
                cv.Required(CONF_ID): cv.use_id(PinscanComponent),
                cv.Required(CONF_PIN): cv.templatable(cv.int_),
                cv.Required(CONF_MODE): cv.templatable(cv.int_),
            }
        )
    ),
)
async def pinscan_set_input_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = await cg.templatable(config[CONF_PIN], args, cg.int_)
    cg.add(var.set_pin(template_))
    template2_ = await cg.templatable(config[CONF_MODE], args, cg.int_)
    cg.add(var.set_mode(template2_))
    return var
