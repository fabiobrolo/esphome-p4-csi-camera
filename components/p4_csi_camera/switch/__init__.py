"""Switch platform for P4 CSI Camera (vertical/horizontal flip)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import p4_csi_camera_ns, P4CSICamera

DEPENDENCIES = ["p4_csi_camera"]

CONF_CAMERA_ID = "camera_id"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_HORIZONTAL_FLIP = "horizontal_flip"

P4CSICameraFlipSwitch = p4_csi_camera_ns.class_(
    "P4CSICameraFlipSwitch", switch.Switch, cg.Component
)

FLIP_MODE_VERTICAL = 0
FLIP_MODE_HORIZONTAL = 1

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CAMERA_ID): cv.use_id(P4CSICamera),
        cv.Optional(CONF_VERTICAL_FLIP): switch.switch_schema(
            P4CSICameraFlipSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:flip-vertical",
        ),
        cv.Optional(CONF_HORIZONTAL_FLIP): switch.switch_schema(
            P4CSICameraFlipSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:flip-horizontal",
        ),
    }
)


async def to_code(config):
    camera = await cg.get_variable(config[CONF_CAMERA_ID])

    if CONF_VERTICAL_FLIP in config:
        conf = config[CONF_VERTICAL_FLIP]
        var = await switch.new_switch(conf)
        await cg.register_component(var, conf)
        cg.add(var.set_camera(camera))
        cg.add(var.set_flip_mode(FLIP_MODE_VERTICAL))

    if CONF_HORIZONTAL_FLIP in config:
        conf = config[CONF_HORIZONTAL_FLIP]
        var = await switch.new_switch(conf)
        await cg.register_component(var, conf)
        cg.add(var.set_camera(camera))
        cg.add(var.set_flip_mode(FLIP_MODE_HORIZONTAL))
