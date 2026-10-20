"""Switch platform for P4 CSI Camera (flip, color correction, AWB)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import p4_csi_camera_ns, P4CSICamera

DEPENDENCIES = ["p4_csi_camera"]

CONF_CAMERA_ID = "camera_id"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_HORIZONTAL_FLIP = "horizontal_flip"
CONF_COLOR_CORRECTION = "color_correction"
CONF_AUTO_WHITE_BALANCE = "auto_white_balance"

P4CSICameraSwitch = p4_csi_camera_ns.class_(
    "P4CSICameraSwitch", switch.Switch, cg.Component
)
P4CSICameraFlipSwitch = p4_csi_camera_ns.class_(
    "P4CSICameraFlipSwitch", switch.Switch, cg.Component
)

SWITCH_MODE_VERTICAL = 0
SWITCH_MODE_HORIZONTAL = 1
SWITCH_MODE_COLOR_CORRECTION = 2
SWITCH_MODE_AUTO_WHITE_BALANCE = 3

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CAMERA_ID): cv.use_id(P4CSICamera),
        cv.Optional(CONF_VERTICAL_FLIP): switch.switch_schema(
            P4CSICameraSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:flip-vertical",
        ),
        cv.Optional(CONF_HORIZONTAL_FLIP): switch.switch_schema(
            P4CSICameraSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:flip-horizontal",
        ),
        cv.Optional(CONF_COLOR_CORRECTION): switch.switch_schema(
            P4CSICameraSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:palette",
        ),
        cv.Optional(CONF_AUTO_WHITE_BALANCE): switch.switch_schema(
            P4CSICameraSwitch,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:white-balance-auto",
        ),
    }
)


async def to_code(config):
    camera = await cg.get_variable(config[CONF_CAMERA_ID])

    for key, mode in [
        (CONF_VERTICAL_FLIP, SWITCH_MODE_VERTICAL),
        (CONF_HORIZONTAL_FLIP, SWITCH_MODE_HORIZONTAL),
        (CONF_COLOR_CORRECTION, SWITCH_MODE_COLOR_CORRECTION),
        (CONF_AUTO_WHITE_BALANCE, SWITCH_MODE_AUTO_WHITE_BALANCE),
    ]:
        if key in config:
            conf = config[key]
            var = await switch.new_switch(conf)
            await cg.register_component(var, conf)
            cg.add(var.set_camera(camera))
            cg.add(var.set_switch_mode(mode))
