"""Number platform for P4 CSI Camera (JPEG quality, RGB gains)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import p4_csi_camera_ns, P4CSICamera

DEPENDENCIES = ["p4_csi_camera"]

CONF_CAMERA_ID = "camera_id"
CONF_TYPE = "type"

P4CSICameraNumber = p4_csi_camera_ns.class_(
    "P4CSICameraNumber", number.Number, cg.Component
)
P4CSICameraQualityNumber = p4_csi_camera_ns.class_(
    "P4CSICameraQualityNumber", number.Number, cg.Component
)

NUMBER_TYPES = {
    "jpeg_quality": 0,
    "red_gain": 1,
    "green_gain": 2,
    "blue_gain": 3,
}

CONFIG_SCHEMA = (
    number.number_schema(
        P4CSICameraNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:tune",
    )
    .extend(
        {
            cv.GenerateID(CONF_CAMERA_ID): cv.use_id(P4CSICamera),
            cv.Optional(CONF_TYPE, default="jpeg_quality"): cv.one_of(
                *NUMBER_TYPES, lower=True
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    ntype = config[CONF_TYPE]
    if ntype == "jpeg_quality":
        min_v, max_v, step = 10, 100, 1
    elif ntype == "green_gain":
        min_v, max_v, step = 0.1, 2.5, 0.05
    else:  # red_gain, blue_gain
        min_v, max_v, step = 0.1, 3.5, 0.05

    var = await number.new_number(
        config,
        min_value=min_v,
        max_value=max_v,
        step=step,
    )
    await cg.register_component(var, config)

    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
    cg.add(var.set_number_type(NUMBER_TYPES[ntype]))
