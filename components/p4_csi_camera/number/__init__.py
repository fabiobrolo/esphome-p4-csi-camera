"""Number platform for P4 CSI Camera (JPEG quality)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import p4_csi_camera_ns, P4CSICamera

DEPENDENCIES = ["p4_csi_camera"]

CONF_CAMERA_ID = "camera_id"

P4CSICameraQualityNumber = p4_csi_camera_ns.class_(
    "P4CSICameraQualityNumber", number.Number, cg.Component
)

CONFIG_SCHEMA = (
    number.number_schema(
        P4CSICameraQualityNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:quality-high",
    )
    .extend(
        {
            cv.GenerateID(CONF_CAMERA_ID): cv.use_id(P4CSICamera),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await number.new_number(
        config,
        min_value=10,
        max_value=100,
        step=1,
    )
    await cg.register_component(var, config)

    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
