import esphome.codegen as cg
from esphome.components import cover
import esphome.config_validation as cv
from esphome.types import ConfigType

mock_speed_cover_ns = cg.esphome_ns.namespace("mock_speed_cover")
MockSpeedCover = mock_speed_cover_ns.class_("MockSpeedCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.cover_schema(MockSpeedCover).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    cover.set_speeds(var, ["slow", "fast"])
