import esphome.codegen as cg
from esphome.components import button, remote_base
import esphome.config_validation as cv
from esphome.const import CONF_CHANNELS, ENTITY_CATEGORY_CONFIG
import esphome.final_validate as fv
from esphome.types import ConfigType

from .. import CONF_LEGACY, CONF_SERIAL, DOMAIN, KeeloqRemote, _channels, keeloq_ns

CONF_KEELOQ_ID = "keeloq_id"
CONF_LEARN = "learn"
CONF_TAKE_OVER = "take_over"

LearnButton = keeloq_ns.class_("LearnButton", button.Button)
TakeOverButton = keeloq_ns.class_("TakeOverButton", button.Button)

CONFIG_SCHEMA = cv.All(
    {
        cv.GenerateID(CONF_KEELOQ_ID): cv.use_id(KeeloqRemote),
        cv.Optional(CONF_LEARN): button.button_schema(
            LearnButton, entity_category=ENTITY_CATEGORY_CONFIG
        )
        .extend(
            {
                cv.Optional(CONF_CHANNELS): _channels,
                cv.Optional(CONF_LEGACY, default=False): cv.boolean,
            }
        )
        .extend(remote_base.REMOTE_TRANSMITTABLE_SCHEMA),
        cv.Optional(CONF_TAKE_OVER): button.button_schema(
            TakeOverButton, entity_category=ENTITY_CATEGORY_CONFIG
        ),
    },
    cv.has_at_least_one_key(CONF_LEARN, CONF_TAKE_OVER),
)


# A remote is either ESPHome's own sender (learn) or a copy of a hand remote (take_over), not both.
def _learn_or_take_over(config: ConfigType) -> ConfigType:
    remote_id = config[CONF_KEELOQ_ID]
    if CONF_LEARN in config and CONF_SERIAL not in _remote_config(remote_id):
        raise cv.Invalid("learn needs a remote with serial", [CONF_LEARN])
    modes = (
        fv.full_config.get()
        .data.setdefault(DOMAIN, {})
        .setdefault(str(remote_id), set())
    )
    modes.update(key for key in (CONF_LEARN, CONF_TAKE_OVER) if key in config)
    if len(modes) > 1:
        raise cv.Invalid(
            f"{remote_id} has both learn and take_over; a remote is either paired as its own sender "
            "or takes over a hand remote"
        )
    return config


def _remote_config(remote_id: object) -> ConfigType:
    full = fv.full_config.get()
    return full.get_config_for_path(full.get_path_for_id(remote_id)[:-1])


FINAL_VALIDATE_SCHEMA = _learn_or_take_over


async def to_code(config: ConfigType) -> None:
    if (conf := config.get(CONF_LEARN)) is not None:
        var = await button.new_button(conf)
        await cg.register_parented(var, config[CONF_KEELOQ_ID])
        await remote_base.register_transmittable(var, conf)
        if (channels := conf.get(CONF_CHANNELS)) is not None:
            cg.add(var.set_channels(channels))
        cg.add(var.set_legacy(conf[CONF_LEGACY]))
    if (conf := config.get(CONF_TAKE_OVER)) is not None:
        var = await button.new_button(conf)
        await cg.register_parented(var, config[CONF_KEELOQ_ID])
