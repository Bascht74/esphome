import logging
from typing import Any

from esphome import automation
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import mqtt, web_server
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_MQTT_ID,
    CONF_MQTT_JSON_STATE_PAYLOAD,
    CONF_ON_IDLE,
    CONF_ON_OPEN,
    CONF_POSITION,
    CONF_POSITION_COMMAND_TOPIC,
    CONF_POSITION_STATE_TOPIC,
    CONF_SPEED,
    CONF_STATE,
    CONF_STOP,
    CONF_TILT,
    CONF_TILT_COMMAND_TOPIC,
    CONF_TILT_STATE_TOPIC,
    CONF_TRIGGER_ID,
    CONF_WEB_SERVER,
    DEVICE_CLASS_AWNING,
    DEVICE_CLASS_BLIND,
    DEVICE_CLASS_CURTAIN,
    DEVICE_CLASS_DAMPER,
    DEVICE_CLASS_DOOR,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_GARAGE,
    DEVICE_CLASS_GATE,
    DEVICE_CLASS_SHADE,
    DEVICE_CLASS_SHUTTER,
    DEVICE_CLASS_WINDOW,
)
from esphome.core import CORE, ID, CoroPriority, coroutine_with_priority
from esphome.core.entity_helpers import (
    entity_duplicate_validator,
    queue_entity_register,
    setup_device_class,
    setup_entity,
)
from esphome.cpp_generator import MockObj, MockObjClass
from esphome.types import ConfigType, SafeExpType, TemplateArgsType

IS_PLATFORM_COMPONENT = True

CODEOWNERS = ["@esphome/core"]
DEVICE_CLASSES = [
    DEVICE_CLASS_AWNING,
    DEVICE_CLASS_BLIND,
    DEVICE_CLASS_CURTAIN,
    DEVICE_CLASS_DAMPER,
    DEVICE_CLASS_DOOR,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_GARAGE,
    DEVICE_CLASS_GATE,
    DEVICE_CLASS_SHADE,
    DEVICE_CLASS_SHUTTER,
    DEVICE_CLASS_WINDOW,
]
DOMAIN = "cover"

_LOGGER = logging.getLogger(__name__)

cover_ns = cg.esphome_ns.namespace("cover")

Cover = cover_ns.class_("Cover", cg.EntityBase)

COVER_OPEN = cover_ns.COVER_OPEN
COVER_CLOSED = cover_ns.COVER_CLOSED

COVER_STATES = {
    "OPEN": COVER_OPEN,
    "CLOSED": COVER_CLOSED,
}
validate_cover_state = cv.enum(COVER_STATES, upper=True)

CoverOperation = cover_ns.enum("CoverOperation")
COVER_OPERATIONS = CoverOperation.options(
    "IDLE", "OPENING", "CLOSING", prefix="COVER_OPERATION_"
)
validate_cover_operation = cv.enum(COVER_OPERATIONS, upper=True)

# Actions
CoverOpenedTrigger = cover_ns.class_(
    "CoverOpenedTrigger", automation.Trigger.template()
)
CoverClosedTrigger = cover_ns.class_(
    "CoverClosedTrigger", automation.Trigger.template()
)
CoverTrigger = cover_ns.class_("CoverTrigger", automation.Trigger.template())

# Cover-specific constants
CONF_ON_CLOSED = "on_closed"
CONF_ON_OPENED = "on_opened"
CONF_ON_OPENING = "on_opening"
CONF_ON_CLOSING = "on_closing"
CONF_STOP_TILT = "stop_tilt"

TRIGGERS = {
    CONF_ON_OPEN: CoverOpenedTrigger,  # Deprecated, use on_opened
    CONF_ON_OPENED: CoverOpenedTrigger,
    CONF_ON_CLOSED: CoverClosedTrigger,
    CONF_ON_CLOSING: CoverTrigger.template(CoverOperation.COVER_OPERATION_CLOSING),
    CONF_ON_OPENING: CoverTrigger.template(CoverOperation.COVER_OPERATION_OPENING),
    CONF_ON_IDLE: CoverTrigger.template(CoverOperation.COVER_OPERATION_IDLE),
}


_COVER_SCHEMA = (
    cv.ENTITY_BASE_SCHEMA.extend(web_server.WEBSERVER_SORTING_SCHEMA)
    .extend(cv.MQTT_COMMAND_COMPONENT_SCHEMA)
    .extend(
        {
            cv.OnlyWith(CONF_MQTT_ID, "mqtt"): cv.declare_id(mqtt.MQTTCoverComponent),
            cv.Optional(CONF_MQTT_JSON_STATE_PAYLOAD): cv.All(
                cv.requires_component("mqtt"), cv.boolean
            ),
            cv.Optional(
                CONF_DEVICE_CLASS, visibility=cv.Visibility.ADVANCED
            ): cv.one_of(*DEVICE_CLASSES, lower=True),
            cv.Optional(CONF_POSITION_COMMAND_TOPIC): cv.All(
                cv.requires_component("mqtt"), cv.subscribe_topic
            ),
            cv.Optional(CONF_POSITION_STATE_TOPIC): cv.All(
                cv.requires_component("mqtt"), cv.subscribe_topic
            ),
            cv.Optional(CONF_TILT_COMMAND_TOPIC): cv.All(
                cv.requires_component("mqtt"), cv.subscribe_topic
            ),
            cv.Optional(CONF_TILT_STATE_TOPIC): cv.All(
                cv.requires_component("mqtt"), cv.subscribe_topic
            ),
            **{
                cv.Optional(conf): automation.validate_automation(
                    {
                        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(trigger_class),
                    }
                )
                for conf, trigger_class in TRIGGERS.items()
            },
        }
    )
)


_COVER_SCHEMA.add_extra(entity_duplicate_validator("cover"))


def _validate_mqtt_state_topics(config: ConfigType) -> ConfigType:
    if config.get(CONF_MQTT_JSON_STATE_PAYLOAD):
        if CONF_POSITION_STATE_TOPIC in config:
            raise cv.Invalid(
                f"'{CONF_POSITION_STATE_TOPIC}' cannot be used with '{CONF_MQTT_JSON_STATE_PAYLOAD}: true'"
            )
        if CONF_TILT_STATE_TOPIC in config:
            raise cv.Invalid(
                f"'{CONF_TILT_STATE_TOPIC}' cannot be used with '{CONF_MQTT_JSON_STATE_PAYLOAD}: true'"
            )
    return config


_COVER_SCHEMA.add_extra(_validate_mqtt_state_topics)


def cover_schema(
    class_: MockObjClass,
    *,
    device_class: str = cv.UNDEFINED,
    entity_category: str = cv.UNDEFINED,
    icon: str = cv.UNDEFINED,
) -> cv.Schema:
    schema = {
        cv.GenerateID(): cv.declare_id(class_),
    }

    for key, default, validator in [
        (CONF_DEVICE_CLASS, device_class, cv.one_of(*DEVICE_CLASSES, lower=True)),
        (CONF_ENTITY_CATEGORY, entity_category, cv.entity_category),
        (CONF_ICON, icon, cv.icon),
    ]:
        if default is not cv.UNDEFINED:
            schema[cv.Optional(key, default=default)] = validator

    return _COVER_SCHEMA.extend(schema)


_SPEEDS_SCHEMA = cv.All(
    cv.ensure_list(cv.All(cv.string_strict, cv.Length(min=1))),
    cv.Length(min=1),
)


def validate_speeds(value: Any) -> list[str]:
    """Validate a platform's speed names; no speed at all means the cover's default speed."""
    value = _SPEEDS_SCHEMA(value)
    errors = [
        cv.Invalid(
            f"Found duplicate speed '{speed}'. Speeds must have unique names.", [i]
        )
        for i, speed in enumerate(value)
        if speed in value[:i]
    ]
    if errors:
        raise cv.MultipleInvalid(errors)
    return value


def set_speeds(var: MockObj, speeds: list[str]) -> None:
    """Point the cover at a shared flash table of speed names."""
    if speeds:
        cg.add_define("USE_COVER_SPEED")
        table = cg.shared_progmem_array("cover_speeds", cg.const_char_ptr, speeds)
        cg.add(var.set_supported_speeds_static(table, len(speeds)))


@setup_entity("cover")
async def setup_cover_core_(var: MockObj, config: ConfigType) -> None:
    setup_device_class(config)

    if CONF_ON_OPEN in config:
        _LOGGER.warning(
            "'on_open' is deprecated, use 'on_opened'. Will be removed in 2026.8.0"
        )
    for trigger_conf in TRIGGERS:
        for conf in config.get(trigger_conf, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if (mqtt_id := config.get(CONF_MQTT_ID)) is not None:
        mqtt_ = cg.new_Pvariable(mqtt_id, var)
        await mqtt.register_mqtt_component(mqtt_, config)

        if (position_state_topic := config.get(CONF_POSITION_STATE_TOPIC)) is not None:
            cg.add(mqtt_.set_custom_position_state_topic(position_state_topic))
        if (
            position_command_topic := config.get(CONF_POSITION_COMMAND_TOPIC)
        ) is not None:
            cg.add(mqtt_.set_custom_position_command_topic(position_command_topic))
        if config.get(CONF_MQTT_JSON_STATE_PAYLOAD):
            cg.add_define("USE_MQTT_COVER_JSON")
            cg.add(mqtt_.set_use_json_format(True))
        if (tilt_state_topic := config.get(CONF_TILT_STATE_TOPIC)) is not None:
            cg.add(mqtt_.set_custom_tilt_state_topic(tilt_state_topic))
        if (tilt_command_topic := config.get(CONF_TILT_COMMAND_TOPIC)) is not None:
            cg.add(mqtt_.set_custom_tilt_command_topic(tilt_command_topic))

    if web_server_config := config.get(CONF_WEB_SERVER):
        await web_server.add_entity_config(var, web_server_config)


async def register_cover(var: MockObj, config: ConfigType) -> None:
    if not CORE.has_id(config[CONF_ID]):
        var = cg.Pvariable(config[CONF_ID], var)
    queue_entity_register("cover", config)
    CORE.register_platform_component("cover", var)
    await setup_cover_core_(var, config)


async def new_cover(config: ConfigType, *args: SafeExpType) -> MockObj:
    var = cg.new_Pvariable(config[CONF_ID], *args)
    await register_cover(var, config)
    return var


COVER_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(Cover),
    }
)


COVER_MOVE_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(Cover),
        cv.Optional(CONF_SPEED): cv.templatable(cv.string_strict),
    }
)


def _register_speed_action(
    name: str,
    schema: cv.Schema,
    *fields: automation.ApplyField | automation.ApplyCall,
) -> None:
    """Register an apply action with a speed key; a set speed compiles the speed support."""
    builder = automation.apply_action_builder(
        name,
        schema,
        *fields,
        automation.ApplyField(
            CONF_SPEED,
            "set_speed",
            cg.std_string,
            const_fn=automation.literal_with_length,
        ),
        call="make_call",
    )

    async def action_to_code(
        config: ConfigType,
        action_id: ID,
        template_arg: cg.TemplateArguments,
        args: TemplateArgsType,
    ) -> MockObj:
        if CONF_SPEED in config:
            cg.add_define("USE_COVER_SPEED")
        return await builder(config, action_id, template_arg, args)

    automation.register_action(name, automation.ApplyAction, schema, synchronous=True)(
        action_to_code
    )


for _name, _command in (
    ("cover.open", "set_command_open()"),
    ("cover.close", "set_command_close()"),
):
    _register_speed_action(
        _name, COVER_MOVE_ACTION_SCHEMA, automation.ApplyCall(_command)
    )

for _name, _command in (
    ("cover.stop", "set_command_stop()"),
    ("cover.stop_tilt", "set_command_stop_tilt()"),
    ("cover.toggle", "set_command_toggle()"),
):
    automation.register_apply_action(
        _name, COVER_ACTION_SCHEMA, automation.ApplyCall(_command), call="make_call"
    )


COVER_CONTROL_ACTION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(Cover),
        cv.Optional(CONF_STOP): cv.templatable(cv.boolean),
        cv.Optional(CONF_STOP_TILT): cv.templatable(cv.boolean),
        cv.Exclusive(CONF_STATE, "pos"): cv.templatable(validate_cover_state),
        cv.Exclusive(CONF_POSITION, "pos"): cv.templatable(cv.percentage),
        cv.Optional(CONF_TILT): cv.templatable(cv.percentage),
        cv.Optional(CONF_SPEED): cv.templatable(cv.string_strict),
    }
)


# CONF_STATE and CONF_POSITION are cv.Exclusive in the schema, so at most
# one is present and both dispatch to set_position.
_register_speed_action(
    "cover.control",
    COVER_CONTROL_ACTION_SCHEMA,
    automation.ApplyField(CONF_STOP, "set_stop", cg.bool_),
    automation.ApplyField(CONF_STOP_TILT, "set_stop_tilt", cg.bool_),
    automation.ApplyField(CONF_STATE, "set_position", cg.float_),
    automation.ApplyField(CONF_POSITION, "set_position", cg.float_),
    automation.ApplyField(CONF_TILT, "set_tilt", cg.float_),
)


COVER_CONDITION_SCHEMA = cv.maybe_simple_value(
    {cv.Required(CONF_ID): cv.use_id(Cover)}, key=CONF_ID
)


automation.register_apply_condition(
    "cover.is_open", COVER_CONDITION_SCHEMA, f"position == {COVER_OPEN}"
)
automation.register_apply_condition(
    "cover.is_closed", COVER_CONDITION_SCHEMA, f"position == {COVER_CLOSED}"
)


@coroutine_with_priority(CoroPriority.CORE)
async def to_code(config: ConfigType) -> None:
    cg.add_global(cover_ns.using)
