import esphome.codegen as cg
from esphome.components import uart
from esphome.config_helpers import filter_source_files_from_defines
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEBUG,
    CONF_DIRECTION,
    CONF_DUMMY_RECEIVER,
    CONF_ID,
    CONF_OUTPUTS,
    CONF_RX_ONLY,
    CONF_UART_ID,
)
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@Bascht74"]
DEPENDENCIES = ["uart"]
DOMAIN = "uart_split"
MULTI_CONF = True

uart_split_ns = cg.esphome_ns.namespace("uart_split")
UartSplit = uart_split_ns.class_("UartSplit", cg.Component)
UartSplitOutput = uart_split_ns.class_("UartSplitOutput", uart.VirtualUARTComponent)
Arbiter = uart_split_ns.class_("Arbiter")

CONF_ARBITER_ID = "arbiter_id"
CONF_SEND_WAIT_TIME = "send_wait_time"
CONF_TURNAROUND_TIME = "turnaround_time"

MAX_OUTPUTS = 8

_TIME_PERIOD = cv.All(
    cv.positive_time_period_milliseconds,
    cv.Range(max=cv.TimePeriod(milliseconds=65535)),
)


def _default_direction(config: ConfigType) -> ConfigType:
    for output in config[CONF_OUTPUTS]:
        if CONF_DIRECTION not in output:
            output[CONF_DIRECTION] = "BOTH" if output[CONF_RX_ONLY] else "RX"
    return config


def _inherit_settings(config: ConfigType) -> ConfigType:
    # Devices on an output are checked against the baud rate and framing of the UART it copies.
    for output in config[CONF_OUTPUTS]:
        uart.inherit_settings(output[CONF_ID], config[CONF_UART_ID])
        if output[CONF_RX_ONLY]:
            uart.mark_receive_only(output[CONF_ID])
    return config


OUTPUT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UartSplitOutput),
        cv.Optional(CONF_RX_ONLY, default=False): cv.boolean,
        cv.Optional(CONF_DIRECTION): cv.one_of("RX", "BOTH", upper=True),
        cv.Optional(CONF_DEBUG): uart.maybe_empty_debug,
    }
)


def _writers(config: ConfigType) -> int:
    return sum(not output[CONF_RX_ONLY] for output in config[CONF_OUTPUTS])


def _arbitration(config: ConfigType) -> ConfigType:
    # One writer gets a plain copy. Several writers take turns on the line.
    if _writers(config) <= 1:
        for key in (CONF_SEND_WAIT_TIME, CONF_TURNAROUND_TIME):
            if key in config:
                raise cv.Invalid(
                    f"{key} only applies when more than one output writes.", [key]
                )
        return config
    for index, output in enumerate(config[CONF_OUTPUTS]):
        if not output[CONF_RX_ONLY] and output.get(CONF_DIRECTION) == "BOTH":
            raise cv.Invalid(
                "A writer gets only the replies to its own requests when several outputs write; "
                "add an rx_only output with direction: BOTH to see the whole line.",
                [CONF_OUTPUTS, index, CONF_DIRECTION],
            )
    config.setdefault(CONF_SEND_WAIT_TIME, _TIME_PERIOD("2000ms"))
    config.setdefault(CONF_TURNAROUND_TIME, _TIME_PERIOD("600ms"))
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UartSplit),
            cv.GenerateID(CONF_ARBITER_ID): cv.declare_id(Arbiter),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_OUTPUTS): cv.All(
                cv.ensure_list(OUTPUT_SCHEMA),
                cv.Length(min=1, max=MAX_OUTPUTS),
            ),
            cv.Optional(CONF_SEND_WAIT_TIME): _TIME_PERIOD,
            cv.Optional(CONF_TURNAROUND_TIME): _TIME_PERIOD,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _arbitration,
    _default_direction,
    _inherit_settings,
)


def _subtree_has_uart(node: object, uart_id: str) -> bool:
    if isinstance(node, dict):
        return any(
            (key == CONF_UART_ID and str(value) == uart_id)
            or _subtree_has_uart(value, uart_id)
            for key, value in node.items()
        )
    if isinstance(node, list):
        return any(_subtree_has_uart(item, uart_id) for item in node)
    return False


def _reject_dummy_receiver(uart_conf: ConfigType) -> ConfigType:
    debug = uart_conf.get(CONF_DEBUG)
    if isinstance(debug, dict) and debug.get(CONF_DUMMY_RECEIVER):
        raise cv.Invalid(
            "dummy_receiver reads this UART and takes the bytes uart_split should copy.",
            [CONF_DEBUG, CONF_DUMMY_RECEIVER],
        )
    return uart_conf


def _final_validate(config: ConfigType) -> ConfigType:
    # This component is the only reader. A second consumer would take the bytes.
    full_config = fv.full_config.get()
    owned = full_config.data.setdefault("uart_split", set())
    uart_id = str(config[CONF_UART_ID])
    if uart_id in owned:
        raise cv.Invalid(
            f"The UART '{uart_id}' is already read by another uart_split.",
            [CONF_UART_ID],
        )
    owned.add(uart_id)
    for domain, domain_conf in full_config.items():
        if domain == "uart_split":
            continue
        if _subtree_has_uart(domain_conf, uart_id):
            raise cv.Invalid(
                f"The UART '{uart_id}' is also used by '{domain}'. "
                "uart_split has to be the only reader; use one of its outputs.",
                [CONF_UART_ID],
            )
    fv.id_declaration_match_schema(_reject_dummy_receiver)(config[CONF_UART_ID])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    uart.require_virtual_uart()
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await cg.register_component(var, config)
    for output_config in config[CONF_OUTPUTS]:
        output = cg.new_Pvariable(output_config[CONF_ID], var)
        if output_config[CONF_RX_ONLY]:
            cg.add(output.set_rx_only(True))
        if output_config[CONF_DIRECTION] == "BOTH":
            cg.add(output.set_mirror_tx(True))
        if debug := output_config.get(CONF_DEBUG):
            cg.add_global(uart.uart_ns.using)
            await uart.debug_to_code(debug, output)
        cg.add(var.add_output(output))
    if _writers(config) > 1:
        cg.add_define("USE_UART_SPLIT_ARBITER")
        arbiter = cg.new_Pvariable(
            config[CONF_ARBITER_ID],
            var,
            config[CONF_SEND_WAIT_TIME],
            config[CONF_TURNAROUND_TIME],
        )
        cg.add(var.set_arbiter(arbiter))


FILTER_SOURCE_FILES = filter_source_files_from_defines(
    {"arbiter.cpp": "USE_UART_SPLIT_ARBITER"}
)
