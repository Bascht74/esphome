import esphome.codegen as cg
from esphome.components import tcp_uart, uart
from esphome.components.const import (
    CONF_DATA_BITS,
    CONF_PARITY,
    CONF_ROLE,
    CONF_STOP_BITS,
)
from esphome.config_helpers import filter_source_files_from_defines
import esphome.config_validation as cv
from esphome.const import CONF_BAUD_RATE, CONF_DEBUG, CONF_ID, CONF_UART_ID
from esphome.types import ConfigType

CODEOWNERS = ["@Bascht74"]
DEPENDENCIES = ["tcp_uart"]
MULTI_CONF = True

CONF_TCP_UART_ID = "tcp_uart_id"

rfc2217_uart_ns = cg.esphome_ns.namespace("rfc2217_uart")
Rfc2217Base = rfc2217_uart_ns.class_("Rfc2217Base", cg.Component)
Rfc2217Client = rfc2217_uart_ns.class_(
    "Rfc2217Client", uart.VirtualUARTComponent, Rfc2217Base
)
Rfc2217Server = rfc2217_uart_ns.class_("Rfc2217Server", Rfc2217Base, uart.UARTDevice)

BASE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TCP_UART_ID): cv.use_id(tcp_uart.TcpUart),
    }
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.typed_schema(
    {
        # The line settings go to the access server.
        "client": BASE_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(Rfc2217Client),
                cv.Required(CONF_BAUD_RATE): cv.int_range(min=1, max=0xFFFFFFFF),
                cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=5, max=8),
                cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                    uart.UART_PARITY_OPTIONS, upper=True
                ),
                cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
                cv.Optional(CONF_DEBUG): uart.maybe_empty_debug,
            }
        ),
        "server": BASE_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(Rfc2217Server),
                # The tcp_uart is a UART too, so there is never a single one to default to.
                cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            }
        ),
    },
    key=CONF_ROLE,
    default_type="client",
    lower=True,
)


def _final_validate(config: ConfigType) -> None:
    if config[CONF_ROLE] == "server":
        uart.final_validate_device_schema(
            "rfc2217_uart", require_tx=True, require_rx=True
        )(config)


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_tcp_uart(await cg.get_variable(config[CONF_TCP_UART_ID])))
    if config[CONF_ROLE] == "server":
        cg.add_define("USE_RFC2217_UART_SERVER")
        await uart.register_uart_device(var, config)
        return
    cg.add_define("USE_RFC2217_UART_CLIENT")
    uart.require_virtual_uart()
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_data_bits(config[CONF_DATA_BITS]))
    cg.add(var.set_stop_bits(config[CONF_STOP_BITS]))
    cg.add(var.set_parity(config[CONF_PARITY]))
    if debug := config.get(CONF_DEBUG):
        cg.add_global(uart.uart_ns.using)
        await uart.debug_to_code(debug, var)


# Each role compiles only when configured; the client also needs the virtual UART base.
FILTER_SOURCE_FILES = filter_source_files_from_defines(
    {
        "rfc2217_client.cpp": "USE_RFC2217_UART_CLIENT",
        "rfc2217_server.cpp": "USE_RFC2217_UART_SERVER",
    }
)
