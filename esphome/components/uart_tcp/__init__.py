import esphome.codegen as cg
from esphome.components import binary_sensor, noise, socket, uart
from esphome.components.const import CONF_HOST, CONF_RECONNECT_INTERVAL, CONF_ROLE
import esphome.config_validation as cv
from esphome.const import (
    CONF_API,
    CONF_ENCRYPTION,
    CONF_ID,
    CONF_KEY,
    CONF_PORT,
    CONF_PROTOCOL,
    CONF_TIMEOUT,
    CONF_UART_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.core import CORE, TimePeriodMilliseconds
from esphome.types import ConfigType

CODEOWNERS = ["@Bascht74"]
DEPENDENCIES = ["network", "uart"]


def AUTO_LOAD(config):
    base = ["binary_sensor", "socket"]
    if not config or CONF_ENCRYPTION in config:
        base.append("noise")
    return base


MULTI_CONF = True

uart_tcp_ns = cg.esphome_ns.namespace("uart_tcp")
UartTcp = uart_tcp_ns.class_("UartTcp", cg.Component, uart.UARTDevice)
UartTcpModbus = uart_tcp_ns.class_("UartTcpModbus", UartTcp)
UartTcpNoise = uart_tcp_ns.class_("UartTcpNoise", UartTcp)
UartTcpModbusNoise = uart_tcp_ns.class_("UartTcpModbusNoise", UartTcpModbus)

CONF_ALLOWED_IPS = "allowed_ips"
CONF_CONNECTED = "connected"
CONF_SEND_WAIT_TIME = "send_wait_time"


def _select_class(config: ConfigType) -> ConfigType:
    encrypted = CONF_ENCRYPTION in config
    if encrypted:
        noise.inherit_encryption_key(config[CONF_ENCRYPTION], (CORE.config or {}).get(CONF_API) or {}, "uart_tcp")
    modbus = config[CONF_PROTOCOL] == "modbus"
    if modbus and encrypted:
        config[CONF_ID].type = UartTcpModbusNoise
    elif modbus:
        config[CONF_ID].type = UartTcpModbus
    elif encrypted:
        config[CONF_ID].type = UartTcpNoise
    return config


def _check_send_wait(config: ConfigType) -> ConfigType:
    if config[CONF_PROTOCOL] != "modbus" and CONF_SEND_WAIT_TIME in config:
        raise cv.Invalid(
            "send_wait_time is only used when protocol is modbus",
            path=[CONF_SEND_WAIT_TIME],
        )
    return config


BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UartTcp),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_PORT): cv.port,
        cv.Optional(
            CONF_RECONNECT_INTERVAL, default="5s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PROTOCOL, default="raw"): cv.one_of("raw", "modbus", lower=True),
        cv.Optional(CONF_TIMEOUT, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SEND_WAIT_TIME): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ENCRYPTION): noise.encryption_schema,
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            "client": BASE_SCHEMA.extend({cv.Required(CONF_HOST): cv.string}),
            "server": BASE_SCHEMA.extend(
                {cv.Optional(CONF_ALLOWED_IPS): socket.IPV4_ALLOW_SCHEMA}
            ),
        },
        key=CONF_ROLE,
        default_type="client",
        lower=True,
    ),
    socket.consume_role_sockets("uart_tcp"),
    _check_send_wait,
    _select_class,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if config[CONF_ROLE] == "server":
        socket.require_tcp_listener()
        cg.add(var.set_server(True))
        socket.add_ipv4_allow(
            var.set_allow, config.get(CONF_ALLOWED_IPS), config[CONF_ID]
        )
    else:
        socket.require_tcp_client_link()
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_reconnect_interval(config[CONF_RECONNECT_INTERVAL]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    if (host := config.get(CONF_HOST)) is not None:
        cg.add(var.set_host(host))
    if config[CONF_PROTOCOL] == "modbus":
        cg.add_define("USE_UART_TCP_MODBUS")
        cg.add(
            var.set_send_wait_time(
                config.get(CONF_SEND_WAIT_TIME, TimePeriodMilliseconds(milliseconds=2000))
            )
        )
    if CONF_ENCRYPTION in config:
        cg.add_define("USE_UART_TCP_NOISE")
        cg.add_define("USE_NOISE_UART")
        cg.add(var.set_noise_psk(noise.new_psk_progmem(config[CONF_ID], config[CONF_ENCRYPTION][CONF_KEY])))
        cg.add(var.set_noise_initiator(config[CONF_ROLE] != "server"))
        if config[CONF_ROLE] == "server":
            noise.enable_spare_ephemeral()
    binary_sensors = binary_sensor.sub_binary_sensors(config)
    await binary_sensors(CONF_CONNECTED, var.set_connected_sensor)
