import esphome.codegen as cg
from esphome.components import binary_sensor, noise, socket, uart
from esphome.components.const import (
    CONF_DATA_BITS,
    CONF_HOST,
    CONF_PARITY,
    CONF_RECONNECT_INTERVAL,
    CONF_STOP_BITS,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_API,
    CONF_BAUD_RATE,
    CONF_ENCRYPTION,
    CONF_ID,
    CONF_KEY,
    CONF_PORT,
    CONF_PROTOCOL,
    CONF_TIMEOUT,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.core import CORE
from esphome.types import ConfigType

CODEOWNERS = ["@Bascht74"]
DEPENDENCIES = ["network"]


def AUTO_LOAD(config):
    base = ["uart", "binary_sensor", "socket"]
    if not config or CONF_ENCRYPTION in config:
        base.append("noise")
    return base


MULTI_CONF = True

tcp_uart_ns = cg.esphome_ns.namespace("tcp_uart")
TcpUart = tcp_uart_ns.class_("TcpUart", uart.UARTComponent, cg.Component)
TcpUartModbus = tcp_uart_ns.class_("TcpUartModbus", TcpUart)
TcpUartNoise = tcp_uart_ns.class_("TcpUartNoise", TcpUart)
TcpUartModbusNoise = tcp_uart_ns.class_("TcpUartModbusNoise", TcpUartModbus)

CONF_CONNECTED = "connected"


def _select_class(config: ConfigType) -> ConfigType:
    encrypted = CONF_ENCRYPTION in config
    if encrypted:
        noise.inherit_encryption_key(config[CONF_ENCRYPTION], (CORE.config or {}).get(CONF_API) or {}, "tcp_uart")
    modbus = config[CONF_PROTOCOL] == "modbus"
    if modbus and encrypted:
        config[CONF_ID].type = TcpUartModbusNoise
    elif modbus:
        config[CONF_ID].type = TcpUartModbus
    elif encrypted:
        config[CONF_ID].type = TcpUartNoise
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TcpUart),
            cv.Required(CONF_HOST): cv.string,
            cv.Required(CONF_PORT): cv.port,
            cv.Optional(CONF_BAUD_RATE, default=9600): cv.int_range(min=1),
            cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=5, max=8),
            cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                uart.UART_PARITY_OPTIONS, upper=True
            ),
            cv.Optional(CONF_STOP_BITS, default=1): cv.one_of(1, 2, int=True),
            cv.Optional(
                CONF_RECONNECT_INTERVAL, default="5s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PROTOCOL, default="raw"): cv.one_of(
                "raw", "modbus", lower=True
            ),
            cv.Optional(CONF_TIMEOUT, default="0s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ENCRYPTION): noise.encryption_schema,
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    socket.consume_sockets(1, "tcp_uart"),
    _select_class,
)


async def to_code(config: ConfigType) -> None:
    socket.require_tcp_client_link()
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_HOST], config[CONF_PORT])
    await cg.register_component(var, config)
    cg.add(var.set_reconnect_interval(config[CONF_RECONNECT_INTERVAL]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    # The socket is not clocked. These only satisfy UARTComponent and a consumer check.
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_data_bits(config[CONF_DATA_BITS]))
    cg.add(var.set_stop_bits(config[CONF_STOP_BITS]))
    cg.add(var.set_parity(config[CONF_PARITY]))
    if config[CONF_PROTOCOL] == "modbus":
        cg.add_define("USE_TCP_UART_MODBUS")
    if CONF_ENCRYPTION in config:
        cg.add_define("USE_TCP_UART_NOISE")
        cg.add_define("USE_NOISE_UART")
        cg.add(var.set_noise_psk(noise.new_psk_progmem(config[CONF_ID], config[CONF_ENCRYPTION][CONF_KEY])))
        cg.add(var.set_noise_initiator(True))
    binary_sensors = binary_sensor.sub_binary_sensors(config)
    await binary_sensors(CONF_CONNECTED, var.set_connected_sensor)
