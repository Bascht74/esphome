"""Tests for the rfc2217_uart component."""

import pytest

from esphome import config_validation as cv
from esphome.components import rfc2217_uart
from esphome.core import CORE, ID

DIR = "tests/component_tests/rfc2217_uart"


def _defines() -> set[str]:
    return {define.name for define in CORE.defines}


def test_client_takes_its_line_settings(generate_main) -> None:
    main_cpp = generate_main(f"{DIR}/test_rfc2217_uart.yaml")
    assert "remote_port->set_tcp_uart(remote_link);" in main_cpp
    assert "remote_port->set_baud_rate(19200);" in main_cpp
    assert "remote_port->set_data_bits(7);" in main_cpp
    assert "remote_port->set_stop_bits(2);" in main_cpp
    assert "remote_port->set_parity(uart::UART_CONFIG_PARITY_EVEN);" in main_cpp
    assert {
        "USE_UART_VIRTUAL",
        "USE_UART_DEBUGGER",
        "USE_RFC2217_UART_CLIENT",
        "USE_RFC2217_UART_SERVER",
    } <= _defines()


def test_server_sits_on_the_hardware_uart(generate_main) -> None:
    main_cpp = generate_main(f"{DIR}/test_rfc2217_uart.yaml")
    assert "port_server->set_tcp_uart(inbound);" in main_cpp
    assert "port_server->set_uart_parent(serial_bus);" in main_cpp
    assert "port_server->set_baud_rate(" not in main_cpp


def test_server_alone_needs_no_virtual_uart(generate_main) -> None:
    main_cpp = generate_main(f"{DIR}/test_server_only.yaml")
    assert "port_server->set_uart_parent(serial_bus);" in main_cpp
    assert "USE_RFC2217_UART_SERVER" in _defines()
    assert not {"USE_UART_VIRTUAL", "USE_RFC2217_UART_CLIENT"} & _defines()


def test_server_needs_the_hardware_uart() -> None:
    with pytest.raises(cv.Invalid, match="uart_id"):
        rfc2217_uart.CONFIG_SCHEMA({"tcp_uart_id": "link", "role": "server"})


def test_client_needs_a_baud_rate() -> None:
    with pytest.raises(cv.Invalid, match="baud_rate"):
        rfc2217_uart.CONFIG_SCHEMA({"tcp_uart_id": "link"})


def test_client_rejects_one_and_a_half_stop_bits() -> None:
    with pytest.raises(cv.Invalid):
        rfc2217_uart.CONFIG_SCHEMA(
            {"tcp_uart_id": "link", "baud_rate": 9600, "stop_bits": 1.5}
        )


def test_server_takes_no_line_settings() -> None:
    with pytest.raises(cv.Invalid, match="baud_rate"):
        rfc2217_uart.CONFIG_SCHEMA(
            {
                "tcp_uart_id": "link",
                "role": "server",
                "uart_id": ID("serial_bus"),
                "baud_rate": 9600,
            }
        )
