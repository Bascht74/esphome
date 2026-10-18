"""Tests for devices on uart_split outputs and the settings those outputs carry."""

from collections.abc import Callable
from pathlib import Path

import pytest

from esphome import config, config_validation as cv, final_validate as fv, yaml_util
from esphome.components import uart, uart_split
from esphome.const import CONF_ID, CONF_OUTPUTS, CONF_RX_ONLY, CONF_UART_ID
from esphome.core import CORE, ID

BASE = """
esphome:
  name: uart-split-test
host:
logger:
"""

UART = """
uart:
  - id: pins
    port: /dev/ttyS0
    baud_rate: {baud_rate}
"""

SPLIT = """
uart_split:
  - id: split_bus
    uart_id: pins
    outputs:
      - id: bus
      - id: tap
        rx_only: true
"""

ONE_WRITER_TWO_TAPS = """
uart_split:
  - id: split_bus
    uart_id: pins
    outputs:
      - id: bus
      - id: tap
        rx_only: true
      - id: second_tap
        rx_only: true
"""

DUMMY_RECEIVER = """
    debug:
      dummy_receiver: true
"""

DFPLAYER = """
dfplayer:
  uart_id: {uart_id}
"""

DISTANCE = """
sensor:
  - platform: a01nyub
    uart_id: {uart_id}
    name: Distance
"""

CHAINED = """
  - id: second_split
    uart_id: tap
    outputs:
      - id: second_tap
        rx_only: true
"""


def _errors(tmp_path: Path, yaml: str) -> list[str]:
    path = tmp_path / "uart_split.yaml"
    path.write_text(yaml, encoding="utf-8")
    CORE.config_path = path
    result = config.validate_config(yaml_util.load_yaml(path), {})
    return [str(err) for err in result.errors]


def test_device_listed_before_the_split_is_valid(tmp_path: Path) -> None:
    """Final validation follows the YAML order; dfplayer checks the output before uart_split runs its own."""
    yaml = BASE + DFPLAYER.format(uart_id="bus") + UART.format(baud_rate=9600) + SPLIT
    assert _errors(tmp_path, yaml) == []


def test_platform_device_on_an_output_is_valid(tmp_path: Path) -> None:
    yaml = BASE + UART.format(baud_rate=9600) + SPLIT + DISTANCE.format(uart_id="tap")
    assert _errors(tmp_path, yaml) == []


def test_wrong_baud_rate_is_reported_at_the_uart(tmp_path: Path) -> None:
    yaml = BASE + DFPLAYER.format(uart_id="bus") + UART.format(baud_rate=115200) + SPLIT
    errors = _errors(tmp_path, yaml)
    assert any("requires baud rate 9600" in err for err in errors), errors


def test_device_that_transmits_on_a_receive_only_output_is_rejected(
    tmp_path: Path,
) -> None:
    yaml = BASE + UART.format(baud_rate=9600) + SPLIT + DFPLAYER.format(uart_id="tap")
    errors = _errors(tmp_path, yaml)
    assert any(
        "dfplayer requires the uart referenced by uart_id to transmit" in err
        for err in errors
    ), errors


def test_one_writer_and_two_receive_only_outputs_are_valid(tmp_path: Path) -> None:
    yaml = BASE + UART.format(baud_rate=9600) + ONE_WRITER_TWO_TAPS
    assert _errors(tmp_path, yaml) == []


def test_receive_only_output_with_a_generated_id_rejects_tx() -> None:
    """CONFIG_SCHEMA marks the output before the ID pass gives it a name."""
    conf = uart_split.CONFIG_SCHEMA(
        {CONF_UART_ID: "pins", CONF_OUTPUTS: [{CONF_RX_ONLY: True}]}
    )
    output_id = conf[CONF_OUTPUTS][0][CONF_ID]
    name = output_id.resolve([])
    full = fv.full_config.get()
    full["uart_split"] = [conf]
    full.declare_ids.append((output_id, ["uart_split", 0, CONF_OUTPUTS, 0, CONF_ID]))
    with pytest.raises(cv.Invalid, match="dfplayer requires the uart referenced"):
        uart.final_validate_device_schema("dfplayer", require_tx=True)(
            {CONF_UART_ID: ID(name)}
        )


def test_dummy_receiver_on_the_split_uart_is_rejected(tmp_path: Path) -> None:
    yaml = BASE + UART.format(baud_rate=9600) + DUMMY_RECEIVER + SPLIT
    errors = _errors(tmp_path, yaml)
    assert any("dummy_receiver reads this UART" in err for err in errors), errors


def test_device_on_a_split_of_an_output_is_valid(tmp_path: Path) -> None:
    yaml = (
        BASE
        + UART.format(baud_rate=9600)
        + SPLIT
        + CHAINED
        + DISTANCE.format(uart_id="second_tap")
    )
    assert _errors(tmp_path, yaml) == []


def test_split_of_an_output_generates_code(
    tmp_path: Path, generate_main: Callable[[str | Path], str]
) -> None:
    """The settings come from the parent at runtime, so no YAML lookup can fail here."""
    path = tmp_path / "uart_split.yaml"
    path.write_text(BASE + UART.format(baud_rate=9600) + SPLIT + CHAINED)
    main_cpp = generate_main(path)
    assert "uart_split::UartSplit(tap);" in main_cpp
    assert "second_tap->set_baud_rate" not in main_cpp
    assert "bus->set_rx_only" not in main_cpp
    assert "tap->set_rx_only(true);" in main_cpp
    # Same setup priority: the split that fills tap has to copy the settings first.
    assert main_cpp.index("register_component_(split_bus") < main_cpp.index(
        "register_component_(second_split"
    )


ARBITRATE = """
uart_split:
  - id: split_bus
    uart_id: pins
{options}    outputs:
      - id: bus
{bus_options}      - id: other
      - id: tap
        rx_only: true
        direction: BOTH
"""


def test_two_writers_reject_a_writer_that_mirrors(tmp_path: Path) -> None:
    yaml = (
        BASE
        + UART.format(baud_rate=9600)
        + ARBITRATE.format(options="", bus_options="        direction: BOTH\n")
    )
    errors = _errors(tmp_path, yaml)
    assert any(
        "A writer gets only the replies to its own requests" in err for err in errors
    ), errors


@pytest.mark.parametrize("key", ["send_wait_time", "turnaround_time"])
def test_timing_with_one_writer_is_rejected(tmp_path: Path, key: str) -> None:
    """One writer never waits for the line, so the times would do nothing."""
    yaml = (
        BASE
        + UART.format(baud_rate=9600)
        + SPLIT.replace("    outputs:", f"    {key}: 1s\n    outputs:")
    )
    errors = _errors(tmp_path, yaml)
    assert any(
        f"{key} only applies when more than one output writes" in err for err in errors
    ), errors


def test_two_writers_get_the_arbiter_with_the_hub_defaults(
    tmp_path: Path, generate_main: Callable[[str | Path], str]
) -> None:
    path = tmp_path / "uart_split.yaml"
    path.write_text(
        BASE
        + UART.format(baud_rate=9600)
        + ARBITRATE.format(options="", bus_options="")
    )
    main_cpp = generate_main(path)
    assert "uart_split::Arbiter(split_bus, 2000, 600);" in main_cpp
    assert "split_bus->set_arbiter(" in main_cpp


def test_two_writers_take_the_waveshare_times(
    tmp_path: Path, generate_main: Callable[[str | Path], str]
) -> None:
    """Instruction timeout 1024 ms and conflict time gap 50 ms, as set on a Waveshare gateway."""
    options = "    send_wait_time: 1024ms\n    turnaround_time: 50ms\n"
    path = tmp_path / "uart_split.yaml"
    path.write_text(
        BASE
        + UART.format(baud_rate=9600)
        + ARBITRATE.format(options=options, bus_options="")
    )
    main_cpp = generate_main(path)
    assert "uart_split::Arbiter(split_bus, 1024, 50);" in main_cpp


@pytest.mark.parametrize("split", [SPLIT, ONE_WRITER_TWO_TAPS])
def test_one_writer_generates_no_arbiter(
    tmp_path: Path, generate_main: Callable[[str | Path], str], split: str
) -> None:
    path = tmp_path / "uart_split.yaml"
    path.write_text(BASE + UART.format(baud_rate=9600) + split)
    main_cpp = generate_main(path)
    assert "Arbiter" not in main_cpp
