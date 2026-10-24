"""Cover speeds: the platform list helpers and the speed option of the actions."""

from collections.abc import Callable
from pathlib import Path
from unittest.mock import patch

import pytest

from esphome.components import cover
import esphome.config_validation as cv
from esphome.core import CORE
from esphome.cpp_generator import MockObj


def test_validate_speeds_keeps_the_names_in_order() -> None:
    assert cover.validate_speeds(["slow", "fast"]) == ["slow", "fast"]
    assert cover.validate_speeds("slow") == ["slow"]


def test_validate_speeds_rejects_a_duplicate() -> None:
    with pytest.raises(cv.Invalid, match="duplicate speed 'slow'"):
        cover.validate_speeds(["slow", "fast", "slow"])


@pytest.mark.parametrize("speeds", [[], [""]])
def test_validate_speeds_rejects_an_empty_list_or_name(speeds: list[str]) -> None:
    # An empty speed means the default speed, so it cannot be a name in the list
    with pytest.raises(cv.Invalid):
        cover.validate_speeds(speeds)


def test_set_speeds_points_the_cover_at_a_flash_table() -> None:
    with (
        patch.object(cover.cg, "add") as add,
        patch.object(cover.cg, "add_define") as add_define,
        patch.object(
            cover.cg, "shared_progmem_array", return_value=MockObj("cover_speeds")
        ) as array,
    ):
        cover.set_speeds(MockObj("blind", "->"), ["slow", "fast"])
    array.assert_called_once_with(
        "cover_speeds", cover.cg.const_char_ptr, ["slow", "fast"]
    )
    assert str(add.call_args.args[0]) == (
        "blind->set_supported_speeds_static(cover_speeds, 2)"
    )
    add_define.assert_called_once_with("USE_COVER_SPEED")


def test_set_speeds_without_speeds_emits_nothing() -> None:
    with (
        patch.object(cover.cg, "add") as add,
        patch.object(cover.cg, "add_define") as add_define,
    ):
        cover.set_speeds(MockObj("blind"), [])
    add.assert_not_called()
    add_define.assert_not_called()


def test_actions_pass_the_speed_to_the_call(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    main_cpp = generate_main(component_config_path("speed_actions.yaml"))

    assert 'apply_call.set_speed("slow", 4);' in main_cpp
    assert 'apply_call.set_speed(std::string("fast"));' in main_cpp
    # cover.close in its short form carries no speed
    assert main_cpp.count("set_speed(") == 2
    assert "USE_COVER_SPEED" in {d.name for d in CORE.defines}


def test_actions_without_a_speed_leave_speed_support_out(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    main_cpp = generate_main(component_config_path("no_speed_actions.yaml"))

    assert "apply_call.set_command_open();" in main_cpp
    assert "set_speed(" not in main_cpp
    assert "USE_COVER_SPEED" not in {d.name for d in CORE.defines}
