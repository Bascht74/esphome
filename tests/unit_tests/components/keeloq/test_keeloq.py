"""The keeloq remote schema: only the Jarolift type for now, a 64-bit key, a 28-bit serial."""

from __future__ import annotations

import pytest

from esphome.components.keeloq import (
    CONFIG_SCHEMA,
    JAROLIFT_COMMANDS,
    _actions_have_channels,
    _channels,
    _command,
)
import esphome.config_validation as cv

# Made-up key, not a real manufacturer key
BASE = {
    "type": "jarolift",
    "manufacturer_key": "0x1A2B3C4D5E6F7081",
    "serial": 0x1234500,
}


def test_jarolift_remote_is_accepted() -> None:
    config = CONFIG_SCHEMA(BASE)
    assert config["manufacturer_key"] == 0x1A2B3C4D5E6F7081


def test_unknown_types_are_rejected() -> None:
    with pytest.raises(cv.Invalid, match="jarolift"):
        CONFIG_SCHEMA({**BASE, "type": "rolling"})


def test_seed_belongs_to_secure_only() -> None:
    assert (
        CONFIG_SCHEMA({**BASE, "type": "secure", "seed": 0x13572468})["seed"]
        == 0x13572468
    )
    with pytest.raises(cv.Invalid, match="seed is needed for type secure"):
        CONFIG_SCHEMA({**BASE, "type": "secure"})
    with pytest.raises(cv.Invalid, match="seed is needed for type secure"):
        CONFIG_SCHEMA({**BASE, "seed": 1})


def test_key_has_64_bits() -> None:
    with pytest.raises(cv.Invalid, match="64-bit"):
        CONFIG_SCHEMA({**BASE, "manufacturer_key": "0x1" + "0" * 16})


def test_serial_has_28_bits() -> None:
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA({**BASE, "serial": 0x10000000})


def test_commands_by_name_or_number() -> None:
    assert _command("Up") == "up"
    assert _command(0x8) == 0x8
    with pytest.raises(cv.Invalid, match="one of up, down, stop, learn, shade"):
        _command("open")


def test_on_frame_needs_a_receiver() -> None:
    with pytest.raises(cv.Invalid, match="on_frame needs receiver_id"):
        CONFIG_SCHEMA({**BASE, "on_frame": []})


def test_a_remote_needs_serial_or_receiver() -> None:
    config = {k: v for k, v in BASE.items() if k != "serial"}
    with pytest.raises(cv.Invalid, match="Set serial, or receiver_id"):
        CONFIG_SCHEMA(config)


def test_on_command_needs_a_receiver_and_valid_filters() -> None:
    with pytest.raises(cv.Invalid, match="on_command needs receiver_id"):
        CONFIG_SCHEMA({**BASE, "on_command": []})
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA(
            {**BASE, "receiver_id": "rx", "on_command": [{"channel": 17, "then": []}]}
        )


def test_shade_is_stop_sent_20_times() -> None:
    assert JAROLIFT_COMMANDS["shade"] == (0x4, 20)
    assert JAROLIFT_COMMANDS["up"] == (0x8, None)


def _check_groups(remote: dict, action: dict) -> None:
    import esphome.final_validate as fv

    full = {"keeloq": [remote], "button": [{"on_press": [{"keeloq.send": action}]}]}
    token = fv.full_config.set(full)
    try:
        _actions_have_channels(remote)
    finally:
        fv.full_config.reset(token)


def test_a_frame_needs_channels_from_the_action_or_the_remote() -> None:
    with_serial = {"id": "own", "type": "jarolift", "serial": 0x1234500}
    _check_groups(with_serial, {"id": "own", "channels": 0x0001})
    _check_groups({**with_serial, "channels": 0x0001}, {"id": "own"})
    # Without serial the channels may come from the hand remote.
    _check_groups({"id": "clone", "type": "jarolift"}, {"id": "clone"})
    with pytest.raises(cv.Invalid, match="own has no channels"):
        _check_groups(with_serial, {"id": "own"})


class _FullConfig(dict):
    """Two remotes: own_sender has a serial, clone takes one over."""

    def __init__(self) -> None:
        super().__init__(
            keeloq=[{"id": "own_sender", "serial": 0x1234500}, {"id": "clone"}]
        )
        self.data: dict = {}

    def get_path_for_id(self, remote_id: str) -> list:
        return ["keeloq", 0 if remote_id == "own_sender" else 1, "id"]

    def get_config_for_path(self, path: list) -> dict:
        return self[path[0]][path[1]]


def _final_validate(*buttons: dict) -> None:
    from esphome.components.keeloq.button import FINAL_VALIDATE_SCHEMA
    import esphome.final_validate as fv

    token = fv.full_config.set(_FullConfig())
    try:
        for config in buttons:
            FINAL_VALIDATE_SCHEMA(config)
    finally:
        fv.full_config.reset(token)


def test_learn_needs_a_serial() -> None:
    _final_validate({"keeloq_id": "own_sender", "learn": {}})
    with pytest.raises(cv.Invalid, match="learn needs a remote with serial"):
        _final_validate({"keeloq_id": "clone", "learn": {}})


def test_take_over_works_with_and_without_serial() -> None:
    _final_validate({"keeloq_id": "own_sender", "take_over": {}})
    _final_validate({"keeloq_id": "clone", "take_over": {}})


def test_learn_and_take_over_exclude_each_other_across_blocks() -> None:
    with pytest.raises(cv.Invalid, match="both learn and take_over"):
        _final_validate(
            {"keeloq_id": "own_sender", "learn": {}},
            {"keeloq_id": "own_sender", "take_over": {}},
        )


def test_maker_names_map_to_a_learning_type() -> None:
    assert CONFIG_SCHEMA({**BASE, "type": "BFT", "seed": 1})["type"] == "secure"


def test_auto_type_only_takes_over() -> None:
    config = {k: v for k, v in BASE.items() if k != "serial"}
    assert (
        CONFIG_SCHEMA({**config, "type": "auto", "receiver_id": "rx"})["type"] == "auto"
    )
    with pytest.raises(cv.Invalid, match="type auto takes the type from a hand remote"):
        CONFIG_SCHEMA({**BASE, "type": "auto", "receiver_id": "rx"})


def test_command_names_only_for_jarolift() -> None:
    garage = {"id": "garage", "type": "normal", "serial": 0x0ABCDEF}
    _check_groups(garage, {"id": "garage", "command": 0x2})
    with pytest.raises(cv.Invalid, match="type normal has no command names"):
        _check_groups(garage, {"id": "garage", "command": "up"})


def test_channels_as_list_or_mask() -> None:
    # Channels 1 and 3 are bits 0 and 2: mask 0x0005.
    assert _channels([1, 3]) == 0x0005
    assert _channels(0x0005) == 0x0005
    assert _channels([16]) == 0x8000
    with pytest.raises(cv.Invalid, match="each channel only once"):
        _channels([1, 1])
    with pytest.raises(cv.Invalid):
        _channels([17])
