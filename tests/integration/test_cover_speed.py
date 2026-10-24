"""The native API and the cover actions pass a cover speed to the platform."""

from __future__ import annotations

import struct

from aioesphomeapi import ButtonInfo, CoverInfo, api_pb2
import pytest

from .log_utils import LineWaiter
from .raw_api_client import (
    MESSAGE_TYPE_OF,
    WIRE_FIXED32,
    WIRE_LENGTH,
    WIRE_VARINT,
    RawApiClient,
    decode_fields,
    encode_varint,
    tag,
)
from .state_utils import require_entity
from .types import APIClientConnectedFactory, RunCompiledFunction

COVER_COMMAND = MESSAGE_TYPE_OF[api_pb2.CoverCommandRequest]
LIST_COVER = MESSAGE_TYPE_OF[api_pb2.ListEntitiesCoverResponse]
LIST_DONE = MESSAGE_TYPE_OF[api_pb2.ListEntitiesDoneResponse]
# The installed client library does not know the speed fields yet.
# ListEntitiesCoverResponse fields
FIELD_ENTITY_KEY, FIELD_SUPPORTED_SPEEDS = 2, 16
# CoverCommandRequest fields
FIELD_KEY, FIELD_HAS_POSITION, FIELD_POSITION = 1, 4, 5
FIELD_HAS_SPEED, FIELD_SPEED = 12, 13


def move_to(key: int, position: float, speed: str) -> bytes:
    """Build a CoverCommandRequest payload that moves to ``position`` at ``speed``."""
    name = speed.encode()
    return b"".join(
        (
            tag(FIELD_KEY, WIRE_FIXED32) + struct.pack("<I", key),
            tag(FIELD_HAS_POSITION, WIRE_VARINT) + b"\x01",
            tag(FIELD_POSITION, WIRE_FIXED32) + struct.pack("<f", position),
            tag(FIELD_HAS_SPEED, WIRE_VARINT) + b"\x01",
            tag(FIELD_SPEED, WIRE_LENGTH) + encode_varint(len(name)) + name,
        )
    )


@pytest.mark.asyncio
async def test_cover_speed(
    yaml_config: str,
    run_compiled: RunCompiledFunction,
    api_client_connected: APIClientConnectedFactory,
    unused_tcp_port: int,
) -> None:
    lines = LineWaiter()
    async with (
        run_compiled(yaml_config, line_callback=lines.callback),
        api_client_connected() as client,
        RawApiClient(unused_tcp_port) as raw,
    ):
        entities, _ = await client.list_entities_services()
        speed_cover = require_entity(entities, "speed_cover", CoverInfo, exact=True)
        plain_cover = require_entity(entities, "plain_cover", CoverInfo, exact=True)

        # Only the cover whose platform declares speeds lists them
        await raw.connect()
        await raw.send_message(api_pb2.ListEntitiesRequest())
        await raw.read_until_frame(LIST_DONE)
        speeds = {}
        for payload in raw.frames(LIST_COVER):
            fields = decode_fields(payload)
            speeds[fields[FIELD_ENTITY_KEY][0]] = fields.get(FIELD_SUPPORTED_SPEEDS, [])
        assert speeds == {speed_cover.key: [b"slow", b"fast"], plain_cover.key: []}

        # A known speed from the API reaches control()
        moved = lines.expect("Move to 0.00 at speed 'fast'")
        await raw.send_raw(COVER_COMMAND, move_to(speed_cover.key, 0.0, "fast"))
        await lines.wait(moved)

        # An unknown speed is dropped with a warning; the cover moves at its default speed
        warned = lines.expect("speed 'turbo' unsupported")
        moved = lines.expect("Move to 1.00 at speed ''")
        await raw.send_raw(COVER_COMMAND, move_to(speed_cover.key, 1.0, "turbo"))
        await lines.wait(warned)
        await lines.wait(moved)

        # The actions: cover.open with a speed, the short cover.close, cover.control with a lambda
        for button, line in (
            ("open_slow", "Move to 1.00 at speed 'slow'"),
            ("close_default", "Move to 0.00 at speed ''"),
            ("half_fast", "Move to 0.50 at speed 'fast'"),
        ):
            moved = lines.expect(line)
            client.button_command(require_entity(entities, button, ButtonInfo).key)
            await lines.wait(moved, msg=button)
