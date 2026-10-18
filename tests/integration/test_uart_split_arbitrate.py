"""Two TCP clients share one serial line through uart_split; its two writers take turns.

Each client has its own uart_tcp port on its own output. The device answers A and B,
never C. Each client must get only the reply to its own request.
"""

from __future__ import annotations

import asyncio

import pytest

from .log_utils import LineWaiter
from .types import APIClientConnectedFactory, RunCompiledFunction

REQUEST_A = bytes([0x0A, 0x03, 0x00, 0x01])
REPLY_A = bytes([0x0A, 0x03, 0xAA, 0xAA])
REQUEST_B = bytes([0x0B, 0x03, 0x00, 0x01])
REPLY_B = bytes([0x0B, 0x03, 0xBB, 0xBB])
REQUEST_C = bytes([0x0C, 0x03, 0x00, 0x01])


async def _wait_for_clients(lines: LineWaiter, count: int) -> None:
    for _ in range(100):
        if sum("Client connected from" in line for line in lines.lines) >= count:
            return
        await asyncio.sleep(0.05)
    pytest.fail(f"fewer than {count} clients connected")


async def _nothing_more(reader: asyncio.StreamReader) -> None:
    with pytest.raises(asyncio.TimeoutError):
        extra = await asyncio.wait_for(reader.read(1), 0.3)
        pytest.fail(f"unexpected bytes {extra.hex()}")


@pytest.mark.asyncio
async def test_uart_split_arbitrate(
    yaml_config: str,
    run_compiled: RunCompiledFunction,
    api_client_connected: APIClientConnectedFactory,
    unused_tcp_port_factory,
) -> None:
    port_a = unused_tcp_port_factory()
    port_b = unused_tcp_port_factory()
    yaml_config = yaml_config.replace("port: 18140", f"port: {port_a}")
    yaml_config = yaml_config.replace("port: 18141", f"port: {port_b}")

    lines = LineWaiter()
    async with (
        run_compiled(yaml_config, line_callback=lines.callback),
        api_client_connected() as client,
    ):
        assert await client.device_info() is not None
        await lines.wait_for(f"Listening on {port_a}")
        await lines.wait_for(f"Listening on {port_b}")

        reader_a, writer_a = await asyncio.open_connection("127.0.0.1", port_a)
        reader_b, writer_b = await asyncio.open_connection("127.0.0.1", port_b)
        await _wait_for_clients(lines, 2)

        # Both ask at the same time: B waits for A's reply and the turnaround time.
        writer_a.write(REQUEST_A)
        writer_b.write(REQUEST_B)
        await asyncio.gather(writer_a.drain(), writer_b.drain())
        assert await asyncio.wait_for(reader_a.readexactly(4), 5) == REPLY_A
        assert await asyncio.wait_for(reader_b.readexactly(4), 5) == REPLY_B
        await asyncio.gather(_nothing_more(reader_a), _nothing_more(reader_b))

        # C gets no reply: after send_wait_time B has the line again.
        writer_a.write(REQUEST_C)
        await writer_a.drain()
        await asyncio.sleep(0.1)
        writer_b.write(REQUEST_B)
        await writer_b.drain()
        await lines.wait_for("No reply for output 0")
        assert await asyncio.wait_for(reader_b.readexactly(4), 5) == REPLY_B

        # A is served again after its unanswered request.
        writer_a.write(REQUEST_A)
        await writer_a.drain()
        assert await asyncio.wait_for(reader_a.readexactly(4), 5) == REPLY_A
        await asyncio.gather(_nothing_more(reader_a), _nothing_more(reader_b))

        writer_a.close()
        writer_b.close()
