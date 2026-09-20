"""Async serial driver for Waveshare USB-TO-LoRa-xF (SX1262).

Typical use — paired stream-mode JSON link::

    import asyncio
    from waveshare_lora import LoRaDevice, LoraConfig

    async def main():
        cfg = LoraConfig(txch=18, rxch=18)          # HF 868 MHz
        async with LoRaDevice("/dev/ttyUSB0", config=cfg) as dev:
            await dev.send_json({"temp": 21.5})
            data, meta = await dev.recv_json(timeout=10.0)
            print(data, meta)

    asyncio.run(main())

Packet mode (AT+MODE=2) adds per-fragment 3-byte addressing::

    await dev.send_json({"cmd": "on"}, dest_addr=0x0005, dest_channel=18)

All serial I/O is ``asyncio``-native via ``pyserial-asyncio``; a background
reader task feeds :class:`codec.StreamDecoder` and publishes complete
messages on an :class:`asyncio.Queue`.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Awaitable, Callable

from . import at as _at
from .codec import (
    DEFAULT_MTU,
    PACKET_MODE_OVERHEAD,
    DecodedMessage,
    StreamDecoder,
    encode_json,
    encode_raw,
    packet_prefix,
)
from .config import LoraConfig, OperationMode, comm_to_pyserial

_AT_OK_TOKENS = (b"OK", b"+OK")


@dataclass
class Received:
    data: Any
    meta: Any  # FrameMeta (+ src/channel filled when known)


class LoRaDevice:
    """Async USB-TO-LoRa endpoint."""

    def __init__(self, port: str, config: LoraConfig | None = None,
                 serial_baud: int | None = None,
                 mtu: int = DEFAULT_MTU,
                 queue_size: int = 128,
                 compress_level: int = 9,
                 at_timeout: float = 2.5) -> None:
        self.port = port
        self.config = config or LoraConfig()
        # Host-side serial baud: must equal the device's AT+BAUD while talking.
        self.serial_baud = serial_baud or int(self.config.baud)
        self.mtu = mtu
        self.compress_level = compress_level
        self.at_timeout = at_timeout

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._rx_task: asyncio.Task | None = None
        self._write_lock = asyncio.Lock()
        self._seq = 0
        self._decoder = StreamDecoder(
            rssi_trailer=1 if self.config.rssi else 0)
        self.queue: asyncio.Queue[DecodedMessage] = asyncio.Queue(maxsize=queue_size)

        # AT-session state (reader task routes bytes here while set)
        self._at_mode = False
        self._at_buf = bytearray()
        self._at_event = asyncio.Event()

    # -- properties ------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    @property
    def packet_mode(self) -> bool:
        return int(self.config.mode) == int(OperationMode.PACKET)

    def _air_mtu(self) -> int:
        return self.mtu - PACKET_MODE_OVERHEAD if self.packet_mode else self.mtu

    # -- lifecycle ---------------------------------------------------------
    async def connect(self) -> None:
        if self.is_open:
            return
        import serial_asyncio

        extra = comm_to_pyserial(self.config.comm)
        extra.pop("_device_databits", None)
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self.port, baudrate=self.serial_baud,
            bytesize=extra["bytesize"], parity=extra["parity"],
            stopbits=extra["stopbits"], timeout=None)
        self._decoder = StreamDecoder(
            rssi_trailer=1 if self.config.rssi else 0)
        # drain stale boot bytes (module prints nothing, but USB re-enumeration can)
        self._rx_task = asyncio.create_task(self._read_loop(), name=f"lora-rx-{self.port}")

    async def disconnect(self) -> None:
        if self._rx_task:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
            self._rx_task = None
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = self._writer = None

    async def __aenter__(self) -> "LoRaDevice":
        await self.connect()
        return self

    async def __aexit__(self, *exc: object) -> None:
        await self.disconnect()

    # -- background reader ---------------------------------------------------
    async def _read_loop(self) -> None:
        assert self._reader is not None
        try:
            while True:
                chunk = await self._reader.read(1024)
                if not chunk:  # EOF / disconnect
                    await asyncio.sleep(0.05)
                    continue
                if self._at_mode:
                    self._at_buf += chunk
                    if self._looks_like_at_reply():
                        self._at_event.set()
                else:
                    for msg in self._decoder.feed(chunk):
                        try:
                            self.queue.put_nowait(msg)
                        except asyncio.QueueFull:
                            # drop oldest, keep newest (sensor link semantics)
                            try:
                                self.queue.get_nowait()
                            except asyncio.QueueEmpty:
                                pass
                            self.queue.put_nowait(msg)
        except asyncio.CancelledError:
            raise
        except Exception:
            return  # serial error ends loop; is_open reflects state

    def _looks_like_at_reply(self) -> bool:
        upper = bytes(self._at_buf).upper()
        return any(t in upper for t in (*_AT_OK_TOKENS, b"ERROR", b"ERR"))

    # -- low-level write -------------------------------------------------------
    async def _write(self, data: bytes) -> None:
        assert self._writer is not None, "not connected"
        async with self._write_lock:
            self._writer.write(data)
            await self._writer.drain()

    # -- AT session --------------------------------------------------------------
    async def _at(self, cmd: str, timeout: float | None = None) -> _at.AtReply:
        """Send one AT command, wait for OK/ERROR. Must be inside AT mode,
        except for the ``+++`` entry command itself."""
        timeout = timeout if timeout is not None else self.at_timeout
        self._at_buf = bytearray()
        self._at_event.clear()
        raw = _at.encode(cmd)
        # single write matters: '+++' sent char-by-char is ignored by firmware
        assert self._writer is not None
        async with self._write_lock:
            self._writer.write(raw)
            await self._writer.drain()
        try:
            await asyncio.wait_for(self._at_event.wait(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"AT timeout for {cmd!r}: got {bytes(self._at_buf)!r}")
        reply = _at.parse_reply(bytes(self._at_buf), expect=cmd)
        if not reply.ok:
            raise RuntimeError(f"AT {cmd!r} failed: {reply.raw!r}")
        return reply

    async def enter_at_mode(self, timeout: float | None = None) -> None:
        timeout = timeout if timeout is not None else self.at_timeout
        self._at_mode = True
        try:
            await self._at("+++", timeout=timeout)
        except Exception:
            self._at_mode = False
            raise

    async def exit_at_mode(self) -> None:
        try:
            await self._at(_at.AT_EXIT, timeout=5.0)
        finally:
            self._at_mode = False
        # module reboots on EXIT; give it time before data flows again
        await asyncio.sleep(3.2)
        # flush any reboot garbage so the frame scanner starts clean
        self._decoder = StreamDecoder(rssi_trailer=1 if self.config.rssi else 0)
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    async def get_version(self) -> str:
        was_in = self._at_mode
        if not was_in:
            await self.enter_at_mode()
        try:
            rep = await self._at(_at.AT_VER)
            return rep.value or rep.raw.strip()
        finally:
            if not was_in:
                await self.exit_at_mode()

    async def factory_reset(self) -> None:
        await self.apply_config(LoraConfig(band=self.config.band), restore_first=True)

    async def read_config(self) -> LoraConfig:
        """Read the live device config via ``AT+ALLP?`` (reboots on EXIT)."""
        was_in = self._at_mode
        if not was_in:
            await self.enter_at_mode()
        try:
            rep = await self._at(_at.AT_ALLP_QUERY, timeout=5.0)
            cfg = LoraConfig.from_allp(rep.raw, band=str(self.config.band))
            cfg.band = self.config.band  # band is host-side, not stored on device
            return cfg
        finally:
            if not was_in:
                await self.exit_at_mode()

    async def apply_config(self, cfg: LoraConfig | None = None,
                           restore_first: bool = False,
                           use_allp: bool = True) -> None:
        """Write *cfg* to the module. Uses one ``AT+AllP=`` write by default.

        WARNING: changing ``baud`` re-opens the host serial port at the new rate
        after the module reboots.
        """
        cfg = cfg or self.config
        cfg.validate()
        await self.enter_at_mode()
        try:
            if restore_first:
                try:
                    await self._at("AT+RESTORE=1")
                    await asyncio.sleep(3.2)
                    await self.enter_at_mode()
                except Exception:
                    pass  # older firmware may lack RESTORE reply; continue
            if use_allp:
                await self._at(cfg.to_allp(), timeout=5.0)
            else:
                for cmd in cfg.to_at_commands():
                    await self._at(cmd)
        finally:
            # EXIT even on failure so the module is never stuck in AT mode
            try:
                await self.exit_at_mode()
            except Exception:
                self._at_mode = False
        new_baud = int(cfg.baud)
        if new_baud != self.serial_baud:
            await self.disconnect()
            self.serial_baud = new_baud
            await self.connect()
        self.config = cfg
        self._decoder.rssi_trailer = 1 if cfg.rssi else 0

    # -- JSON transport ------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    async def send_json(self, obj: Any, dest_addr: int | None = None,
                        dest_channel: int | None = None,
                        pace: bool = True) -> int:
        """Encode + transmit a JSON object. Returns air bytes written.

        In packet mode *dest_addr*/*dest_channel* are required (3-byte prefix
        per fragment); in stream mode they must be None (transparent).
        """
        if self.packet_mode and (dest_addr is None or dest_channel is None):
            raise ValueError("packet mode requires dest_addr and dest_channel")
        if not self.packet_mode and (dest_addr is not None or dest_channel is not None):
            raise ValueError("dest_* only valid in packet mode (AT+MODE=2)")
        frags = encode_json(obj, self._next_seq(), mtu=self._air_mtu(),
                            compress_level=self.compress_level)
        return await self._send_fragments(frags, dest_addr, dest_channel, pace)

    async def send_raw(self, data: bytes, dest_addr: int | None = None,
                       dest_channel: int | None = None, pace: bool = True) -> int:
        frags = encode_raw(bytes(data), self._next_seq(), mtu=self._air_mtu(),
                           compress_level=self.compress_level)
        return await self._send_fragments(frags, dest_addr, dest_channel, pace)

    async def _send_fragments(self, frags: list[bytes], dest_addr: int | None,
                              dest_channel: int | None, pace: bool) -> int:
        prefix = packet_prefix(dest_addr, dest_channel) if self.packet_mode else b""
        total = 0
        for f in frags:
            await self._write(prefix + f)
            total += len(prefix) + len(f)
            if pace and len(frags) > 1:
                # airtime-proportional pacing: the radio (not USB) is the bottleneck
                ms = self.config.estimate_airtime_ms(len(f))
                await asyncio.sleep(min(2.0, ms / 1000.0 + 0.05))
        return total

    async def recv(self, timeout: float | None = None) -> DecodedMessage:
        try:
            return await asyncio.wait_for(self.queue.get(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f"no LoRa message within {timeout}s")

    async def recv_json(self, timeout: float | None = None) -> tuple[Any, Any]:
        msg = await self.recv(timeout)
        return msg.data, msg.meta

    async def messages(self):
        """Async iterator: ``async for data, meta in dev.messages(): ...``"""
        while True:
            msg = await self.queue.get()
            yield msg.data, msg.meta

    async def listen(self, callback: Callable[[Any, Any], Awaitable[None] | None]) -> None:
        """Run *callback(data, meta)* for every received message, forever."""
        async for data, meta in self.messages():
            res = callback(data, meta)
            if isinstance(res, Awaitable):
                await res

    # -- RSSI helper ---------------------------------------------------------
    @staticmethod
    def rssi_to_dbm(raw: int | None) -> float | None:
        """Rough SX1262 RSSI conversion. Returns None when RSSI disabled/unknown."""
        if raw is None:
            return None
        # SX1262 reports RSSI in 0.5 dB steps with an offset; exact mapping is
        # firmware-dependent, so expose raw + best-effort dBm.
        return -164.0 + raw / 2.0
