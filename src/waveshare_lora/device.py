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

Packet mode (AT+MODE=2) adds a 3-byte destination prefix to one bounded
frame::

    await dev.send_json({"cmd": "on"}, dest_addr=0x0005, dest_channel=18)

All serial I/O is ``asyncio``-native via ``pyserial-asyncio``; a background
reader task feeds :class:`codec.StreamDecoder` and publishes complete
messages on an :class:`asyncio.Queue`.
"""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Awaitable, Callable
from typing import Any

from . import at as _at
from .codec import (
    DEFAULT_MTU,
    HEADER_LEN,
    DecodedMessage,
    StreamDecoder,
    encode_json,
    encode_raw,
    packet_prefix,
)
from .config import (
    CACHE_BYTES,
    MAX_AIR_SINGLE,
    VALID_BAUDS,
    LoraConfig,
    OperationMode,
    comm_to_pyserial,
)

_AT_OK_TOKENS = (b"OK", b"+OK")
RADIO_CACHE_BYTES = CACHE_BYTES


class LoRaDevice:
    """Async USB-TO-LoRa endpoint.

    ``mtu`` caps application fragment size. ``tx_chunk_size`` also caps each
    serial write. Both are bounded by the 240-byte RF packet limit.
    """

    def __init__(
        self,
        port: str,
        config: LoraConfig | None = None,
        serial_baud: int | None = None,
        mtu: int = DEFAULT_MTU,
        queue_size: int = 128,
        compress_level: int = 9,
        at_timeout: float = 2.5,
        *,
        tx_chunk_size: int = RADIO_CACHE_BYTES,
    ) -> None:
        self.port = port
        self.config = config or LoraConfig()
        # Host-side serial baud: must equal the device's AT+BAUD while talking.
        self.serial_baud = int(self.config.baud if serial_baud is None else serial_baud)
        if self.serial_baud not in VALID_BAUDS:
            raise ValueError(f"serial_baud must be one of {VALID_BAUDS}")
        self.serial_comm = self.config.comm
        min_mtu = HEADER_LEN + 1 + (3 if self.packet_mode else 0)
        if not min_mtu <= mtu <= MAX_AIR_SINGLE:
            raise ValueError(f"mtu must be {min_mtu}..{MAX_AIR_SINGLE}")
        self.mtu = mtu
        min_chunk = HEADER_LEN + 1 + (3 if self.packet_mode else 0)
        if not min_chunk <= tx_chunk_size <= RADIO_CACHE_BYTES:
            raise ValueError(f"tx_chunk_size must be {min_chunk}..{RADIO_CACHE_BYTES}")
        self.tx_chunk_size = tx_chunk_size
        self.compress_level = compress_level
        self.at_timeout = at_timeout

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._rx_task: asyncio.Task | None = None
        self._write_lock = asyncio.Lock()
        self._io_lock = asyncio.Lock()
        self._io_owner: asyncio.Task | None = None
        self._io_depth = 0
        self._seq = 0
        self._decoder = self._new_decoder()
        self.queue: asyncio.Queue[DecodedMessage] = asyncio.Queue(maxsize=queue_size)

        # AT-session state (reader task routes bytes here while set)
        self._at_mode = False
        self._at_buf = bytearray()
        self._at_event = asyncio.Event()
        self._exit_command_confirmed = False

    # -- properties ------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return (
            self._writer is not None
            and not self._writer.is_closing()
            and self._reader is not None
            and self._rx_task is not None
            and not self._rx_task.done()
        )

    @property
    def packet_mode(self) -> bool:
        return int(self.config.mode) == int(OperationMode.PACKET)

    def _new_decoder(self) -> StreamDecoder:
        return StreamDecoder(rssi_trailer=1 if self.config.rssi else 0)

    async def _acquire_io(self) -> None:
        task = asyncio.current_task()
        if task is None:
            raise RuntimeError("serial I/O requires an asyncio task")
        if self._io_owner is task:
            self._io_depth += 1
            return
        await self._io_lock.acquire()
        self._io_owner = task
        self._io_depth = 1

    def _release_io(self) -> None:
        if self._io_owner is not asyncio.current_task():
            raise RuntimeError("serial I/O lock owned by another task")
        self._io_depth -= 1
        if self._io_depth == 0:
            self._io_owner = None
            self._io_lock.release()

    @contextlib.asynccontextmanager
    async def _io_guard(self):
        await self._acquire_io()
        try:
            yield
        finally:
            self._release_io()

    # -- lifecycle ---------------------------------------------------------
    async def connect(self) -> None:
        async with self._io_guard():
            if self.is_open:
                return
            if self._writer is not None:
                await self.disconnect()
            import serial_asyncio

            extra = comm_to_pyserial(self.serial_comm)
            extra.pop("_device_databits", None)
            self._reader, self._writer = await serial_asyncio.open_serial_connection(
                url=self.port,
                baudrate=self.serial_baud,
                bytesize=extra["bytesize"],
                parity=extra["parity"],
                stopbits=extra["stopbits"],
                timeout=None,
            )
            self._decoder = self._new_decoder()
            # Start reading serial data in the background.
            self._rx_task = asyncio.create_task(self._read_loop(), name=f"lora-rx-{self.port}")

    async def disconnect(self) -> None:
        async with self._io_guard():
            cancelled = False
            reader_error: Exception | None = None
            if self._rx_task:
                task = self._rx_task
                task.cancel()
                while not task.done():
                    try:
                        await asyncio.wait({task})
                    except asyncio.CancelledError:
                        cancelled = True
                if not task.cancelled():
                    try:
                        task.result()
                    except OSError:
                        pass
                    except Exception as exc:  # noqa: BLE001 - close port before re-raising
                        reader_error = exc
                self._rx_task = None
            if self._writer:
                try:
                    self._writer.close()
                    close_wait = asyncio.create_task(self._writer.wait_closed())
                    while not close_wait.done():
                        try:
                            await asyncio.shield(close_wait)
                        except asyncio.CancelledError:
                            cancelled = True
                    await close_wait
                except OSError:  # port already gone
                    pass
            self._reader = self._writer = None
            if reader_error is not None:
                raise reader_error
            if cancelled:
                raise asyncio.CancelledError

    async def _reopen_for_config(self, cfg: LoraConfig) -> bool:
        """Finish a UART setting change despite cancellation; report any cancellation."""
        cancelled = False
        while True:
            try:
                await self.disconnect()
                break
            except asyncio.CancelledError:
                cancelled = True
        self.serial_baud = int(cfg.baud)
        self.serial_comm = cfg.comm
        self.config = cfg
        while True:
            try:
                await self.connect()
                break
            except asyncio.CancelledError:
                cancelled = True
        return cancelled

    @staticmethod
    async def _wait_for_reboot() -> bool:
        """Wait through cancellation and report it to the caller."""
        wait = asyncio.create_task(asyncio.sleep(3.2))
        cancelled = False
        while not wait.done():
            try:
                await asyncio.shield(wait)
            except asyncio.CancelledError:
                cancelled = True
        await wait
        return cancelled

    async def __aenter__(self) -> LoRaDevice:  # noqa: PYI034
        await self.connect()
        return self

    async def __aexit__(self, *exc: object) -> None:
        await self.disconnect()

    # -- background reader ---------------------------------------------------
    async def _read_loop(self) -> None:
        assert self._reader is not None
        try:
            while True:
                try:
                    chunk = await asyncio.wait_for(self._reader.read(1024), timeout=1.0)
                except asyncio.TimeoutError:
                    chunk = b""  # tick the decoder so stalled frames can expire
                eof = not chunk and self._reader.at_eof()
                if self._at_mode:
                    if not chunk:
                        if eof:
                            return
                        continue
                    self._at_buf += chunk
                    if self._looks_like_at_reply():
                        self._at_event.set()
                else:
                    for msg in self._decoder.feed(chunk, final=eof):
                        try:
                            self.queue.put_nowait(msg)
                        except asyncio.QueueFull:
                            # drop oldest, keep newest (sensor link semantics)
                            with contextlib.suppress(asyncio.QueueEmpty):
                                self.queue.get_nowait()
                            self.queue.put_nowait(msg)
                if eof:
                    return
        except asyncio.CancelledError:
            raise
        except OSError:  # port gone / SerialException (subclass of OSError)
            return  # serial error ends loop; is_open reflects state

    def _looks_like_at_reply(self) -> bool:
        upper = bytes(self._at_buf).upper()
        # NOTE (HW Ver1.2): '+++' from data mode is echoed with NO OK/ERROR,
        # from AT mode it returns ERROR. Either way the echo means "heard".
        return any(t in upper for t in (*_AT_OK_TOKENS, b"ERROR", b"ERR", b"+++"))

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
        if self._io_owner is not asyncio.current_task():
            raise RuntimeError("AT command requires an active AT session")
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
        already_owned = self._at_mode and self._io_owner is asyncio.current_task()
        if not already_owned:
            await self._acquire_io()
        self._at_mode = True
        try:
            # HW Ver1.2: '+++' from data mode replies with a bare echo
            # (no OK), from AT mode with ERROR. Neither parses as ok, so
            # ignore its result and confirm AT mode with AT+VER instead.
            # (CancelledError still propagates to the cleanup below.)
            # Retry: the module may still be rebooting from a recent EXIT.
            last_exc: BaseException | None = None
            for attempt in range(3):
                try:
                    await self._at("+++", timeout=min(timeout, 1.0))
                except (TimeoutError, RuntimeError, OSError) as exc:
                    last_exc = exc
                try:
                    await self._at(_at.AT_VER, timeout=timeout)
                    last_exc = None
                    break
                except (TimeoutError, RuntimeError, OSError) as exc:
                    last_exc = exc
                    await asyncio.sleep(1.0)
            if last_exc is not None:
                raise last_exc
        except BaseException:
            self._at_mode = False
            if not already_owned:
                self._release_io()
            raise

    async def exit_at_mode(self) -> None:
        if self._io_owner is not asyncio.current_task() or not self._at_mode:
            raise RuntimeError("AT mode is owned by another task or is not active")
        try:
            self._exit_command_confirmed = False
            exit_error: BaseException | None = None
            try:
                await self._at(_at.AT_EXIT, timeout=5.0)
                self._exit_command_confirmed = True
            except (TimeoutError, RuntimeError, OSError, asyncio.CancelledError) as exc:
                exit_error = exc
            finally:
                self._at_mode = False
            # module reboots on EXIT; give it time before data flows again
            cancelled_during_wait = await self._wait_for_reboot()
            # flush any reboot garbage so the frame scanner starts clean
            self._decoder = self._new_decoder()
            while not self.queue.empty():
                try:
                    self.queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
            if exit_error is not None:
                raise exit_error
            if cancelled_during_wait:
                raise asyncio.CancelledError
        finally:
            self._release_io()

    async def get_version(self) -> str:
        was_in = self._at_mode and self._io_owner is asyncio.current_task()
        if not was_in:
            await self.enter_at_mode()
        try:
            rep = await self._at(_at.AT_VER)
        except BaseException:
            if not was_in:
                with contextlib.suppress(BaseException):
                    await self.exit_at_mode()
            raise
        if not was_in:
            await self.exit_at_mode()
        return rep.value or rep.raw.strip()

    async def factory_reset(self) -> None:
        await self.apply_config(LoraConfig(band=self.config.band), restore_first=True)

    async def read_config(self) -> LoraConfig:
        """Read the live device config via ``AT+ALLP?`` (reboots on EXIT)."""
        was_in = self._at_mode and self._io_owner is asyncio.current_task()
        if not was_in:
            await self.enter_at_mode()
        try:
            rep = await self._at(_at.AT_ALLP_QUERY, timeout=5.0)
            cfg = LoraConfig.from_allp(rep.raw, band=str(self.config.band))
            cfg.band = self.config.band  # band is host-side, not stored on device
        except BaseException:
            if not was_in:
                with contextlib.suppress(BaseException):
                    await self.exit_at_mode()
            raise
        if not was_in:
            await self.exit_at_mode()
        return cfg

    async def apply_config(
        self, cfg: LoraConfig | None = None, restore_first: bool = False, use_allp: bool = True
    ) -> None:
        """Write *cfg* to the module. Uses one ``AT+AllP=`` write by default.

        Changing ``baud`` or ``comm`` re-opens the host serial port after the
        module reboots.
        """
        cfg = cfg or self.config
        cfg.validate()
        if int(cfg.mode) == int(OperationMode.PACKET) and min(self.mtu, self.tx_chunk_size) < 21:
            raise ValueError("packet mode requires mtu and tx_chunk_size of at least 21 bytes")
        async with self._io_guard():
            if self._at_mode:
                raise RuntimeError("apply_config cannot run inside an AT session")
            await self.enter_at_mode()
            try:
                if restore_first:
                    try:
                        await self._at("AT+RESTORE=1")
                    except TimeoutError:
                        pass  # firmware may reboot without replying
                    cancelled_after_restore = await self._wait_for_reboot()
                    # RESTORE reboots into factory UART settings. The old AT
                    # session no longer has a usable serial connection.
                    self._at_mode = False
                    self._release_io()
                    defaults = LoraConfig(band=self.config.band)
                    cancelled_after_restore |= await self._reopen_for_config(defaults)
                    if cancelled_after_restore:
                        raise asyncio.CancelledError
                    await self.enter_at_mode()
                if use_allp:
                    await self._at(cfg.to_allp(), timeout=5.0)
                else:
                    for cmd in cfg.to_at_commands():
                        await self._at(cmd)
            except BaseException:
                # Preserve the write failure after attempting to leave AT mode.
                if self._at_mode:
                    with contextlib.suppress(BaseException):
                        await self.exit_at_mode()
                raise
            else:
                cancelled_after_exit = False
                try:
                    await self.exit_at_mode()
                except asyncio.CancelledError:
                    if not self._exit_command_confirmed:
                        raise
                    cancelled_after_exit = True
            new_baud = int(cfg.baud)
            new_comm = cfg.comm
            if new_baud != self.serial_baud or new_comm != self.serial_comm:
                cancelled_after_exit |= await self._reopen_for_config(cfg)
            else:
                self.config = cfg
                self._decoder = self._new_decoder()
            if cancelled_after_exit:
                raise asyncio.CancelledError

    # -- JSON transport ------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    async def send_json(
        self,
        obj: Any,
        dest_addr: int | None = None,
        dest_channel: int | None = None,
        pace: bool = True,
    ) -> int:
        """Encode + transmit one JSON message. Return bytes submitted to USB.

        In packet mode *dest_addr*/*dest_channel* are required. The message
        and routing prefix must fit in one RF packet (240 bytes total).
        """
        async with self._io_guard():
            if self._at_mode:
                raise RuntimeError("cannot send while in AT mode")
            self._validate_destination(dest_addr, dest_channel)
            mtu = min(self.mtu, self.tx_chunk_size, MAX_AIR_SINGLE) - (
                3 if self.packet_mode else 0
            )
            frames = encode_json(obj, self._next_seq(), mtu=mtu, compress_level=self.compress_level)
            return await self._send_frames(frames, dest_addr, dest_channel, pace)

    async def send_raw(
        self,
        data: bytes,
        dest_addr: int | None = None,
        dest_channel: int | None = None,
        pace: bool = True,
    ) -> int:
        async with self._io_guard():
            if self._at_mode:
                raise RuntimeError("cannot send while in AT mode")
            self._validate_destination(dest_addr, dest_channel)
            mtu = min(self.mtu, self.tx_chunk_size, MAX_AIR_SINGLE) - (
                3 if self.packet_mode else 0
            )
            frames = encode_raw(bytes(data), self._next_seq(), mtu=mtu, compress_level=self.compress_level)
            return await self._send_frames(frames, dest_addr, dest_channel, pace)

    def _validate_destination(self, dest_addr: int | None, dest_channel: int | None) -> None:
        if self.packet_mode and (dest_addr is None or dest_channel is None):
            raise ValueError("packet mode requires dest_addr and dest_channel")
        if not self.packet_mode and (dest_addr is not None or dest_channel is not None):
            raise ValueError("dest_* only valid in packet mode (AT+MODE=2)")

    def _estimate_batch_airtime_ms(self, size: int) -> float:
        """Estimate drain time using max-size packets, UART rate, and LBT delay."""
        serial_size = size
        airtime_ms = 0.0
        while size:
            packet_size = min(MAX_AIR_SINGLE, size)
            lbt_delay = 2000.0 if self.config.lbt else 0.0
            airtime_ms += self.config.estimate_airtime_ms(packet_size) + 50.0 + lbt_delay
            size -= packet_size
        comm = self.config.comm.upper()
        bits_per_byte = 1 + 8 + (comm[1] != "N") + max(1, int(comm[2]))
        serial_ms = serial_size * bits_per_byte * 1000.0 / self.serial_baud
        return max(airtime_ms, serial_ms)

    async def _send_frames(
        self, frames: list[bytes], dest_addr: int | None, dest_channel: int | None, pace: bool
    ) -> int:
        async with self._io_guard():
            if self._at_mode:
                raise RuntimeError("cannot send while in AT mode")
            prefix = packet_prefix(dest_addr, dest_channel) if self.packet_mode else b""
            if self.config.rssi and len(frames) > 1:
                raise ValueError("RSSI mode requires a message to fit in one RF packet")
            if self.packet_mode and len(frames) > 1:
                raise ValueError("packet-mode message must fit in one serial batch and RF packet")
            if (self.packet_mode or self.config.rssi) and not pace:
                raise ValueError("packet and RSSI modes require pacing between sends")
            total = 0
            for frame in frames:
                batch = prefix + frame
                if len(batch) > min(self.tx_chunk_size, MAX_AIR_SINGLE):
                    raise ValueError("frame exceeds serial batch or RF packet limit")
                await self._write(batch)
                total += len(batch)
                if pace:
                    # Pacing is an estimate, not a transmit-complete signal.
                    wait_ms = self._estimate_batch_airtime_ms(len(batch))
                    await asyncio.sleep(wait_ms / 1000.0)
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
