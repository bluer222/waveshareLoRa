"""Tests for config validation, AT helpers and the binary JSON codec.

Hardware-free: the codec and AT layers are fully testable without a radio.
Device tests capture serial writes instead of using a radio.
"""

import asyncio
import contextlib
import json
import random

import pytest

from waveshare_lora import at as AT
from waveshare_lora import codec as C
from waveshare_lora.config import MAX_AIR_SINGLE, BandwidthIndex, LoraConfig, OperationMode
from waveshare_lora.device import LoRaDevice


def test_defaults_validate():
    cfg = LoraConfig()
    assert cfg.tx_freq_mhz() == 868  # HF default CH18
    lf = LoraConfig(band="LF", txch=23, rxch=23)
    assert lf.tx_freq_mhz() == 433


def test_documented_parameter_boundaries():
    cfg = LoraConfig(
        sf=12,
        bw=500,
        cr=4,
        pwr=10,
        netid=65535,
        lbt=True,
        mode=3,
        txch=80,
        rxch=0,
        rssi=True,
        addr=65535,
        port=1,
        baud=14400,
        comm="9E2",
        key=65535,
        band="LF",
    )

    assert cfg.to_allp() == 'AT+AllP=12,500,4,10,65535,1,3,80,0,1,65535,1,14400,"9E2",65535'
    assert (cfg.tx_freq_mhz(), cfg.rx_freq_mhz()) == (490, 410)


def test_rejects_bad_values():
    with pytest.raises(ValueError):
        LoraConfig(sf=13)
    with pytest.raises(ValueError):
        LoraConfig(txch=81)
    with pytest.raises(ValueError):
        LoraConfig(port=4)
    with pytest.raises(ValueError):
        LoraConfig(baud=12345)
    with pytest.raises(ValueError):
        LoraConfig(comm="7X9")


def test_at_commands_cover_all_params():
    cmds = LoraConfig().to_at_commands()
    joined = " ".join(cmds)
    for p in (
        "SF",
        "BW",
        "CR",
        "PWR",
        "NETID",
        "LBT",
        "MODE",
        "TXCH",
        "RXCH",
        "RSSI",
        "ADDR",
        "PORT",
        "BAUD",
        "COMM",
        "KEY",
    ):
        assert f"AT+{p}=" in joined
    allp = LoraConfig().to_allp()
    assert allp.startswith("AT+AllP=7,125,")  # BW as kHz value, not index
    assert '"8N1"' in allp


def test_allp_roundtrip():
    cfg = LoraConfig(sf=9, txch=20, rxch=20, addr=5, key=1234, rssi=True, mode=OperationMode.PACKET)
    back = LoraConfig.from_allp(cfg.to_allp())
    assert back.sf == 9 and back.txch == 20 and back.addr == 5
    assert back.key == 1234 and back.rssi is True
    assert int(back.bw) == 0 and back.bw_khz == 125


def test_bw_khz_accepted():
    assert LoraConfig(bw=500).bw == BandwidthIndex.KHZ_500
    assert LoraConfig(bw=1).bw == BandwidthIndex.KHZ_250
    assert LoraConfig(baud=14400).baud == 14400


def test_at_reply_parsing():
    r = AT.parse_reply(b"AT+SF=7\r\nOK\r\n")
    assert r.ok
    r = AT.parse_reply(b"+SF:7\r\nOK\r\n", expect="AT+SF?")
    assert r.ok and r.value == "7"
    r = AT.parse_reply(b"ERROR\r\n")
    assert not r.ok
    assert AT.encode("+++") == b"+++\r\n"
    assert AT.encode("AT+SF=7") == b"AT+SF=7\r\n"


def test_packet_mode_requires_destination_for_raw_data():
    dev = LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET))

    with pytest.raises(ValueError, match="dest_addr"):
        asyncio.run(dev.send_raw(b"data"))


@pytest.mark.asyncio
@pytest.mark.parametrize("baud", [115200, 9600])
async def test_apply_config_reopens_with_new_serial_framing(monkeypatch, baud):
    dev = LoRaDevice("unused")
    cfg = LoraConfig(comm="8E1", baud=baud)
    calls = []

    async def enter():
        dev._at_mode = True

    async def at_command(*args, **kwargs):
        return None

    async def exit_mode():
        dev._at_mode = False

    async def disconnect():
        calls.append("disconnect")

    async def connect():
        calls.append((dev.serial_baud, dev.serial_comm, dev.config))

    monkeypatch.setattr(dev, "enter_at_mode", enter)
    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(dev, "exit_at_mode", exit_mode)
    monkeypatch.setattr(dev, "disconnect", disconnect)
    monkeypatch.setattr(dev, "connect", connect)

    await dev.apply_config(cfg)

    assert calls == ["disconnect", (baud, "8E1", cfg)]


@pytest.mark.asyncio
async def test_cancelled_at_entry_releases_io_lock(monkeypatch):
    dev = LoRaDevice("unused")
    started = asyncio.Event()

    async def blocked_at(*args, **kwargs):
        started.set()
        await asyncio.Event().wait()

    monkeypatch.setattr(dev, "_at", blocked_at)
    task = asyncio.create_task(dev.enter_at_mode())
    await started.wait()
    task.cancel()

    with pytest.raises(asyncio.CancelledError):
        await task
    assert not dev._at_mode and not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_restore_reenters_at_mode_after_missing_reply(monkeypatch):
    dev = LoRaDevice("unused", config=LoraConfig(baud=9600, comm="8E1"))
    cfg = LoraConfig(sf=9)
    commands = []
    reopened = []

    async def at_command(cmd, **kwargs):
        commands.append(cmd)
        if cmd == "AT+RESTORE=1":
            raise TimeoutError("no reply")
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        pass

    async def disconnect():
        reopened.append("disconnect")

    async def connect():
        reopened.append((dev.serial_baud, dev.serial_comm, dev.config))

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)
    monkeypatch.setattr(dev, "disconnect", disconnect)
    monkeypatch.setattr(dev, "connect", connect)

    await dev.apply_config(cfg, restore_first=True)

    assert commands == ["+++", "AT+RESTORE=1", "+++", cfg.to_allp(), AT.AT_EXIT]
    assert reopened[0] == "disconnect"
    assert reopened[1][0:2] == (115200, "8N1")
    assert reopened[1][2].baud == 115200
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_apply_config_reports_exit_failure(monkeypatch):
    dev = LoRaDevice("unused")
    original_cfg = dev.config

    async def at_command(cmd, **kwargs):
        if cmd == AT.AT_EXIT:
            raise TimeoutError("EXIT did not reply")
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        pass

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)

    with pytest.raises(TimeoutError, match="EXIT did not reply"):
        await dev.apply_config(LoraConfig(sf=9, comm="8E1"))
    assert dev.config is original_cfg
    assert dev.serial_comm == "8N1"
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_cancelled_exit_holds_lock_until_reboot_wait_finishes(monkeypatch):
    dev = LoRaDevice("unused")
    original_sleep = asyncio.sleep
    sleeping = asyncio.Event()
    release = asyncio.Event()

    async def at_command(cmd, **kwargs):
        return AT.AtReply(raw="OK", ok=True)

    async def reboot_sleep(seconds):
        sleeping.set()
        await release.wait()

    async def session():
        await dev.enter_at_mode()
        await dev.exit_at_mode()

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", reboot_sleep)
    task = asyncio.create_task(session())
    await sleeping.wait()
    task.cancel()
    await original_sleep(0)
    assert dev._io_lock.locked()
    release.set()

    with pytest.raises(asyncio.CancelledError):
        await task
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_cancelled_apply_config_resyncs_host_after_confirmed_exit(monkeypatch):
    dev = LoRaDevice("unused")
    cfg = LoraConfig(baud=9600, comm="8E1")
    original_sleep = asyncio.sleep
    sleeping = asyncio.Event()
    release = asyncio.Event()
    reopened = []

    async def at_command(cmd, **kwargs):
        return AT.AtReply(raw="OK", ok=True)

    async def reboot_sleep(seconds):
        sleeping.set()
        await release.wait()

    async def disconnect():
        reopened.append("disconnect")

    async def connect():
        reopened.append((dev.serial_baud, dev.serial_comm))

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(dev, "disconnect", disconnect)
    monkeypatch.setattr(dev, "connect", connect)
    monkeypatch.setattr(asyncio, "sleep", reboot_sleep)
    task = asyncio.create_task(dev.apply_config(cfg))
    await sleeping.wait()
    task.cancel()
    await original_sleep(0)
    assert dev._io_lock.locked()
    release.set()

    with pytest.raises(asyncio.CancelledError):
        await task
    assert reopened == ["disconnect", (9600, "8E1")]
    assert dev.config is cfg
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
@pytest.mark.parametrize("stage", ["disconnect", "connect"])
async def test_cancelled_serial_reopen_finishes_config(monkeypatch, stage):
    dev = LoRaDevice("unused")
    cfg = LoraConfig(baud=9600, comm="8E1")
    entered = asyncio.Event()
    calls = []
    blocked_once = False

    async def at_command(cmd, **kwargs):
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        pass

    async def maybe_block(name):
        nonlocal blocked_once
        if name == stage and not blocked_once:
            blocked_once = True
            entered.set()
            await asyncio.Event().wait()
        calls.append(name)

    async def disconnect():
        await maybe_block("disconnect")

    async def connect():
        await maybe_block("connect")
        calls.append((dev.serial_baud, dev.serial_comm))

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(dev, "disconnect", disconnect)
    monkeypatch.setattr(dev, "connect", connect)
    monkeypatch.setattr(asyncio, "sleep", no_wait)
    task = asyncio.create_task(dev.apply_config(cfg))
    await entered.wait()
    task.cancel()

    with pytest.raises(asyncio.CancelledError):
        await task
    assert calls == ["disconnect", "connect", (9600, "8E1")]
    assert dev.config is cfg
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_cancelled_restore_reopens_at_factory_settings(monkeypatch):
    dev = LoRaDevice("unused", config=LoraConfig(baud=9600, comm="8E1"))
    original_sleep = asyncio.sleep
    sleeping = asyncio.Event()
    release = asyncio.Event()
    commands = []
    reopened = []

    async def at_command(cmd, **kwargs):
        commands.append(cmd)
        return AT.AtReply(raw="OK", ok=True)

    async def reboot_sleep(seconds):
        sleeping.set()
        await release.wait()

    async def disconnect():
        reopened.append("disconnect")

    async def connect():
        reopened.append((dev.serial_baud, dev.serial_comm))

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(dev, "disconnect", disconnect)
    monkeypatch.setattr(dev, "connect", connect)
    monkeypatch.setattr(asyncio, "sleep", reboot_sleep)
    task = asyncio.create_task(dev.apply_config(LoraConfig(sf=9), restore_first=True))
    await sleeping.wait()
    task.cancel()
    await original_sleep(0)
    release.set()

    with pytest.raises(asyncio.CancelledError):
        await task
    assert commands == ["+++", "AT+RESTORE=1"]
    assert reopened == ["disconnect", (115200, "8N1")]
    assert dev.config.baud == 115200
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
@pytest.mark.parametrize("failure", ["error", "eof"])
async def test_connect_reopens_when_reader_has_stopped(monkeypatch, failure):
    dev = LoRaDevice("unused")
    closed = []

    class FakeWriter:
        def __init__(self):
            self.closed = False

        def is_closing(self):
            return self.closed

        def close(self):
            self.closed = True
            closed.append(True)

        async def wait_closed(self):
            pass

    async def stopped_reader():
        raise OSError("port disappeared")

    async def open_serial_connection(**kwargs):
        return asyncio.StreamReader(), FakeWriter()

    import serial_asyncio

    dev._reader = asyncio.StreamReader()
    dev._writer = FakeWriter()
    if failure == "error":
        dev._rx_task = asyncio.create_task(stopped_reader())
        with pytest.raises(OSError):
            await dev._rx_task
    else:
        dev._reader.feed_eof()
        dev._rx_task = asyncio.create_task(dev._read_loop())
        await dev._rx_task
    monkeypatch.setattr(serial_asyncio, "open_serial_connection", open_serial_connection)

    await dev.connect()

    assert closed
    assert dev.is_open
    await dev.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("method, command", [("get_version", AT.AT_VER), ("read_config", AT.AT_ALLP_QUERY)])
async def test_query_error_survives_exit_error(monkeypatch, method, command):
    dev = LoRaDevice("unused")

    async def at_command(cmd, **kwargs):
        if cmd == command:
            raise TimeoutError("query failed")
        if cmd == AT.AT_EXIT:
            raise RuntimeError("exit failed")
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        pass

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)

    with pytest.raises(TimeoutError, match="query failed"):
        await getattr(dev, method)()
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_other_task_cannot_exit_at_session(monkeypatch):
    dev = LoRaDevice("unused")
    entered = asyncio.Event()
    release = asyncio.Event()

    async def at_command(cmd, **kwargs):
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        pass

    async def owner():
        await dev.enter_at_mode()
        entered.set()
        await release.wait()
        await dev.exit_at_mode()

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)
    task = asyncio.create_task(owner())
    await entered.wait()

    with pytest.raises(RuntimeError, match="owned by another task"):
        await dev.exit_at_mode()
    assert dev._at_mode
    release.set()
    await task
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_disconnect_waits_for_send(monkeypatch):
    dev = LoRaDevice("unused")
    writing = asyncio.Event()
    release = asyncio.Event()
    closed = asyncio.Event()

    class FakeWriter:
        def close(self):
            closed.set()

        async def wait_closed(self):
            pass

    async def blocked_write(data):
        writing.set()
        await release.wait()

    dev._writer = FakeWriter()
    monkeypatch.setattr(dev, "_write", blocked_write)
    send = asyncio.create_task(dev.send_raw(b"message", pace=False))
    await writing.wait()
    disconnect = asyncio.create_task(dev.disconnect())

    with pytest.raises(asyncio.TimeoutError):
        await asyncio.wait_for(closed.wait(), timeout=0.05)
    release.set()
    await send
    await disconnect
    assert closed.is_set()


@pytest.mark.asyncio
async def test_disconnect_finishes_cleanup_after_cancellation():
    dev = LoRaDevice("unused")
    closing = asyncio.Event()
    close_waiter = asyncio.get_running_loop().create_future()

    class FakeWriter:
        def close(self):
            closing.set()

        async def wait_closed(self):
            await close_waiter

    dev._writer = FakeWriter()
    task = asyncio.create_task(dev.disconnect())
    await closing.wait()
    task.cancel()
    await asyncio.sleep(0)
    assert not close_waiter.cancelled()
    close_waiter.set_result(None)

    with pytest.raises(asyncio.CancelledError):
        await task
    assert dev._writer is None
    assert not dev._io_lock.locked()


@pytest.mark.asyncio
async def test_at_session_waits_for_paced_send(monkeypatch):
    dev = LoRaDevice("unused")
    writing = asyncio.Event()
    release = asyncio.Event()
    entered = asyncio.Event()

    async def blocked_write(data):
        writing.set()
        await release.wait()

    async def at_command(*args, **kwargs):
        return AT.AtReply(raw="OK", ok=True)

    async def at_session():
        await dev.enter_at_mode()
        entered.set()
        await dev.exit_at_mode()

    monkeypatch.setattr(dev, "_write", blocked_write)
    monkeypatch.setattr(dev, "_at", at_command)
    send = asyncio.create_task(dev.send_raw(b"message", pace=False))
    await writing.wait()
    session = asyncio.create_task(at_session())

    with pytest.raises(asyncio.TimeoutError):
        await asyncio.wait_for(entered.wait(), timeout=0.05)
    release.set()
    await send
    await asyncio.wait_for(entered.wait(), timeout=1.0)
    await session
    assert not dev._io_lock.locked()


def test_packet_prefix_helpers():
    p = C.packet_prefix(0xFFFE, 0x12)
    assert p == bytes([0xFF, 0xFE, 0x12])
    addr, ch, body = C.packet_split(p + b"\xaa")
    assert (addr, ch, body) == (0xFFFE, 0x12, b"\xaa")


def test_relay_addr_helper():
    assert LoraConfig.relay_addr(0x03, 0x04) == 0x0304


# -- codec: one frame per RF packet ------------------------------------------


def test_codec_roundtrip_small():
    obj = {"temp": 21.5, "id": 7}
    frame = C.encode_json(obj, seq=1)
    assert len(frame) < C.MAX_FRAME
    msg = C.decode_frame(frame)
    assert msg.data == obj
    assert msg.meta.seq == 1 and msg.meta.size == len(frame)


def test_wire_header_layout():
    frame = C.encode_raw(b"x", seq=0x1234, compress_level=0)
    assert C.HEADER_LEN == 10
    assert frame[:6] == bytes.fromhex("4c 4f 22 12 34 01")
    assert int.from_bytes(frame[6:10], "big") == C.crc32(frame[2:6] + b"x")
    assert frame[C.HEADER_LEN:] == b"x"
    assert C.decode_frame(frame).data == b"x"


def test_codec_compression_uses_raw_deflate():
    obj = {"sensor": "bme280", "readings": [21.5] * 30, "loc": "greenhouse-1"}
    plain = len(json.dumps(obj, separators=(",", ":")).encode())
    frame = C.encode_json(obj, seq=2)
    assert len(frame) - C.HEADER_LEN < plain
    assert frame[2] & C.FLAG_COMPRESSED
    assert frame[C.HEADER_LEN:] == C._deflate(json.dumps(obj, separators=(",", ":")).encode(), 9)
    assert C.decode_frame(frame).data == obj


def test_oversized_messages_are_rejected_not_split():
    big = random.Random(1234).randbytes(400).hex()
    with pytest.raises(C.MessageTooLarge):
        C.encode_json(big, seq=1)
    assert not C.fits_json(big)
    assert C.fits_json({"a": 1})
    exact = random.Random(2).randbytes(C.MAX_PAYLOAD)
    assert len(C.encode_raw(exact, seq=1)) == C.MAX_FRAME
    with pytest.raises(C.MessageTooLarge):
        C.encode_raw(exact + b"!", seq=1)
    assert len(C.encode_raw(exact[:90], seq=1, max_frame=100)) == 100
    with pytest.raises(ValueError, match="max_frame"):
        C.encode_raw(b"x", seq=1, max_frame=C.MAX_FRAME + 1)


def test_decode_frame_rejects_damage_and_extra_bytes():
    frame = C.encode_json({"a": 1}, seq=3)
    bad = bytearray(frame)
    bad[-1] ^= 0xFF
    with pytest.raises(ValueError, match="CRC"):
        C.decode_frame(bytes(bad))
    with pytest.raises(ValueError, match="length"):
        C.decode_frame(frame + b"extra")
    reserved = bytearray(frame)
    reserved[2] |= 0x04
    with pytest.raises(ValueError, match="header"):
        C.decode_frame(bytes(reserved))


def test_decoder_handles_split_and_merged_reads():
    frames = [C.encode_json({"n": n}, seq=n) for n in range(5)]
    stream = b"\x00\xffgarbage" + b"".join(frames)
    dec = C.StreamDecoder()
    out = []
    for i in range(0, len(stream), 7):
        out += dec.feed(stream[i : i + 7])
    assert [m.data["n"] for m in out] == list(range(5))


def test_decoder_resyncs_after_corrupt_frame():
    broken = bytearray(C.encode_raw(b"x" * 200, seq=1, compress_level=0))
    broken[-1] ^= 0xFF
    good = C.encode_json({"ok": True}, seq=2)
    assert [m.data for m in C.StreamDecoder().feed(bytes(broken) + good)] == [{"ok": True}]


def test_decoder_does_not_wait_on_a_truncated_frame_once_a_later_one_arrives():
    truncated = C.encode_raw(random.Random(9).randbytes(200), seq=1)[:40]
    dec = C.StreamDecoder()
    assert dec.feed(truncated) == []
    out = dec.feed(C.encode_json({"ok": True}, seq=2))
    assert [m.data for m in out] == [{"ok": True}]


def test_decoder_skips_a_false_magic_claiming_a_long_payload():
    fake = C.MAGIC_BYTES + bytes([C.VERSION << 4, 0, 1, 200, 0, 0, 0, 0])
    good = C.encode_json({"ok": True}, seq=2)
    assert [m.data for m in C.StreamDecoder().feed(fake + good)] == [{"ok": True}]


def test_decoder_final_discards_incomplete_frame():
    complete = C.encode_raw(b"ok", seq=2)
    truncated = C.encode_raw(random.Random(3).randbytes(200), seq=3)[:50]
    dec = C.StreamDecoder()
    assert [m.data for m in dec.feed(complete + truncated)] == [b"ok"]
    assert dec.feed(b"", final=True) == []
    assert not dec._buf


def test_rssi_trailer_follows_each_frame():
    a = C.encode_json({"t": 1}, seq=5)
    b = C.encode_json({"t": 2}, seq=6)
    dec = C.StreamDecoder(rssi_trailer=1)
    msgs = dec.feed(a + bytes([0xC8]) + b)
    assert [(m.data, m.meta.rssi) for m in msgs] == [({"t": 1}, 0xC8)]
    msgs = dec.feed(bytes([0x90]))
    assert [(m.data, m.meta.rssi) for m in msgs] == [({"t": 2}, 0x90)]


def test_raw_bytes_roundtrip():
    msg = C.decode_frame(C.encode_raw(b"\x00\x01\x02hello", seq=9))
    assert msg.data == b"\x00\x01\x02hello" and msg.meta.raw is True


def test_decompressed_size_is_limited():
    bomb = C._deflate(b"A" * (C.MAX_DECODED + 1), 9)
    assert len(bomb) <= C.MAX_PAYLOAD
    verflags = (C.VERSION << 4) | C.FLAG_COMPRESSED | C.FLAG_RAW
    checked = C.struct.pack("!BHB", verflags, 1, len(bomb)) + bomb
    frame = C._HEADER.pack(C.MAGIC_BYTES, verflags, 1, len(bomb), C.crc32(checked)) + bomb
    with pytest.raises(ValueError, match="invalid payload"):
        C.decode_frame(frame)


# -- device: one write and one pacing wait per message -----------------------


def _capture(monkeypatch, dev):
    writes, waits = [], []

    async def write(data):
        writes.append(data)

    async def wait(seconds):
        waits.append(seconds)

    monkeypatch.setattr(dev, "_write", write)
    monkeypatch.setattr(asyncio, "sleep", wait)
    return writes, waits


def test_send_writes_one_frame_and_paces_once(monkeypatch):
    dev = LoRaDevice("unused")
    writes, waits = _capture(monkeypatch, dev)
    written = asyncio.run(dev.send_json({"n": 1}))
    assert writes == [C.encode_json({"n": 1}, seq=1)] and written == len(writes[0])
    assert waits == [pytest.approx(dev._estimate_send_ms(len(writes[0])) / 1000)]
    with pytest.raises(C.MessageTooLarge):
        asyncio.run(dev.send_raw(random.Random(1).randbytes(300)))
    assert len(writes) == 1


def test_packet_mode_prefix_counts_against_the_packet(monkeypatch):
    dev = LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET))
    writes, waits = _capture(monkeypatch, dev)
    raw = random.Random(8).randbytes(C.MAX_PAYLOAD - 3)
    written = asyncio.run(dev.send_raw(raw, dest_addr=5, dest_channel=18))
    assert writes == [C.packet_prefix(5, 18) + C.encode_raw(raw, seq=1)]
    assert written == MAX_AIR_SINGLE and len(waits) == 1
    assert dev.frame_limit == MAX_AIR_SINGLE - 3
    with pytest.raises(C.MessageTooLarge):
        asyncio.run(dev.send_raw(raw + b"!", dest_addr=5, dest_channel=18))
    with pytest.raises(ValueError, match="require pacing"):
        asyncio.run(dev.send_raw(b"short", dest_addr=5, dest_channel=18, pace=False))


def test_device_fits_matches_its_frame_limit():
    dev = LoRaDevice("unused", max_frame=100)
    assert dev.fits({"a": 1})
    assert not dev.fits(random.Random(5).randbytes(200).hex())
    with pytest.raises(ValueError, match="max_frame"):
        LoRaDevice("unused", max_frame=MAX_AIR_SINGLE + 1)
    with pytest.raises(ValueError, match="max_frame"):
        LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET), max_frame=C.HEADER_LEN + 1)
    with pytest.raises(ValueError, match="serial_baud"):
        LoRaDevice("unused", serial_baud=12345)
    small = LoRaDevice("unused", max_frame=C.HEADER_LEN + 1)
    with pytest.raises(ValueError, match="at least 14"):
        asyncio.run(small.apply_config(LoraConfig(mode=OperationMode.PACKET)))


def test_pacing_covers_airtime_serial_rate_and_lbt():
    cfg = LoraConfig(sf=12, lbt=True)
    dev = LoRaDevice("unused", config=cfg)
    assert dev._estimate_send_ms(MAX_AIR_SINGLE) == pytest.approx(cfg.estimate_airtime_ms(MAX_AIR_SINGLE) + 50 + 2000)
    slow_serial = LoRaDevice("unused", serial_baud=1200, config=LoraConfig(sf=7))
    assert slow_serial._estimate_send_ms(MAX_AIR_SINGLE) == pytest.approx(MAX_AIR_SINGLE * 10 * 1000 / 1200)


@pytest.mark.asyncio
async def test_background_reader_delivers_frames_and_skips_a_truncated_one():
    dev = LoRaDevice("unused")
    reader = asyncio.StreamReader()
    dev._reader = reader
    truncated = C.encode_raw(random.Random(9).randbytes(200), seq=1)[:60]
    reader.feed_data(truncated + C.encode_raw(b"ok", seq=2))
    reader.feed_eof()
    await dev._read_loop()
    assert dev.queue.get_nowait().data == b"ok"
    assert dev.queue.empty()

