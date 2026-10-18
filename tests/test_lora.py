"""Tests for config validation, AT helpers and the binary JSON codec.

Hardware-free: the codec and AT layers are fully testable without a radio.
Device tests use a loopback-style byte check via StreamDecoder.
"""

import asyncio
import contextlib
import json
import random

import pytest

from waveshare_lora import at as AT
from waveshare_lora import codec as C
from waveshare_lora.config import MAX_AIR_SINGLE, BandwidthIndex, LoraConfig, OperationMode
from waveshare_lora.device import RADIO_CACHE_BYTES, LoRaDevice


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

    assert cfg.to_allp() == 'AT+AllP=12,2,4,10,65535,1,3,80,0,1,65535,1,"9E2",14400,65535'
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
    assert allp.startswith("AT+AllP=7,0,")  # BW as index (AT+BW spelling), not kHz
    assert '"8N1"' in allp


def test_allp_roundtrip():
    cfg = LoraConfig(sf=9, txch=20, rxch=20, addr=5, key=1234, rssi=True, mode=OperationMode.PACKET)
    back = LoraConfig.from_allp(cfg.to_allp())
    assert back.sf == 9 and back.txch == 20 and back.addr == 5
    assert back.key == 1234 and back.rssi is True
    assert int(back.bw) == 0 and back.bw_khz == 125


def test_allp_parses_hardware_and_wiki_spellings():
    hw = '+ALLP=7,0,1,22,0,0,1,18,18,0,0,1,"8N1",115200,0'  # FW Ver1.2 capture
    cfg = LoraConfig.from_allp(hw)
    assert int(cfg.bw) == 0 and cfg.bw_khz == 125
    assert (cfg.comm, cfg.baud, cfg.port) == ("8N1", 115200, 1)
    assert cfg.to_allp() == f"AT+AllP={hw.split('=',1)[1]}"

    wiki = 'AT+AllP=7,125,1,22,0,0,1,18,18,0,0,3,115200,"8N1",0'  # wiki order/units
    wcfg = LoraConfig.from_allp(wiki)
    assert int(wcfg.bw) == 0 and (wcfg.comm, wcfg.baud, wcfg.port) == ("8N1", 115200, 3)


@pytest.mark.parametrize("bw,index", [("0", 0), ("1", 1), ("2", 2), ("250", 1), ("500", 2)])
def test_allp_parses_query_echo_and_trailing_ok(bw, index):
    reply = f'AT+ALLP?\r\n+allp=7,{bw},1,22,0,0,1,65,65,0,0,3,"8N1",115200,0\r\nOK\r\n'

    cfg = LoraConfig.from_allp(reply)

    assert int(cfg.bw) == index
    assert (cfg.txch, cfg.rxch, cfg.comm, cfg.baud) == (65, 65, "8N1", 115200)


@pytest.mark.parametrize("tail", ['"8N1","8N1"', "115200,9600"])
def test_allp_rejects_ambiguous_serial_tail(tail):
    with pytest.raises(ValueError, match="AllP tail"):
        LoraConfig.from_allp(f"7,0,1,22,0,0,1,65,65,0,0,3,{tail},0")


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


def test_codec_roundtrip_small():
    obj = {"temp": 21.5, "id": 7}
    frags = C.encode_json(obj, seq=1)
    assert len(frags) == 1 and len(frags[0]) < 240
    msg = C.decode_fragments(frags)
    assert msg.data == obj
    assert msg.meta.seq == 1


def test_wire_header_layout():
    frame = C.encode_raw(b"x", seq=0x1234)[0]

    assert C.HEADER_LEN == 17
    assert frame[:11] == bytes.fromhex("4c 4f 12 12 34 00 01 00 01 00 01")
    assert len(frame) == C.HEADER_LEN + 1
    assert frame[C.HEADER_LEN :] == b"x"
    assert C.decode_fragments([frame]).data == b"x"


def test_codec_compression_saves_bytes():
    obj = {"sensor": "bme280", "readings": [21.5] * 30, "loc": "greenhouse-1"}
    plain = len(json.dumps(obj, separators=(",", ":")).encode())
    frags = C.encode_json(obj, seq=2)
    air = sum(len(f) for f in frags)
    assert air < plain  # zlib must win on repetitive JSON
    assert C.decode_fragments(frags).data == obj


def test_large_message_uses_bounded_application_fragments():
    rng = random.Random(1234)
    obj = {"blob": rng.randbytes(600).hex()}  # high-entropy: compresses poorly
    frags = C.encode_json(obj, seq=42)
    assert len(frags) > 1
    assert all(len(frag) <= MAX_AIR_SINGLE for frag in frags)
    dec = C.StreamDecoder()
    out = []
    # feed byte-by-byte to prove split-read safety
    for f in frags:
        for i in range(0, len(f), 7):
            out += dec.feed(f[i : i + 7])
    assert len(out) == 1 and out[0].data == obj and out[0].meta.count == len(frags)


def test_encoder_keeps_mtu_argument_and_fragment_limit():
    data = random.Random(5).randbytes(500)
    keyword = C.encode_raw(data, seq=3, mtu=100, compress_level=0)
    positional = C.encode_raw(data, 3, 100, 0)

    assert keyword == positional
    assert all(len(frame) <= 100 for frame in keyword)
    assert C.decode_fragments(keyword).data == data
    assert C.StreamDecoder(mtu=100).max_room == 100 - C.HEADER_LEN


def test_codec_limits_decompressed_payload():
    oversized = b"A" * (C.MAX_DECODED + 1)
    with pytest.raises(ValueError, match="decoded message too large"):
        C.encode_raw(oversized, seq=1)

    compressed = C.zlib.compress(oversized)
    frame = bytearray(C.HEADER_LEN + len(compressed))
    C._pack_header(
        frame,
        (C.VERSION << 4) | C.FLAG_COMPRESSED | C.FLAG_RAW,
        1,
        len(compressed),
        0,
        1,
        len(compressed),
        C.crc32(compressed) & 0xFFFFFFFF,
    )
    frame[C.HEADER_LEN :] = compressed
    with pytest.raises(ValueError, match="invalid message payload"):
        C.decode_fragments([bytes(frame)])


def test_decode_fragments_rejects_short_fragment():
    frames = C.encode_raw(random.Random(5).randbytes(300), seq=1)
    with pytest.raises(ValueError, match="truncated fragment header"):
        C.decode_fragments([frames[0], b""])


def test_codec_rejects_reserved_flags_and_trailing_bytes():
    frame = C.encode_raw(b"ok", seq=1)[0]
    changed = bytearray(frame)
    h = C._unpack_header(frame)
    assert h is not None
    _, seq, total, idx, count, fl, msg_crc = h
    C._pack_header(changed, (C.VERSION << 4) | C.FLAG_RAW | 0x04, seq, total, idx, count, fl, msg_crc)

    assert C.StreamDecoder().feed(changed) == []
    with pytest.raises(ValueError, match="fragment length"):
        C.decode_fragments([frame + b"extra"])


def test_decoder_resyncs_after_corrupt_large_frame():
    broken = bytearray(C.encode_raw(b"x" * 500, seq=1)[0])
    broken[-1] ^= 0xFF
    next_frame = C.encode_json({"ok": True}, seq=2)[0]

    msgs = C.StreamDecoder().feed(bytes(broken) + next_frame)

    assert [msg.data for msg in msgs] == [{"ok": True}]


def test_decoder_recovers_when_a_large_frame_is_truncated():
    frame = C.encode_raw(random.Random(9).randbytes(4000), seq=1)[0]
    next_frame = C.encode_json({"ok": True}, seq=2)[0]
    dec = C.StreamDecoder()

    assert dec.feed(frame[:100]) == []
    assert dec.feed(next_frame) == []
    dec._incomplete_last_input -= dec.partial_timeout + 1
    msgs = dec.feed(b"")

    assert [msg.data for msg in msgs] == [{"ok": True}]


def test_truncated_fragment_does_not_block_continuous_messages():
    frame = C.encode_raw(random.Random(9).randbytes(4000), seq=1)[0]
    dec = C.StreamDecoder()

    assert dec.feed(frame[:100]) == []
    out = []
    for i in range(20):
        out.extend(dec.feed(C.encode_json({"n": i}, seq=i + 2)[0]))

    assert out and out[-1].data == {"n": 19}


def test_decoder_skips_impossible_single_frame_length():
    bad = bytearray(C.HEADER_LEN)
    C._pack_header(bad, C.VERSION << 4, 1, 1, 0, 1, 4000, 0)
    good = C.encode_json({"ok": True}, seq=2)[0]

    msgs = C.StreamDecoder().feed(bytes(bad) + good)

    assert [msg.data for msg in msgs] == [{"ok": True}]


def test_decoder_keeps_valid_nested_frame_in_raw_payload():
    inner = C.encode_raw(b"inner", seq=2)[0]
    outer = C.encode_raw(b"start" + inner + b"end", seq=1)[0]
    dec = C.StreamDecoder()

    assert dec.feed(outer[:-3]) == []
    msgs = dec.feed(outer[-3:])

    assert [msg.data for msg in msgs] == [b"start" + inner + b"end"]


@pytest.mark.asyncio
async def test_background_reader_recovers_after_idle():
    dev = LoRaDevice("unused")
    reader = asyncio.StreamReader()
    dev._reader = reader
    dev._decoder = C.StreamDecoder(partial_timeout=0.01)
    frame = C.encode_raw(random.Random(9).randbytes(4000), seq=1)[0]
    next_frame = C.encode_json({"ok": True}, seq=2)[0]
    reader.feed_data(frame[:100] + next_frame)
    task = asyncio.create_task(dev._read_loop())

    try:
        msg = await asyncio.wait_for(dev.queue.get(), timeout=1.5)
        assert msg.data == {"ok": True}
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task


@pytest.mark.asyncio
async def test_background_reader_recovers_complete_frame_at_eof():
    dev = LoRaDevice("unused")
    reader = asyncio.StreamReader()
    dev._reader = reader
    truncated = C.encode_raw(random.Random(9).randbytes(4000), seq=1)[0][:100]
    complete = C.encode_raw(b"ok", seq=2)[0]
    reader.feed_data(truncated + complete)
    reader.feed_eof()

    await dev._read_loop()

    assert dev.queue.get_nowait().data == b"ok"
    assert dev.queue.empty()


def test_decoder_reassembles_fragments():
    payload = b"x" * 300
    seq = 17
    count = 2
    msg_crc = C.crc32(payload) & 0xFFFFFFFF
    verflags = (C.VERSION << 4) | C.FLAG_RAW
    frames = []
    for idx, start in enumerate((0, 223)):
        chunk = payload[start : start + 223]
        frame = bytearray(C.HEADER_LEN + len(chunk))
        C._pack_header(frame, verflags, seq, len(payload), idx, count, len(chunk), msg_crc)
        frame[C.HEADER_LEN :] = chunk
        frames.append(bytes(frame))

    msgs = C.StreamDecoder().feed(b"".join(frames))

    assert len(msgs) == 1 and msgs[0].data == payload


def test_partial_timeout_resets_when_fragments_arrive():
    data = random.Random(5).randbytes(700)
    frames = C.encode_raw(data, seq=17)
    dec = C.StreamDecoder(partial_timeout=30)
    out = []

    for frame in frames:
        if 17 in dec._partials:
            dec._partials[17].t0 -= 20
        out.extend(dec.feed(frame))

    assert len(out) == 1 and out[0].data == data


def test_decoder_rejects_mixed_fragment_flags():
    frames = C.encode_raw(b'{"x":1}', seq=4, mtu=21, compress_level=0)
    mixed = [bytearray(frame) for frame in frames]
    h = C._unpack_header(mixed[-1])
    assert h is not None
    _, seq, total, idx, count, fl, msg_crc = h
    C._pack_header(mixed[-1], C.VERSION << 4, seq, total, idx, count, fl, msg_crc)

    with pytest.raises(ValueError, match="fragment set mismatch"):
        C.decode_fragments(mixed)
    assert C.StreamDecoder().feed(b"".join(mixed)) == []


def test_stream_send_batches_below_documented_cache(monkeypatch):
    frames = C.encode_raw(random.Random(7).randbytes(2500), seq=1)
    dev = LoRaDevice("unused", tx_chunk_size=RADIO_CACHE_BYTES)
    writes = []
    waits = []

    async def capture(data):
        writes.append(data)

    async def capture_wait(seconds):
        waits.append(seconds)

    monkeypatch.setattr(dev, "_write", capture)
    monkeypatch.setattr(asyncio, "sleep", capture_wait)

    written = asyncio.run(dev._send_frames(frames, None, None, pace=True))

    assert written == sum(map(len, frames))
    assert writes == frames
    assert all(len(chunk) <= MAX_AIR_SINGLE for chunk in writes)
    assert len(waits) == len(writes)
    assert all(wait > 0 for wait in waits)


def test_packet_mode_uses_one_bounded_batch(monkeypatch):
    raw = random.Random(8).randbytes(220)
    expected_frame = C.encode_raw(raw, seq=1)[0]
    dev = LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET), tx_chunk_size=960)
    writes = []
    waits = []

    async def capture(data):
        writes.append(data)

    async def capture_wait(seconds):
        waits.append(seconds)

    monkeypatch.setattr(dev, "_write", capture)
    monkeypatch.setattr(asyncio, "sleep", capture_wait)

    written = asyncio.run(dev.send_raw(raw, dest_addr=5, dest_channel=18))
    prefix = C.packet_prefix(5, 18)
    expected = prefix + expected_frame

    assert written == len(expected)
    assert writes == [expected]
    assert len(writes[0]) == MAX_AIR_SINGLE
    assert len(waits) == 1 and waits[0] > 0


def test_packet_mode_rejects_multi_packet_messages():
    dev = LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET))
    raw = random.Random(8).randbytes(221)

    with pytest.raises(ValueError, match="one serial batch and RF packet"):
        asyncio.run(dev.send_raw(raw, dest_addr=5, dest_channel=18))
    with pytest.raises(ValueError, match="require pacing"):
        asyncio.run(dev.send_raw(b"short", dest_addr=5, dest_channel=18, pace=False))


def test_packet_mode_requires_destination_for_raw_data():
    dev = LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET))

    with pytest.raises(ValueError, match="dest_addr"):
        asyncio.run(dev.send_raw(b"data"))


def test_rssi_mode_rejects_multi_packet_messages():
    dev = LoRaDevice("unused", config=LoraConfig(rssi=True))
    raw = random.Random(8).randbytes(300)

    with pytest.raises(ValueError, match="RSSI mode"):
        asyncio.run(dev.send_raw(raw))
    assert dev._decoder.max_room == MAX_AIR_SINGLE - C.HEADER_LEN
    with pytest.raises(ValueError, match="RSSI mode"):
        asyncio.run(LoRaDevice("unused", config=LoraConfig(rssi=True), tx_chunk_size=100).send_raw(raw[:100]))


def test_tx_chunk_size_cannot_exceed_radio_cache():
    with pytest.raises(ValueError, match="tx_chunk_size"):
        LoRaDevice("unused", tx_chunk_size=RADIO_CACHE_BYTES + 1)
    with pytest.raises(ValueError, match="tx_chunk_size"):
        LoRaDevice("unused", tx_chunk_size=C.HEADER_LEN)
    with pytest.raises(ValueError, match="tx_chunk_size"):
        LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET), tx_chunk_size=20)
    with pytest.raises(ValueError, match="serial_baud"):
        LoRaDevice("unused", serial_baud=12345)


def test_device_keeps_positional_mtu_argument(monkeypatch):
    dev = LoRaDevice("unused", None, None, 200)
    writes = []

    async def capture(data):
        writes.append(data)

    monkeypatch.setattr(dev, "_write", capture)
    asyncio.run(dev.send_raw(random.Random(4).randbytes(300), pace=False))

    assert len(writes) > 1
    assert all(len(frame) <= 200 for frame in writes)
    assert dev._decoder.max_room == MAX_AIR_SINGLE - C.HEADER_LEN


def test_packet_mode_requires_space_for_prefix_and_payload():
    with pytest.raises(ValueError, match="mtu"):
        LoRaDevice("unused", config=LoraConfig(mode=OperationMode.PACKET), mtu=20)
    dev = LoRaDevice("unused", mtu=18)
    with pytest.raises(ValueError, match="at least 21"):
        asyncio.run(dev.apply_config(LoraConfig(mode=OperationMode.PACKET)))


def test_batch_pacing_covers_airtime_serial_rate_and_lbt():
    cfg = LoraConfig(sf=12, lbt=True)
    dev = LoRaDevice("unused", config=cfg)
    expected = 4 * (cfg.estimate_airtime_ms(MAX_AIR_SINGLE) + 50 + 2000)

    assert dev._estimate_batch_airtime_ms(RADIO_CACHE_BYTES) == pytest.approx(expected)

    slow_serial = LoRaDevice("unused", serial_baud=1200)
    assert slow_serial._estimate_batch_airtime_ms(RADIO_CACHE_BYTES) >= 8000


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
@pytest.mark.parametrize("reply", [b"+++\r\n", b"ERROR\r\n", b"OK\r\n"])
async def test_at_entry_confirms_version_after_echo_or_error(monkeypatch, reply):
    dev = LoRaDevice("unused")
    commands = []

    async def at_command(cmd, **kwargs):
        commands.append(cmd)
        if cmd == "+++":
            dev._at_buf = bytearray(reply)
            assert dev._looks_like_at_reply()
            if not AT.parse_reply(reply).ok:
                raise RuntimeError("entry has no OK")
        return AT.AtReply(raw="OK", ok=True)

    monkeypatch.setattr(dev, "_at", at_command)

    await dev.enter_at_mode()

    assert commands == ["+++", AT.AT_VER]
    assert dev._at_mode and dev._io_owner is asyncio.current_task()
    dev._release_io()


@pytest.mark.asyncio
async def test_at_entry_retries_version_while_radio_reboots(monkeypatch):
    dev = LoRaDevice("unused")
    commands = []
    waits = []

    async def at_command(cmd, **kwargs):
        commands.append(cmd)
        if cmd == "+++":
            raise RuntimeError("entry echo has no OK")
        if commands.count(AT.AT_VER) < 3:
            raise TimeoutError("radio rebooting")
        return AT.AtReply(raw="OK", ok=True)

    async def no_wait(seconds):
        waits.append(seconds)

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)

    await dev.enter_at_mode()

    assert commands == ["+++", AT.AT_VER] * 3
    assert waits == [1.0, 1.0]
    assert dev._at_mode and dev._io_lock.locked()
    dev._release_io()


@pytest.mark.asyncio
async def test_failed_at_entry_releases_only_its_io_lock(monkeypatch):
    dev = LoRaDevice("unused")

    async def at_command(cmd, **kwargs):
        raise OSError("radio disconnected")

    async def no_wait(seconds):
        pass

    monkeypatch.setattr(dev, "_at", at_command)
    monkeypatch.setattr(asyncio, "sleep", no_wait)

    async with dev._io_guard():
        with pytest.raises(OSError, match="radio disconnected"):
            await dev.enter_at_mode()
        assert not dev._at_mode
        assert dev._io_owner is asyncio.current_task()
        assert dev._io_depth == 1

    assert not dev._io_lock.locked()


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

    assert commands == ["+++", AT.AT_VER, "AT+RESTORE=1", "+++", AT.AT_VER, cfg.to_allp(), AT.AT_EXIT]
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
    assert commands == ["+++", AT.AT_VER, "AT+RESTORE=1"]
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


def test_codec_crc_detects_corruption():
    frags = C.encode_json({"a": 1}, seq=3)
    bad = bytearray(frags[0])
    bad[-1] ^= 0xFF
    with pytest.raises(ValueError):
        C.decode_fragments([bytes(bad)])


def test_codec_merged_reads_and_garbage_resync():
    a = C.encode_json({"n": 1}, seq=10)[0]
    b = C.encode_json({"n": 2}, seq=11)[0]
    dec = C.StreamDecoder()
    msgs = dec.feed(b"\x00\xffgarbage" + a + b)
    assert [m.data["n"] for m in msgs] == [1, 2]


def test_packet_prefix_helpers():
    p = C.packet_prefix(0xFFFE, 0x12)
    assert p == bytes([0xFF, 0xFE, 0x12])
    addr, ch, body = C.packet_split(p + b"\xaa")
    assert (addr, ch, body) == (0xFFFE, 0x12, b"\xaa")


def test_rssi_trailer():
    frags = C.encode_json({"t": 1}, seq=5)
    dec = C.StreamDecoder(rssi_trailer=1)
    msgs = dec.feed(frags[0] + bytes([0xC8]))
    assert len(msgs) == 1 and msgs[0].meta.rssi == 0xC8


def test_raw_bytes_roundtrip():
    frags = C.encode_raw(b"\x00\x01\x02hello", seq=9)
    msg = C.decode_fragments(frags)
    assert msg.data == b"\x00\x01\x02hello" and msg.meta.raw is True


def test_relay_addr_helper():
    assert LoraConfig.relay_addr(0x03, 0x04) == 0x0304
