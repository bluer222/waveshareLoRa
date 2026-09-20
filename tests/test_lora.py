"""Tests for config validation, AT helpers and the binary JSON codec.

Hardware-free: the codec and AT layers are fully testable without a radio.
Device tests use a loopback-style byte check via StreamDecoder.
"""

import pytest

from waveshare_lora import at as AT
from waveshare_lora import codec as C
from waveshare_lora.config import BandwidthIndex, LoraConfig, OperationMode


def test_defaults_validate():
    cfg = LoraConfig()
    assert cfg.tx_freq_mhz() == 868  # HF default CH18
    lf = LoraConfig(band="LF", txch=23, rxch=23)
    assert lf.tx_freq_mhz() == 433


def test_rejects_bad_values():
    with pytest.raises(ValueError):
        LoraConfig(sf=13)
    with pytest.raises(ValueError):
        LoraConfig(txch=81)
    with pytest.raises(ValueError):
        LoraConfig(baud=12345)
    with pytest.raises(ValueError):
        LoraConfig(comm="7X9")


def test_at_commands_cover_all_params():
    cmds = LoraConfig().to_at_commands()
    joined = " ".join(cmds)
    for p in ("SF", "BW", "CR", "PWR", "NETID", "LBT", "MODE",
              "TXCH", "RXCH", "RSSI", "ADDR", "PORT", "BAUD", "COMM", "KEY"):
        assert f"AT+{p}=" in joined
    allp = LoraConfig().to_allp()
    assert allp.startswith("AT+AllP=7,125,")  # BW as kHz value, not index
    assert '"8N1"' in allp


def test_allp_roundtrip():
    cfg = LoraConfig(sf=9, txch=20, rxch=20, addr=5, key=1234, rssi=True,
                     mode=OperationMode.PACKET)
    back = LoraConfig.from_allp(cfg.to_allp())
    assert back.sf == 9 and back.txch == 20 and back.addr == 5
    assert back.key == 1234 and back.rssi is True
    assert int(back.bw) == 0 and back.bw_khz == 125


def test_bw_khz_accepted():
    assert LoraConfig(bw=500).bw == BandwidthIndex.KHZ_500
    assert LoraConfig(bw=1).bw == BandwidthIndex.KHZ_250


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


def test_codec_compression_saves_bytes():
    obj = {"sensor": "bme280", "readings": [21.5] * 30, "loc": "greenhouse-1"}
    import json
    plain = len(json.dumps(obj, separators=(",", ":")).encode())
    frags = C.encode_json(obj, seq=2)
    air = sum(len(f) for f in frags)
    assert air < plain  # zlib must win on repetitive JSON
    assert C.decode_fragments(frags).data == obj


def test_codec_fragmentation_and_stream_reassembly():
    import random
    rng = random.Random(1234)
    obj = {"blob": rng.randbytes(600).hex()}  # high-entropy: compresses poorly
    frags = C.encode_json(obj, seq=42)
    assert len(frags) > 1
    dec = C.StreamDecoder()
    out = []
    # feed byte-by-byte to prove split-read safety
    for f in frags:
        for i in range(0, len(f), 7):
            out += dec.feed(f[i:i + 7])
    assert len(out) == 1 and out[0].data == obj and out[0].meta.count == len(frags)


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
    addr, ch, body = C.packet_split(p + b"\xAA")
    assert (addr, ch, body) == (0xFFFE, 0x12, b"\xAA")


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
