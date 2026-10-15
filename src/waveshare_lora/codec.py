"""Minimal-bytes async-friendly JSON framing for the SX1262 USB-TO-LoRa link.

Wire format (big-endian, ``struct.pack``-packed, all C-speed):

.. code-block:: text

    +-------+----------+------+------+-----+-------+----------+---------+---------+-----------+
    | MAGIC | VERFLAGS | SEQ  | TOTAL| IDX | COUNT | FRAG_LEN | HDR_CRC | MSG_CRC | PAYLOAD   |
    | 2B    | 1B       | 2B   | 2B   | 1B  | 1B    | 2B       | 2B      | 4B      | FRAG_LEN B|
    +-------+----------+------+------+-----+-------+----------+---------+---------+-----------+

* ``MAGIC`` = ``4C 4F`` (``'LO'``) for resync scanning in stream mode.
* ``VERFLAGS`` = ``(VER << 4) | FLAGS``; ``VER`` = 1. ``FLAGS`` bit0 = zlib
  compressed, bit1 = raw-bytes (not JSON). Other bits reserved (0).
* ``SEQ`` = uint16 message sequence (wraps).
* ``TOTAL`` = compressed-domain message length, max 65535.
* ``IDX``/``COUNT`` = fragment index / count (up to 255 fragments).
* ``FRAG_LEN`` = this frame's payload length.
* ``HDR_CRC`` = CRC16-CCITT over the first 11 header bytes. Together with the
  2-byte magic this rejects false resync hits inside payload data (~2^-32
  per position), so arbitrary binary survives transparent stream mode.
* ``MSG_CRC`` = CRC32 of the full reassembled payload (compressed domain).
* ``PAYLOAD`` = a slice of ``zlib.compress(compact_json)`` (or raw compact JSON
  if compression did not shrink it).

Why this is fast *and* small:

* ``json.dumps(..., separators=(",", ":"))`` (or ``orjson`` when installed)
  removes all insignificant whitespace.
* ``zlib`` (C implementation) typically cuts JSON 50–80 %; the flag bit means
  incompressible payloads never grow.
* ``struct.Struct.pack_into`` / ``unpack_from`` keeps the header compact;
  CRCs use C ``binascii``.
* Each application fragment fits in one 240-byte RF packet. Firmware packet
  boundaries still need hardware verification when RSSI output is enabled.
"""

from __future__ import annotations

import json
import struct
import zlib
from binascii import crc32, crc_hqx
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any

try:  # optional C-speed JSON
    import orjson as _orjson
except ImportError:  # pragma: no cover
    _orjson = None


MAGIC: int = 0x4C4F  # 'LO'
MAGIC_BYTES: bytes = b"\x4c\x4f"
VERSION: int = 1
FLAG_COMPRESSED: int = 0x01
FLAG_RAW: int = 0x02

_PREFIX = struct.Struct("!HBHHBBH")  # 11 B: magic, verflags, seq, total, idx, count, frag_len
_TAIL = struct.Struct("!HI")  # 6 B: header CRC16, message CRC32
HEADER_LEN: int = _PREFIX.size + _TAIL.size  # 17
_IDX_OFF: int = 7  # byte offset of IDX inside a packed fragment
MAX_TOTAL: int = 0xFFFF
MAX_FRAGMENTS: int = 0xFF
DEFAULT_MTU: int = 240
MAX_ROOM: int = DEFAULT_MTU - HEADER_LEN
MAX_DECODED: int = 1_048_576


def _dump_json(obj: Any) -> bytes:
    if _orjson is not None:
        return _orjson.dumps(obj)  # already compact
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _load_json(data: bytes) -> Any:
    if _orjson is not None:
        return _orjson.loads(data)
    return json.loads(data.decode("utf-8"))


@dataclass
class FrameMeta:
    seq: int = 0
    count: int = 1
    total: int = 0  # compressed-domain bytes
    compressed: bool = False
    raw: bool = False  # True = bytes payload, not JSON
    json_bytes: int = 0  # uncompressed JSON bytes (0 for raw)
    rssi: int | None = None  # filled by transport when AT+RSSI=1
    src_addr: int | None = None
    channel: int | None = None


@dataclass
class DecodedMessage:
    data: Any  # dict/list/... (or bytes when FLAG_RAW)
    meta: FrameMeta


def encode_json(
    obj: Any, seq: int, mtu: int = DEFAULT_MTU, compress_level: int = 9
) -> list[bytes]:
    """Encode *obj* as bounded fragments (up to 1 MiB before compression)."""
    plain = _dump_json(obj)
    if len(plain) > MAX_DECODED:
        raise ValueError(f"decoded message too large: {len(plain)} > {MAX_DECODED}")
    comp = zlib.compress(plain, compress_level)
    if len(comp) < len(plain):
        payload, compressed = comp, True
    else:
        payload, compressed = plain, False
    return _fragment(payload, seq, compressed=compressed, raw=False, mtu=mtu)


def fits_one_frame(obj: Any, mtu: int = DEFAULT_MTU, compress_level: int = 9) -> bool:
    """Whether *obj* encodes to a single fragment, which travels as one RF packet.

    A one-fragment message survives on its own: losing another packet cannot
    make it undecodable. Callers can pack independent data into such messages.
    """
    try:
        return len(encode_json(obj, 0, mtu=mtu, compress_level=compress_level)) == 1
    except ValueError:
        return False


def encode_raw(
    data: bytes, seq: int, mtu: int = DEFAULT_MTU, compress_level: int = 9
) -> list[bytes]:
    """Encode opaque bytes as bounded fragments (up to 1 MiB)."""
    if len(data) > MAX_DECODED:
        raise ValueError(f"decoded message too large: {len(data)} > {MAX_DECODED}")
    plain = bytes(data)
    comp = zlib.compress(plain, compress_level)
    if len(comp) < len(plain):
        payload, compressed = comp, True
    else:
        payload, compressed = plain, False
    return _fragment(payload, seq, compressed=compressed, raw=True, mtu=mtu)


def _pack_header(
    buf: bytearray,
    verflags: int,
    seq: int,
    total: int,
    idx: int,
    count: int,
    frag_len: int,
    msg_crc: int,
) -> None:
    _PREFIX.pack_into(buf, 0, MAGIC, verflags, seq, total, idx, count, frag_len)
    hdr_crc = crc_hqx(bytes(buf[: _PREFIX.size]), 0xFFFF) & 0xFFFF
    _TAIL.pack_into(buf, _PREFIX.size, hdr_crc, msg_crc)


def _unpack_header(
    buf: bytes | bytearray | memoryview, off: int = 0, max_room: int = MAX_TOTAL
) -> tuple[int, int, int, int, int, int, int] | None:
    """Unpack + fully validate a header. Returns
    ``(verflags, seq, total, idx, count, frag_len, msg_crc)`` or None."""
    if len(buf) - off < HEADER_LEN:
        return None
    try:
        magic, verflags, seq, total, idx, count, frag_len = _PREFIX.unpack_from(buf, off)
    except struct.error:
        return None
    if magic != MAGIC or (verflags >> 4) != VERSION:
        return None
    if verflags & ~(VERSION << 4 | FLAG_COMPRESSED | FLAG_RAW):
        return None
    if count == 0 or count > MAX_FRAGMENTS or idx >= count:
        return None
    if total > MAX_TOTAL or frag_len > max_room:
        return None
    if frag_len > total or (count == 1 and frag_len != total):
        return None
    try:
        hdr_crc, msg_crc = _TAIL.unpack_from(buf, off + _PREFIX.size)
    except struct.error:
        return None
    if (crc_hqx(bytes(buf[off : off + _PREFIX.size]), 0xFFFF) & 0xFFFF) != hdr_crc:
        return None
    return verflags, seq, total, idx, count, frag_len, msg_crc


def _fragment(payload: bytes, seq: int, compressed: bool, raw: bool, mtu: int) -> list[bytes]:
    if len(payload) > MAX_TOTAL:
        raise ValueError(f"message too large: {len(payload)} > {MAX_TOTAL}")
    room = mtu - HEADER_LEN
    if room <= 0 or mtu > DEFAULT_MTU:
        raise ValueError(f"mtu must be {HEADER_LEN + 1}..{DEFAULT_MTU}")
    flags = (FLAG_COMPRESSED if compressed else 0) | (FLAG_RAW if raw else 0)
    verflags = ((VERSION & 0x0F) << 4) | (flags & 0x0F)
    seq &= 0xFFFF
    total = len(payload)
    msg_crc = crc32(payload) & 0xFFFFFFFF
    count = max(1, -(-total // room))
    if count > MAX_FRAGMENTS:
        raise ValueError(f"needs {count} fragments > {MAX_FRAGMENTS}")
    frames: list[bytes] = []
    for idx in range(count):
        part = payload[idx * room : (idx + 1) * room]
        buf = bytearray(HEADER_LEN + len(part))
        _pack_header(buf, verflags, seq, total, idx, count, len(part), msg_crc)
        buf[HEADER_LEN:] = part
        frames.append(bytes(buf))
    return frames


def decode_fragments(fragments: Iterable[bytes]) -> DecodedMessage:
    """Decode a complete fragment set (single message) -> DecodedMessage."""
    frags = list(fragments)
    if not frags:
        raise ValueError("no fragments")
    h = _unpack_header(frags[0])
    if h is None:
        raise ValueError("bad header / CRC")
    verflags, seq, total, _idx, count, _fl, msg_crc = h
    if len(frags) != count:
        raise ValueError(f"need {count} fragments, got {len(frags)}")
    if any(len(f) < HEADER_LEN for f in frags):
        raise ValueError("truncated fragment header")
    ordered = sorted(frags, key=lambda f: f[_IDX_OFF])
    body = bytearray(total)
    pos = 0
    for expected_idx, f in enumerate(ordered):
        hh = _unpack_header(f)
        if hh is None:
            raise ValueError("bad fragment header / CRC")
        v, s, t, idx, c, fl, crc = hh
        if (v, s, t, c, crc) != (verflags, seq, total, count, msg_crc) or idx != expected_idx:
            raise ValueError("fragment set mismatch or duplicate index")
        if len(f) != HEADER_LEN + fl:
            raise ValueError("fragment length does not match header")
        body[pos : pos + fl] = f[HEADER_LEN : HEADER_LEN + fl]
        pos += fl
    if pos != total:
        raise ValueError(f"reassembled {pos} != TOTAL {total}")
    if (crc32(body) & 0xFFFFFFFF) != msg_crc:
        raise ValueError("CRC32 mismatch — corrupted LoRa packet")
    msg = _make_message(seq, verflags, total, count, msg_crc, bytes(body))
    if msg is None:
        raise ValueError("invalid message payload or CRC")
    return msg


def _make_message(
    seq: int,
    verflags: int,
    total: int,
    count: int,
    msg_crc: int,
    body: bytes,
    rssi: int | None = None,
) -> DecodedMessage | None:
    if len(body) != total or (crc32(body) & 0xFFFFFFFF) != msg_crc:
        return None
    flags = verflags & 0x0F
    compressed, raw = bool(flags & FLAG_COMPRESSED), bool(flags & FLAG_RAW)
    try:
        if compressed:
            decoder = zlib.decompressobj()
            payload = decoder.decompress(body, MAX_DECODED + 1)
            if len(payload) > MAX_DECODED or not decoder.eof or decoder.unused_data:
                return None
        else:
            payload = body
        data = payload if raw else _load_json(payload)
    except (ValueError, zlib.error):
        return None
    meta = FrameMeta(
        seq=seq,
        count=count,
        total=total,
        compressed=compressed,
        raw=raw,
        json_bytes=0 if raw else len(payload),
        rssi=rssi,
    )
    return DecodedMessage(data=data, meta=meta)


# ---------------------------------------------------------------------------
# Streaming reassembler — feed() arbitrary serial chunks, get messages back
# ---------------------------------------------------------------------------


@dataclass
class _Partial:
    verflags: int
    total: int
    count: int
    msg_crc: int
    parts: dict[int, bytes] = field(default_factory=dict)
    t0: float = 0.0
    rssi: int | None = None


class StreamDecoder:
    """Incremental decoder for transparent stream mode.

    Handles split/merged serial reads and inter-message garbage by scanning
    for the 2-byte magic + CRC16-validated header. False resync hits inside
    payload data are ~2^-32 per position.
    """

    def __init__(
        self,
        rssi_trailer: int = 0,
        partial_timeout: float = 30.0,
        mtu: int = DEFAULT_MTU,
        *,
        max_frame_size: int | None = None,
    ) -> None:
        self._buf = bytearray()
        self._partials: dict[int, _Partial] = {}
        self._incomplete_header: bytes | None = None
        self._incomplete_last_input = 0.0
        self.rssi_trailer = rssi_trailer  # one trailer per frame, if frame boundaries align
        self.partial_timeout = partial_timeout
        if not HEADER_LEN + 1 <= mtu <= DEFAULT_MTU:
            raise ValueError(f"mtu must be {HEADER_LEN + 1}..{DEFAULT_MTU}")
        if max_frame_size is not None and mtu != DEFAULT_MTU:
            raise ValueError("specify mtu or max_frame_size, not both")
        room = mtu - HEADER_LEN if max_frame_size is None else max_frame_size
        if not 0 <= room <= MAX_TOTAL:
            raise ValueError(f"max_frame_size must be 0..{MAX_TOTAL}")
        self.max_room = room

    def feed(
        self, chunk: bytes | bytearray | memoryview, *, final: bool = False
    ) -> list[DecodedMessage]:
        """Decode serial bytes; with *final*, discard incomplete frames at EOF."""
        import time as _time

        now = _time.monotonic()
        stale_incomplete = (
            self._incomplete_header is not None
            and now - self._incomplete_last_input >= self.partial_timeout
        )
        self._buf += chunk
        out: list[DecodedMessage] = []
        buf = self._buf
        pos = 0
        n = len(buf)
        while True:
            self._prune(_time.monotonic())
            i = buf.find(MAGIC_BYTES, pos) if pos < n else -1
            if i < 0:
                # keep at most one trailing magic byte; drop scanned garbage
                keep = 1 if not final and n > 0 and buf[-1:] == MAGIC_BYTES[:1] else 0
                del buf[: max(pos, n - keep)]
                self._incomplete_header = None
                if final:
                    self._partials.clear()
                return out
            if n - i < HEADER_LEN:
                if final:
                    pos = i + 1
                    continue
                del buf[:i]  # wait for more
                return out
            h = _unpack_header(buf, i, self.max_room)
            if h is None:
                self._incomplete_header = None
                pos = i + 1
                continue
            verflags, seq, total, idx, count, fl, msg_crc = h
            end = i + HEADER_LEN + fl
            if n < end:
                if final:
                    pos = i + 1
                    continue
                header = bytes(buf[i : i + HEADER_LEN])
                if stale_incomplete and header == self._incomplete_header:
                    self._incomplete_header = None
                    stale_incomplete = False
                    pos = i + 1
                    continue
                self._incomplete_header = header
                if chunk:
                    self._incomplete_last_input = now
                del buf[:i]  # incomplete frame, wait
                return out
            self._incomplete_header = None
            rssi: int | None = None
            if self.rssi_trailer and n >= end + self.rssi_trailer:
                rssi = buf[end]  # 1-byte binary RSSI trailer (AT+RSSI=1)
                end += self.rssi_trailer
            elif self.rssi_trailer:
                if final:
                    pos = i + 1
                    continue
                del buf[:i]  # wait for trailer byte
                return out
            payload = bytes(buf[i + HEADER_LEN : end - (self.rssi_trailer or 0)])
            if count == 1:
                msg = _make_message(seq, verflags, total, count, msg_crc, payload, rssi)
                if msg is None:
                    # A damaged frame may have swallowed later bytes. Scan for
                    # another valid header before discarding the buffered data.
                    pos = i + 1
                    continue
                pos = end
            else:
                pos = end
                msg = self._push(seq, verflags, total, idx, count, msg_crc, payload, rssi=rssi)
            if msg is not None:
                out.append(msg)
            # opportunistically compact consumed prefix
            if pos > 4096:
                del buf[:pos]
                n = len(buf)
                pos = 0

    def _prune(self, now: float) -> None:
        if not self._partials:
            return
        stale = [s for s, p in self._partials.items() if now - p.t0 > self.partial_timeout]
        for s in stale:
            del self._partials[s]

    def _push(
        self,
        seq: int,
        verflags: int,
        total: int,
        idx: int,
        count: int,
        msg_crc: int,
        payload: bytes,
        rssi: int | None = None,
    ) -> DecodedMessage | None:
        import time as _time

        p = self._partials.get(seq)
        if (
            p is None
            or p.verflags != verflags
            or p.count != count
            or p.total != total
            or p.msg_crc != msg_crc
        ):
            p = self._partials[seq] = _Partial(
                verflags, total, count, msg_crc, t0=_time.monotonic(), rssi=rssi
            )
        p.parts[idx] = payload
        p.t0 = _time.monotonic()  # expire only after progress stops
        if rssi is not None:
            p.rssi = rssi  # last fragment's RSSI wins
        if len(p.parts) != count:
            return None
        ordered = [p.parts[k] for k in sorted(p.parts)]
        body = b"".join(ordered)
        del self._partials[seq]
        return _make_message(seq, verflags, total, count, msg_crc, body, p.rssi)


# ---------------------------------------------------------------------------
# Packet-mode (AT+MODE=2) 3-byte addressing helpers — struct-packed
# ---------------------------------------------------------------------------

_PACKET_PREFIX = struct.Struct("!HB")  # addr BE16 + channel U8


def packet_prefix(dest_addr: int, dest_channel: int) -> bytes:
    if not 0 <= dest_addr <= 0xFFFF:
        raise ValueError("dest_addr 0..65535")
    if not 0 <= dest_channel <= 80:
        raise ValueError("dest_channel 0..80")
    return _PACKET_PREFIX.pack(dest_addr, dest_channel)


def packet_split(data: bytes) -> tuple[int, int, bytes]:
    """Split received packet-mode payload -> (src_addr?, channel, body).

    NOTE: the air format carries only the *destination*; the receiver sees the
    body. We return (dest_addr_echo, channel, body) — callers that need the
    sender use stream metadata instead. Kept symmetric for tests/loopback.
    """
    if len(data) < 3:
        raise ValueError("packet too short for 3-byte prefix")
    addr, ch = _PACKET_PREFIX.unpack_from(data, 0)
    return addr, ch, bytes(data[3:])
