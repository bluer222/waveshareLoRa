"""JSON framing for the SX1262 USB-TO-LoRa link: one frame per RF packet.

Wire format (big-endian):

.. code-block:: text

    +-------+----------+------+-----+-------+-------------+
    | MAGIC | VERFLAGS | SEQ  | LEN | CRC32 | PAYLOAD     |
    | 2B    | 1B       | 2B   | 1B  | 4B    | LEN B       |
    +-------+----------+------+-----+-------+-------------+

* ``MAGIC`` = ``4C 4F`` (``'LO'``) for resync scanning in stream mode.
* ``VERFLAGS`` = ``(VER << 4) | FLAGS``; ``VER`` = 2. ``FLAGS`` bit0 = raw
  DEFLATE compressed, bit1 = raw bytes (not JSON). Other bits reserved (0).
* ``SEQ`` = uint16 message sequence (wraps). A gap means lost packets.
* ``LEN`` = payload length.
* ``CRC32`` covers ``VERFLAGS``, ``SEQ``, ``LEN``, and the payload.

A frame is at most 240 bytes, the module's largest single RF packet, so every
message is one packet and decodes on its own: losing one packet never takes
another message with it. Messages that do not fit raise
:class:`MessageTooLarge`; split independent data across several messages, and
use :func:`fits_json` to check before sending.
"""

from __future__ import annotations

import json
import struct
import zlib
from binascii import crc32
from dataclasses import dataclass
from typing import Any

from .config import MAX_AIR_SINGLE

try:  # optional C-speed JSON
    import orjson as _orjson
except ImportError:  # pragma: no cover
    _orjson = None


MAGIC_BYTES: bytes = b"\x4c\x4f"  # 'LO'
VERSION: int = 2
FLAG_COMPRESSED: int = 0x01
FLAG_RAW: int = 0x02

_HEADER = struct.Struct("!2sBHBI")  # magic, verflags, seq, len, crc32
HEADER_LEN: int = _HEADER.size  # 10
MAX_FRAME: int = MAX_AIR_SINGLE  # 240
MAX_PAYLOAD: int = MAX_FRAME - HEADER_LEN  # 230
MAX_DECODED: int = 65_536  # a full packet of DEFLATE can expand to about 237 KB


class MessageTooLarge(ValueError):
    """The encoded message does not fit in one RF packet."""


def _dump_json(obj: Any) -> bytes:
    if _orjson is not None:
        return _orjson.dumps(obj)  # already compact
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _load_json(data: bytes) -> Any:
    if _orjson is not None:
        return _orjson.loads(data)
    return json.loads(data.decode("utf-8"))


def _deflate(data: bytes, level: int) -> bytes:
    # Raw DEFLATE: the frame CRC already covers the payload, so zlib's own
    # 2-byte header and 4-byte checksum would only cost room.
    compressor = zlib.compressobj(level, zlib.DEFLATED, -15)
    return compressor.compress(data) + compressor.flush()


@dataclass
class FrameMeta:
    seq: int = 0
    size: int = 0  # frame bytes on the wire, header included
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


def _frame(plain: bytes, seq: int, raw: bool, max_frame: int, compress_level: int) -> bytes:
    if not HEADER_LEN < max_frame <= MAX_FRAME:
        raise ValueError(f"max_frame must be {HEADER_LEN + 1}..{MAX_FRAME}")
    room = max_frame - HEADER_LEN
    payload, compressed = plain, False
    if compress_level:
        packed = _deflate(plain, compress_level)
        if len(packed) < len(plain):
            payload, compressed = packed, True
    if len(payload) > room:
        raise MessageTooLarge(f"message needs {HEADER_LEN + len(payload)} bytes; one packet holds {max_frame}")
    verflags = (VERSION << 4) | (FLAG_COMPRESSED if compressed else 0) | (FLAG_RAW if raw else 0)
    checked = struct.pack("!BHB", verflags, seq & 0xFFFF, len(payload)) + payload
    return _HEADER.pack(MAGIC_BYTES, verflags, seq & 0xFFFF, len(payload), crc32(checked)) + payload


def encode_json(obj: Any, seq: int, max_frame: int = MAX_FRAME, compress_level: int = 9) -> bytes:
    """Encode *obj* as one frame. Raises :class:`MessageTooLarge` if it does not fit."""
    return _frame(_dump_json(obj), seq, raw=False, max_frame=max_frame, compress_level=compress_level)


def encode_raw(data: bytes, seq: int, max_frame: int = MAX_FRAME, compress_level: int = 9) -> bytes:
    """Encode opaque bytes as one frame. Raises :class:`MessageTooLarge` if it does not fit."""
    return _frame(bytes(data), seq, raw=True, max_frame=max_frame, compress_level=compress_level)


def fits_json(obj: Any, max_frame: int = MAX_FRAME, compress_level: int = 9) -> bool:
    """Whether *obj* encodes to one frame of at most *max_frame* bytes."""
    try:
        encode_json(obj, 0, max_frame=max_frame, compress_level=compress_level)
    except MessageTooLarge:
        return False
    return True


def _header(buf: bytes | bytearray, off: int) -> tuple[int, int, int, int] | None:
    """Validate the fixed header fields at *off*: ``(verflags, seq, length, crc)`` or None."""
    if len(buf) - off < HEADER_LEN:
        return None
    magic, verflags, seq, length, crc = _HEADER.unpack_from(buf, off)
    if magic != MAGIC_BYTES or verflags >> 4 != VERSION or verflags & 0x0C or length > MAX_PAYLOAD:
        return None
    return verflags, seq, length, crc


def _message(buf: bytes | bytearray, off: int, rssi: int | None = None) -> DecodedMessage | None:
    """Decode the complete frame at *off*, or None if it is invalid."""
    header = _header(buf, off)
    if header is None:
        return None
    verflags, seq, length, crc = header
    end = off + HEADER_LEN + length
    if len(buf) < end:
        return None
    payload = bytes(buf[off + HEADER_LEN : end])
    if crc32(bytes(buf[off + 2 : off + 6]) + payload) != crc:
        return None
    compressed, raw = bool(verflags & FLAG_COMPRESSED), bool(verflags & FLAG_RAW)
    try:
        if compressed:
            decompressor = zlib.decompressobj(-15)
            plain = decompressor.decompress(payload, MAX_DECODED + 1)
            if len(plain) > MAX_DECODED or not decompressor.eof or decompressor.unused_data:
                return None
        else:
            plain = payload
        data = plain if raw else _load_json(plain)
    except (ValueError, zlib.error):
        return None
    meta = FrameMeta(
        seq=seq,
        size=HEADER_LEN + length,
        compressed=compressed,
        raw=raw,
        json_bytes=0 if raw else len(plain),
        rssi=rssi,
    )
    return DecodedMessage(data=data, meta=meta)


def decode_frame(frame: bytes, rssi: int | None = None) -> DecodedMessage:
    """Decode exactly one frame. Raises ValueError if it is damaged or has extra bytes."""
    header = _header(frame, 0)
    if header is None or len(frame) != HEADER_LEN + header[2]:
        raise ValueError("bad frame header or length")
    msg = _message(frame, 0, rssi)
    if msg is None:
        raise ValueError("CRC mismatch or invalid payload")
    return msg


class StreamDecoder:
    """Incremental decoder for transparent stream mode.

    Handles split and merged serial reads and garbage between frames by
    scanning for the magic and checking the CRC. A damaged header can claim
    up to :data:`MAX_PAYLOAD` bytes; the decoder stops waiting for it as soon
    as a later complete frame appears.
    """

    def __init__(self, rssi_trailer: int = 0) -> None:
        self._buf = bytearray()
        self.rssi_trailer = rssi_trailer  # bytes the dongle appends after each RF packet

    def _later_frame(self, start: int) -> int | None:
        buf = self._buf
        i = buf.find(MAGIC_BYTES, start)
        while i >= 0:
            if _message(buf, i) is not None:
                return i
            i = buf.find(MAGIC_BYTES, i + 1)
        return None

    def feed(self, chunk: bytes | bytearray | memoryview, *, final: bool = False) -> list[DecodedMessage]:
        """Decode serial bytes; with *final*, discard incomplete frames at EOF."""
        self._buf += chunk
        buf = self._buf
        out: list[DecodedMessage] = []
        pos = 0
        while True:
            i = buf.find(MAGIC_BYTES, pos)
            if i < 0:
                keep = 1 if not final and buf[-1:] == MAGIC_BYTES[:1] else 0
                del buf[: len(buf) - keep]
                return out
            header = _header(buf, i)
            if header is None and len(buf) - i >= HEADER_LEN:
                pos = i + 1
                continue
            end = i + HEADER_LEN + (header[2] if header else MAX_PAYLOAD) + self.rssi_trailer
            if header is None or len(buf) < end:
                later = self._later_frame(i + 1)
                if later is not None:
                    pos = later
                elif final:
                    pos = i + 1
                else:
                    del buf[:i]
                    return out
                continue
            rssi = buf[end - 1] if self.rssi_trailer else None
            msg = _message(buf, i, rssi)
            if msg is None:
                pos = i + 1
                continue
            out.append(msg)
            pos = end


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
