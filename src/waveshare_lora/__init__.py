"""Async Python library for JSON over Waveshare USB-TO-LoRa-xF (SX1262)."""

from .codec import (
    DecodedMessage,
    FrameMeta,
    StreamDecoder,
    decode_fragments,
    encode_json,
    encode_raw,
    packet_prefix,
    packet_split,
)
from .config import (
    BandwidthIndex,
    CodeRateIndex,
    FrequencyBand,
    LoraConfig,
    OperationMode,
    SerialPort,
)
from .device import LoRaDevice

__all__ = [
    "BandwidthIndex",
    "CodeRateIndex",
    "DecodedMessage",
    "FrameMeta",
    "FrequencyBand",
    "LoRaDevice",
    "LoraConfig",
    "OperationMode",
    "SerialPort",
    "StreamDecoder",
    "decode_fragments",
    "encode_json",
    "encode_raw",
    "packet_prefix",
    "packet_split",
]

__version__ = "0.1.0"
