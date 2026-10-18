"""Parameter configuration for Waveshare USB-TO-LoRa-xF / SX1262-LoRa-DTU-xF.

Implements every documented AT parameter, with validation, AT command
generation, AllP (bulk) encoding, and frequency helpers.

References:
  https://www.waveshare.com/wiki/USB-TO-LoRa-xF
  https://www.waveshare.com/wiki/SX1262-LoRa-DTU-xF

AllP field order (15 fields, SF..KEY) on firmware Ver1.2 (hardware truth):
  SF, BW_index, CR, PWR, NETID, LBT, MODE, TXCH, RXCH, RSSI, ADDR, PORT, COMM, BAUD, KEY

NOTE: the Waveshare wiki documents ``AT+AllP=7,125,...`` (BW in kHz, order
PORT,BAUD,COMM), but Ver1.2 hardware replies ``+ALLP=7,0,...`` (BW *index*
0=125/1=250/2=500, order PORT,COMM,BAUD). This class writes the hardware
order and parses both spellings.
"""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any

# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class BandwidthIndex(IntEnum):
    KHZ_125 = 0
    KHZ_250 = 1
    KHZ_500 = 2

    @property
    def khz(self) -> int:
        return {0: 125, 1: 250, 2: 500}[int(self)]

    @classmethod
    def from_khz(cls, khz: int) -> BandwidthIndex:
        mapping = {125: cls.KHZ_125, 250: cls.KHZ_250, 500: cls.KHZ_500}
        try:
            return mapping[int(khz)]
        except KeyError:
            raise ValueError(f"bandwidth must be one of 125/250/500 kHz, got {khz!r}")


class CodeRateIndex(IntEnum):
    CR_4_5 = 1
    CR_4_6 = 2
    CR_4_7 = 3
    CR_4_8 = 4

    @property
    def label(self) -> str:
        return {1: "4/5", 2: "4/6", 3: "4/7", 4: "4/8"}[int(self)]


class OperationMode(IntEnum):
    STREAM = 1  # transparent stream (e.g. Modbus ASCII / plain serial)
    PACKET = 2  # packet mode, 3-byte addr+channel prefix (e.g. Modbus RTU)
    RELAY = 3  # relay node: forwards, no serial output


class SerialPort(IntEnum):
    RS422 = 1
    RS485 = 2
    RS232 = 3  # default; also used as placeholder on USB-only units


class FrequencyBand(str):
    LF = "LF"  # 410..490 MHz  (410 + CH)
    HF = "HF"  # 850..930 MHz  (850 + CH)

    def __new__(cls, value: str) -> "FrequencyBand":  # noqa: PYI034, UP037
        v = str(value).upper()
        if v not in ("LF", "HF"):
            raise ValueError(f"band must be 'LF' or 'HF', got {value!r}")
        obj = str.__new__(cls, v)
        return obj


VALID_BAUDS = (1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200)
COMM_RE = re.compile(r"^([89])([NOE])([012])$")

BROADCAST_ADDR = 0xFFFF  # 65535: broadcast / listen-all
MAX_AIR_SINGLE = 240  # RF packet cap; the device packetizes longer serial streams
CACHE_BYTES = 960  # documented device cache size


def comm_to_pyserial(comm: str) -> dict[str, Any]:
    """Convert ``8N1``-style COMM string to pyserial kwargs.

    COMM format: ``<databits><parity><stop>`` e.g. ``8N1``.
    databits: 8 or 9 (9 is mapped to 8 because pyserial has no 9-bit mode;
      the DTU UART still accepts it, the USB CDC side negotiates 8).
    parity: N/O/E. stop: 0/1/2 (0 is treated as 1 on the host side).
    """
    m = COMM_RE.match(comm.upper())
    if not m:
        raise ValueError(f"COMM must match [89][NOE][012], e.g. '8N1', got {comm!r}")
    databits, parity, stop = m.groups()
    import serial

    return {
        "bytesize": serial.EIGHTBITS,  # 9-bit has no host equivalent; device handles it
        "parity": {
            "N": serial.PARITY_NONE,
            "O": serial.PARITY_ODD,
            "E": serial.PARITY_EVEN,
        }[parity],
        "stopbits": {
            # Device value '0' historically means one stop bit.
            "0": serial.STOPBITS_ONE,
            "1": serial.STOPBITS_ONE,
            "2": serial.STOPBITS_TWO,
        }[stop],
        "_device_databits": int(databits),
    }


# ---------------------------------------------------------------------------
# Main config dataclass
# ---------------------------------------------------------------------------


@dataclass
class LoraConfig:
    """Full device parameter set. Defaults = factory stream-mode pairing defaults."""

    sf: int = 7  # 7..12
    bw: BandwidthIndex = field(default=BandwidthIndex.KHZ_125)
    cr: CodeRateIndex = field(default=CodeRateIndex.CR_4_5)
    pwr: int = 22  # 10..22 dBm
    netid: int = 0  # 0..65535 (early docs say 0..255; firmware accepts 0..65535)
    lbt: bool = False  # listen-before-talk
    mode: OperationMode = field(default=OperationMode.STREAM)
    txch: int = 18  # 0..80
    rxch: int = 18  # 0..80
    rssi: bool = False  # append RSSI byte on RX output
    addr: int = 0  # 0..65535, 65535 = broadcast listen
    port: int = 3  # 1=RS422 2=RS485 3=RS232 (kept for AllP compat on USB units)
    baud: int = 115200  # serial baud 1200..115200
    comm: str = "8N1"  # serial framing
    key: int = 0  # 0=disable AES, 1..65535 AES key
    band: FrequencyBand = field(default_factory=lambda: FrequencyBand("HF"))

    def __post_init__(self) -> None:
        # normalize enum-ish inputs
        if not isinstance(self.bw, BandwidthIndex):
            # accept raw kHz or index
            try:
                v = int(self.bw)  # type: ignore[arg-type]
                self.bw = BandwidthIndex(v) if v in (0, 1, 2) else BandwidthIndex.from_khz(v)
            except (ValueError, TypeError):
                raise ValueError(
                    f"bw must be BandwidthIndex, 0/1/2 or 125/250/500, got {self.bw!r}"
                )
        if not isinstance(self.cr, CodeRateIndex):
            self.cr = CodeRateIndex(int(self.cr))  # type: ignore[arg-type]
        if not isinstance(self.mode, OperationMode):
            self.mode = OperationMode(int(self.mode))  # type: ignore[arg-type]
        if isinstance(self.band, str) and not isinstance(self.band, FrequencyBand):
            self.band = FrequencyBand(self.band)
        if isinstance(self.lbt, int) and not isinstance(self.lbt, bool):
            self.lbt = bool(self.lbt)
        if isinstance(self.rssi, int) and not isinstance(self.rssi, bool):
            self.rssi = bool(self.rssi)
        self.comm = str(self.comm).upper().strip().strip('"').strip("'")
        self.validate()

    # -- validation ------------------------------------------------------
    def validate(self) -> None:
        if not 7 <= int(self.sf) <= 12:
            raise ValueError("sf must be 7..12")
        if int(self.bw) not in (0, 1, 2):
            raise ValueError("bw index must be 0/1/2")
        if int(self.cr) not in (1, 2, 3, 4):
            raise ValueError("cr must be 1..4 (1=4/5..4=4/8)")
        if not 10 <= int(self.pwr) <= 22:
            raise ValueError("pwr must be 10..22 dBm")
        if not 0 <= int(self.netid) <= 65535:
            raise ValueError("netid must be 0..65535")
        if int(self.mode) not in (1, 2, 3):
            raise ValueError("mode must be 1/2/3 (stream/packet/relay)")
        for name in ("txch", "rxch"):
            if not 0 <= int(getattr(self, name)) <= 80:
                raise ValueError(f"{name} must be 0..80")
        if not 0 <= int(self.addr) <= 65535:
            raise ValueError("addr must be 0..65535")
        if int(self.port) not in (1, 2, 3):
            raise ValueError("port must be 1, 2, or 3 (RS422, RS485, or RS232)")
        if int(self.baud) not in VALID_BAUDS:
            raise ValueError(f"baud must be one of {VALID_BAUDS}")
        if not COMM_RE.match(self.comm):
            raise ValueError(f"comm must match [89][NOE][012], e.g. '8N1', got {self.comm!r}")
        if not 0 <= int(self.key) <= 65535:
            raise ValueError("key must be 0..65535 (0 disables AES)")

    # -- frequency helpers -----------------------------------------------
    @property
    def base_mhz(self) -> int:
        return 410 if str(self.band) == "LF" else 850

    def tx_freq_mhz(self) -> int:
        return self.base_mhz + int(self.txch)

    def rx_freq_mhz(self) -> int:
        return self.base_mhz + int(self.rxch)

    @property
    def bw_khz(self) -> int:
        return self.bw.khz

    @property
    def is_broadcast(self) -> bool:
        return int(self.addr) == BROADCAST_ADDR

    # -- AT command generation -------------------------------------------
    def to_at_commands(self) -> list[str]:
        """Individual AT set-commands (without CRLF) for every parameter."""
        return [
            f"AT+SF={int(self.sf)}",
            f"AT+BW={int(self.bw)}",
            f"AT+CR={int(self.cr)}",
            f"AT+PWR={int(self.pwr)}",
            f"AT+NETID={int(self.netid)}",
            f"AT+LBT={int(self.lbt)}",
            f"AT+MODE={int(self.mode)}",
            f"AT+TXCH={int(self.txch)}",
            f"AT+RXCH={int(self.rxch)}",
            f"AT+RSSI={int(self.rssi)}",
            f"AT+ADDR={int(self.addr)}",
            f"AT+PORT={int(self.port)}",
            f"AT+BAUD={int(self.baud)}",
            f'AT+COMM="{self.comm}"',
            f"AT+KEY={int(self.key)}",
        ]

    def to_allp(self) -> str:
        """Build the bulk command in the device's field order (FW Ver1.2).

        BW is sent as the index (0=125/1=250/2=500, like ``AT+BW``) and the
        tail order is ``PORT,COMM,BAUD,KEY`` to match ``AT+ALLP?`` replies.
        """
        return (
            f"AT+AllP={int(self.sf)},{int(self.bw)},{int(self.cr)},{int(self.pwr)},"
            f"{int(self.netid)},{int(self.lbt)},{int(self.mode)},"
            f"{int(self.txch)},{int(self.rxch)},{int(self.rssi)},{int(self.addr)},"
            f'{int(self.port)},"{self.comm}",{int(self.baud)},{int(self.key)}'
        )

    @classmethod
    def from_allp(cls, response: str, band: str = "HF") -> LoraConfig:
        """Parse ``AT+ALLP?`` reply or an AllP value list into a config.

        Accepts ``+AllP:7,125,1,22,...`` / ``7,125,1,22,...`` with optional quotes.
        BW accepts index (0/1/2) or kHz (125/250/500); the tail accepts both
        ``PORT,COMM,BAUD,KEY`` (hardware) and ``PORT,BAUD,COMM,KEY`` (wiki).
        """
        s = response.strip()
        # strip "+ALLP=..." / "+AllP:..." prefix and OK/ERROR/echo lines:
        # the reply block echoes the query plus OK, so pick the data line.
        for line in s.splitlines():
            line = line.strip()
            if "allp" in line.lower() and ("," in line):
                s = line
                break
        s = re.sub(r"(?i)^\+?(AT\+)?allp\s*[:=]\s*", "", s).strip()
        parts = [p.strip().strip('"').strip("'") for p in s.split(",")]
        if len(parts) != 15:
            raise ValueError(f"AllP needs 15 fields, got {len(parts)}: {response!r}")
        (sf, bw_raw, cr, pwr, netid, lbt, mode, txch, rxch, rssi, addr, port, f12, f13, key) = (
            parts
        )
        try:
            bw_val = int(bw_raw)
        except ValueError:
            raise ValueError(f"AllP BW must be 0/1/2 or 125/250/500, got {bw_raw!r}")
        if bw_val in (0, 1, 2):
            bw = BandwidthIndex(bw_val)
        else:
            bw = BandwidthIndex.from_khz(bw_val)  # wiki-style kHz spelling
        # Tail order: hardware sends PORT,COMM,BAUD; wiki documents PORT,BAUD,COMM.
        if COMM_RE.match(f12.upper()) and f13.isdigit():
            comm, baud_raw = f12, f13
        elif f12.isdigit() and COMM_RE.match(f13.upper()):
            baud_raw, comm = f12, f13
        else:
            raise ValueError(f"AllP tail must be PORT,COMM,BAUD or PORT,BAUD,COMM, got {f12!r},{f13!r}")
        return cls(
            sf=int(sf),
            bw=bw,
            cr=int(cr),
            pwr=int(pwr),
            netid=int(netid),
            lbt=bool(int(lbt)),
            mode=int(mode),
            txch=int(txch),
            rxch=int(rxch),
            rssi=bool(int(rssi)),
            addr=int(addr),
            port=int(port),
            baud=int(baud_raw),
            comm=comm,
            key=int(key),
            band=band,
        )

    def diff_commands(self, other: LoraConfig) -> list[str]:
        """AT commands that differ from *other* (useful for minimal writes)."""
        mine = self.to_at_commands()
        theirs = set(other.to_at_commands())
        return [c for c in mine if c not in theirs]

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["bw"] = int(self.bw)
        d["bw_khz"] = self.bw_khz
        d["cr"] = int(self.cr)
        d["cr_label"] = self.cr.label
        d["mode"] = int(self.mode)
        d["mode_label"] = self.mode.name
        d["lbt"] = bool(self.lbt)
        d["rssi"] = bool(self.rssi)
        d["band"] = str(self.band)
        d["tx_freq_mhz"] = self.tx_freq_mhz()
        d["rx_freq_mhz"] = self.rx_freq_mhz()
        return d

    # -- link helpers ------------------------------------------------------
    def estimate_airtime_ms(self, payload_bytes: int) -> float:
        """Rough LoRa airtime estimate (Semtech formula, 8-symbol preamble, CRC on).

        Useful for rough transmit pacing; not a substitute for a link budget tool.
        """
        sf = int(self.sf)
        bw_hz = self.bw_khz * 1000
        cr_denom = {1: 5, 2: 6, 3: 7, 4: 8}[int(self.cr)]
        # symbols
        tsym_ms = (2**sf) / bw_hz * 1000.0
        # payload symbols (simplified, header enabled, CRC on, no low-data-rate opt correction
        # except SF11/12 @125k which need DE=1)
        de = 1 if (sf >= 11 and bw_hz == 125000) else 0
        ih, crc_on = 0, 1
        import math

        num = 8 * payload_bytes - 4 * sf + 28 + 16 * crc_on - 20 * ih
        den = 4 * (sf - 2 * de)
        payload_sym = 8 + max(0, math.ceil(num / den)) * cr_denom
        return (8 + 4.25 + payload_sym) * tsym_ms  # 8-symbol preamble + sync

    @classmethod
    def pair(
        cls, addr_a: int = 0, addr_b: int = 0, channel: int = 18, band: str = "HF", **overrides: Any
    ) -> tuple[LoraConfig, LoraConfig]:
        """Two stream-mode configs that talk to each other out of the box."""
        base = {"sf": 7, "mode": 1, "txch": channel, "rxch": channel, "band": band}
        base.update(overrides)
        return cls(addr=addr_a, **base), cls(addr=addr_b, **base)

    @staticmethod
    def relay_addr(netid_first: int, netid_second: int) -> int:
        """Compose a relay-node ADDR from two endpoint NETIDs: ``first<<8 | second``.

        Matches Waveshare docs (e.g. relay 0x0304 bridges NETIDs 0x03 and 0x04).
        """
        if not (0 <= netid_first <= 255 and 0 <= netid_second <= 255):
            raise ValueError("relay NETIDs must each be 0..255")
        return ((netid_first & 0xFF) << 8) | (netid_second & 0xFF)
