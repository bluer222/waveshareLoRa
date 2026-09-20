# waveshare-lora — async JSON over Waveshare USB-TO-LoRa-xF (SX1262)

Async Python library for sending/receiving JSON over the
[Waveshare USB-TO-LoRa-xF](https://www.waveshare.com/usb-to-lora.htm)
([wiki](https://www.waveshare.com/wiki/USB-TO-LoRa-xF), SX1262, 410–490 / 850–930 MHz).
Full AT parameter coverage, `asyncio`-native serial, and a minimal-bytes
`struct.pack` + `zlib` frame codec with auto-fragmentation.

## Install

```bash
pip install -e ".[fast]"   # fast: orjson for C-speed JSON
```

Needs two units on the same channel/band to form a link (point-to-point out of the box).

## Quick start (stream mode, transparent)

```python
import asyncio
from waveshare_lora import LoRaDevice, LoraConfig

async def main():
    cfg = LoraConfig(txch=18, rxch=18, band="HF")  # 868 MHz
    async with LoRaDevice("/dev/ttyUSB0", config=cfg) as dev:
        await dev.send_json({"temp": 21.5, "id": 7})
        data, meta = await dev.recv_json(timeout=10.0)
        print(data, meta)

asyncio.run(main())
```

Packet mode (per-message addressing, `AT+MODE=2`):

```python
cfg = LoraConfig(mode=2, addr=0x0001, txch=18, rxch=18)
async with LoRaDevice("/dev/ttyUSB0", config=cfg) as dev:
    await dev.send_json({"cmd": "on"}, dest_addr=0x0002, dest_channel=18)
```

Configure every radio parameter at once:

```python
await dev.apply_config(LoraConfig(sf=9, pwr=22, key=1234, lbt=True, rssi=True))
live = await dev.read_config()   # AT+ALLP? round-trip
```

See `examples/configure.py` and `examples/send_receive.py`.

## All parameters (`LoraConfig`)

| Field | AT | Range | Notes |
|---|---|---|---|
| `sf` | `AT+SF` | 7–12 | spreading factor |
| `bw` | `AT+BW` | 0/1/2 = 125/250/500 kHz | `AllP` uses kHz value |
| `cr` | `AT+CR` | 1–4 = 4/5…4/8 | error-correction strength |
| `pwr` | `AT+PWR` | 10–22 dBm | TX power |
| `netid` | `AT+NETID` | 0–65535 | relay addressing domain |
| `lbt` | `AT+LBT` | 0/1 | listen-before-talk |
| `mode` | `AT+MODE` | 1 stream / 2 packet / 3 relay | |
| `txch`/`rxch` | `AT+TXCH`/`AT+RXCH` | 0–80 | freq = 410+CH (LF) or 850+CH (HF) |
| `rssi` | `AT+RSSI` | 0/1 | 1-byte trailer on RX |
| `addr` | `AT+ADDR` | 0–65535 | 65535 = broadcast listen |
| `port` | `AT+PORT` | 1 RS422 / 2 RS485 / 3 RS232 | kept for AllP compat on USB units |
| `baud` | `AT+BAUD` | 1200…115200 | host port re-opens on change |
| `comm` | `AT+COMM` | `"8N1"`-style | `[89][NOE][012]` |
| `key` | `AT+KEY` | 0–65535 | 0 = AES off |
| `band` | — | HF/LF | host-side frequency base (not stored) |

Plus `+++` / `AT+EXIT` / `ATE` / `AT+VER` / `AT+HELP` / `AT+ALLP?` /
`AT+AllP=…` / `AT+RESTORE=1`. `apply_config()` writes a single `AT+AllP`
(15 fields) by default, exits (module reboots ≈3 s), and re-opens the host
port if the baud changed. `+++` is sent in one write — char-by-char entry
is ignored by firmware.

## Wire efficiency (codec)

`codec.py` packs each message with precompiled `struct`:

```
MAGIC 2B | VERFLAGS 1B | SEQ 2B | TOTAL 2B | IDX 1B | COUNT 1B | FRAG_LEN 2B | HDR_CRC 2B | CRC32 4B | payload
```

* JSON dumped compactly (`orjson` when installed, else `separators=(",",":")`).
* `zlib` (C) compression applied only when it shrinks; 1 flag bit records it.
* Fragments sized to the 240 B air MTU (237 B in packet mode after the
  `struct`-packed 3 B addr+channel prefix); header CRC16 rejects false
  resync hits inside payload data, CRC32 guards reassembly;
  `StreamDecoder` resyncs across split/merged serial reads with per-fragment
  `struct.unpack_from` and `memoryview` (no per-byte Python loops).
* Airtime-proportional pacing between fragments (`estimate_airtime_ms`).

Typical sensor JSON compresses 50–80 %; header overhead is 17 B/fragment.

## Layout

```
src/waveshare_lora/{config,at,codec,device}.py  # library
examples/{send_receive,configure}.py
tests/test_lora.py                               # hardware-free, pytest-asyncio
```

## Hardware notes

* Default pairing: stream mode, 115200 8N1, CH18 (HF 868 MHz) / CH23 (LF 433 MHz).
* Single air packet ≤ 240 B; device auto-fragments above that with a 960 B cache.
* Relay mode (`AT+MODE=3`) forwards only — no serial output. Relay ADDR =
  `first_netid << 8 | second_netid` (`LoraConfig.relay_addr`).
* `65535` (`0xFFFF`) is the broadcast/listen-all address.
* AES: same nonzero `key` on both ends or no link.

## Test

```bash
pip install -e ".[test]"
python -m pytest -q
```
