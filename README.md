# waveshare-lora — async JSON over Waveshare USB-TO-LoRa-xF (SX1262)

Async Python library for sending/receiving JSON over the
[Waveshare USB-TO-LoRa-xF](https://www.waveshare.com/usb-to-lora.htm)
([wiki](https://www.waveshare.com/wiki/USB-TO-LoRa-xF), SX1262, 410–490 / 850–930 MHz).
Full AT parameter coverage, `asyncio`-native serial, and a compact
`struct.pack` + `zlib` message codec.

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

Packet mode (`AT+MODE=2`) sends a destination prefix followed by the frame.
The prefix and frame must fit within 240 bytes:

```python
cfg = LoraConfig(mode=2, addr=0x0001, txch=18, rxch=18)
async with LoRaDevice("/dev/ttyUSB0", config=cfg) as dev:
    await dev.apply_config(cfg)  # set AT+MODE=2 on the radio
    await dev.send_json({"cmd": "on"}, dest_addr=0x0002, dest_channel=18)
```

Configure every radio parameter at once:

```python
await dev.apply_config(LoraConfig(sf=9, pwr=22, key=1234, lbt=True))
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
| `rssi` | `AT+RSSI` | 0/1 | trailer per dongle output packet; library sends are limited to one RF packet when enabled |
| `addr` | `AT+ADDR` | 0–65535 | 65535 = broadcast listen |
| `port` | `AT+PORT` | 1 RS422 / 2 RS485 / 3 RS232 | kept for AllP compat on USB units |
| `baud` | `AT+BAUD` | 1200…115200 | host port re-opens on change |
| `comm` | `AT+COMM` | `"8N1"`-style | `[89][NOE][012]` |
| `key` | `AT+KEY` | 0–65535 | 0 = AES off |
| `band` | — | HF/LF | host-side frequency base (not stored) |

Plus `+++` / `AT+EXIT` / `ATE` / `AT+VER` / `AT+HELP` / `AT+ALLP?` /
`AT+AllP=…` / `AT+RESTORE=1`. `apply_config()` writes a single `AT+AllP`
(15 fields) by default, exits (module reboots ≈3 s), and re-opens the host
port if the baud or serial framing changed. `+++` is sent in one write — char-by-char entry
is ignored by firmware.
With `restore_first=True`, the library waits for the reset, re-opens the host
port at the factory 115200/8N1 setting, then enters AT mode to apply the new
configuration. The reset command's exact reply and reboot timing need a
hardware check on the target firmware.

## One message, one packet (codec)

Every message is one frame, and every frame is one RF packet of at most 240
bytes, the module's single-packet limit. A lost packet loses only its own
message. There is no fragmentation: `send_json` raises `MessageTooLarge` for a
message that does not fit, and callers split independent data into several
messages. `LoRaDevice.fits(obj)` (or `fits_json(obj, max_frame)`) checks first.

```
MAGIC 2B | VERFLAGS 1B | SEQ 2B | LEN 1B | CRC32 4B | payload (≤ 230 B)
```

* JSON dumped compactly (`orjson` when installed, else `separators=(",",":")`).
* Raw DEFLATE applied only when it shrinks the payload; 1 flag bit records it.
  The frame CRC32 covers the header fields and payload, so zlib's own wrapper
  and checksum are dropped. Decoded payloads are limited to 64 KiB.
* `SEQ` increments per message, so a receiver can count lost packets from gaps.
* `StreamDecoder` finds frames across arbitrary serial reads by scanning for
  the magic and checking the CRC. A damaged header can claim up to 230 bytes;
  the decoder stops waiting for it as soon as a later complete frame arrives.
* The sender writes one frame and waits for a rough airtime estimate plus
  50 ms (and 2 s with LBT). Pacing is not a transmit-complete signal.
* Packet mode puts a 3-byte prefix before the frame, leaving 237 bytes for it.
  Waveshare documents what the first three bytes of a sent data stream mean,
  but not the UART idle time that separates streams. Consecutive packet-mode
  sends still need a hardware test to confirm that the dongle treats them as
  separate streams. To check this, alternate small and 240-byte sends to two
  receiver addresses and verify both the destination and decoded payload on
  each receiver.
* With `AT+RSSI=1`, the dongle appends an RSSI byte after each RF packet;
  since a frame is one packet, each frame carries its own RSSI.

Typical sensor JSON compresses 50–80 %; each frame has a 10-byte header.

## Layout

```
src/waveshare_lora/{config,at,codec,device}.py  # library
examples/{send_receive,configure}.py
tests/test_lora.py                               # hardware-free, pytest-asyncio
```

## Hardware notes

* Default pairing: stream mode, 115200 8N1, CH18 (HF 868 MHz) / CH23 (LF 433 MHz).
* Single RF packet ≤ 240 B; the dongle auto-packetizes larger streams and has
  a fixed 960-byte cache. Each library write is one frame within the RF packet
  limit, but the vendor does not document RF flush timing.
* Relay mode (`AT+MODE=3`) forwards only — no serial output. Relay ADDR =
  `first_netid << 8 | second_netid` (`LoraConfig.relay_addr`).
* `65535` (`0xFFFF`) is the broadcast/listen-all address.
* AES: same nonzero `key` on both ends or no link.

## Test

```bash
pip install -e ".[test]"
python -m pytest -q
```
