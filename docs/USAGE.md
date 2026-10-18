# Usage and protocol details

Run the commands below from the repository root after installing Python 3.10 or newer.

## Install

```bash
pip install -e ".[fast]"   # fast: orjson for C-speed JSON
```

Use two radios configured for the same band, channel, and radio settings.
Passing `config` to `LoRaDevice` sets host expectations. It does not write settings
to the radio; use `apply_config()` or `examples/configure.py` first.

## Quick start in transparent stream mode

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

Packet mode (`AT+MODE=2`) sends a destination prefix followed by one frame.
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

See [configure.py](../examples/configure.py) and [send_receive.py](../examples/send_receive.py).

### Periodic data with occasional commands

`examples/periodic_sender.py` sends a fake temperature reading every five
seconds while listening for commands. `examples/periodic_controller.py`
prints incoming readings. Type `humidity` or `temperature` on the controller
to select the field sent in future readings.

Set `PORT` in each script and `INTERVAL` in the sender. Configure both HF
radios once for stream mode on channel 65, matching these examples:

```bash
# Run on each computer, using its radio's port.
python examples/configure.py --port /dev/ttyUSB0 --channel 65 --band HF --mode 1
```

Then run one script on each side:

```bash
python examples/periodic_sender.py
python examples/periodic_controller.py
```

If both radios are on one computer, use separate ports and terminals.
The scripts assume the radios are already configured; passing `config` to
`LoRaDevice` does not write settings to the radio. The readings are placeholders
for your sensor code. This basic example has no command acknowledgements or
retries. Send commands between readings; simultaneous radio transmissions can
lose messages. If the sender keeps sending the old field, send the command again.

## All parameters (`LoraConfig`)

| Field | AT | Range | Notes |
|---|---|---|---|
| `sf` | `AT+SF` | 7–12 | spreading factor |
| `bw` | `AT+BW` | 0/1/2 = 125/250/500 kHz | `AllP` carries the same index on FW Ver1.2 (wiki shows kHz) |
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
(15 fields, order `SF,BW,CR,PWR,NETID,LBT,MODE,TXCH,RXCH,RSSI,ADDR,PORT,COMM,BAUD,KEY`
with BW as index, matching FW Ver1.2 `AT+ALLP?` replies; the wiki shows
`PORT,BAUD,COMM` with BW in kHz) by default, exits (module reboots ≈3 s),
and re-opens the host port if the baud or serial framing changed.
`+++` is sent in one write because firmware ignores char-by-char entry.
FW Ver1.2 echoes it with no `OK`, so the library confirms AT mode
with `AT+VER`, retried while the module reboots. HF channel 65 corresponds to
915 MHz; the default HF channel 18 corresponds to 868 MHz. Choose a band, channel, and transmit power allowed for your location.
With `restore_first=True`, the library waits for the reset, re-opens the host
port at the factory 115200/8N1 setting, then enters AT mode to apply the new
configuration. The reset command's exact reply and reboot timing need a
hardware check on the target firmware.

## Wire efficiency (codec)

`codec.py` packs each application message with a precompiled `struct`:

```
MAGIC 2B | VERFLAGS 1B | SEQ 2B | TOTAL 2B | IDX 1B | COUNT 1B | FRAG_LEN 2B | HDR_CRC 2B | CRC32 4B | payload
```

* JSON dumped compactly (`orjson` when installed, else `separators=(",",":")`).
* `zlib` (C) compression applied only when it shrinks; 1 flag bit records it.
  The uncompressed message limit is 1 MiB on both encoding and decoding.
* The encoder compresses each message once, then divides it into application
  fragments no larger than 240 bytes including each 17-byte header.
  `StreamDecoder` reassembles them across arbitrary serial reads. A truncated
  fragment can delay later messages until its declared length is filled or
  30 seconds pass without input; it cannot claim thousands of later bytes.
  On EOF, the decoder scans past a truncated fragment to recover complete
  frames. A complete frame deliberately embedded inside a truncated raw
  payload is indistinguishable from a later frame and may also be delivered.
* The sender writes one fragment at a time and waits for a rough airtime
  estimate. Each write is at most 240 bytes, or less when `tx_chunk_size` is
  smaller. Pacing is not a transmit-complete signal.
* Packet mode sends one prefix and one frame in a single write. The library
  rejects frames that exceed 237 bytes after the 3-byte prefix and requires
  pacing between sends. Waveshare documents what the first three bytes of a
  sent data stream mean, but not the UART idle time that separates streams.
  Consecutive packet-mode sends still need a hardware test to confirm that
  the dongle treats them as separate streams. To check this, alternate small
  and 240-byte sends to two receiver addresses and verify both the destination
  and decoded payload on each receiver.
* When `AT+RSSI=1`, the library rejects outgoing messages needing more than
  one fragment. Every peer must observe that limit.
  RSSI bytes follow the dongle's RF packets, not application messages. This
  mode also assumes the dongle emits one application frame per RF packet;
  verify that on hardware, or leave RSSI disabled.

Compression depends on the payload; each fragment has a 17-byte header.

## Layout

```
src/waveshare_lora/{config,at,codec,device}.py  # library
examples/{send_receive,configure,periodic_sender,periodic_controller}.py
tests/test_lora.py                               # hardware-free, pytest-asyncio
```

## Hardware notes

* Default pairing: stream mode, 115200 8N1, CH18 (HF 868 MHz) / CH23 (LF 433 MHz).
* Single RF packet ≤ 240 B; the dongle auto-packetizes larger streams and has
  a fixed 960-byte cache. Library writes stay within the RF packet limit, but
  the vendor does not document RF flush timing.
* Relay mode, `AT+MODE=3`, forwards packets without serial output. Relay ADDR =
  `first_netid << 8 | second_netid` (`LoraConfig.relay_addr`).
* `65535` (`0xFFFF`) is the broadcast/listen-all address.
* AES: same nonzero `key` on both ends or no link.

## Test

```bash
pip install -e ".[test]"
python -m pytest -q
```
