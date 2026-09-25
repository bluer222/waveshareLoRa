"""Async send/receive example (needs two USB-TO-LoRa units).

Pair two modules on the same channel first (see configure.py), then run this
script on each side with a different --port.
"""

import argparse
import asyncio

from waveshare_lora import LoraConfig, LoRaDevice


async def talk(port: str, channel: int, band: str) -> None:
    cfg = LoraConfig(txch=channel, rxch=channel, band=band)
    async with LoRaDevice(port, config=cfg) as dev:
        print(f"[{port}] listening @ {cfg.tx_freq_mhz()} MHz ... (Ctrl-C to stop)")
        for i in range(5):
            await dev.send_json({"node": port, "seq": i, "temp": 20.0 + i * 0.1})
            print(f"[{port}] sent seq={i}")
            try:
                data, meta = await dev.recv_json(timeout=8.0)
                print(
                    f"[{port}] got {data} "
                    f"(seq={meta.seq} frags={meta.count} "
                    f"compressed={meta.compressed} rssi={meta.rssi})"
                )
            except TimeoutError:
                print(f"[{port}] no reply within 8s")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--channel", type=int, default=18)
    ap.add_argument("--band", default="HF", choices=["HF", "LF"])
    args = ap.parse_args()
    asyncio.run(talk(args.port, args.channel, args.band))


if __name__ == "__main__":
    main()
