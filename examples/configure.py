"""Configure a USB-TO-LoRa module (all AT parameters)."""

import argparse
import asyncio

from waveshare_lora import LoraConfig, LoRaDevice


async def main() -> None:
    ap = argparse.ArgumentParser(description="Write/query USB-TO-LoRa AT config")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--channel", type=int, default=18)
    ap.add_argument("--band", default="HF", choices=["HF", "LF"])
    ap.add_argument("--mode", type=int, default=1, choices=[1, 2, 3])
    ap.add_argument("--addr", type=int, default=0)
    ap.add_argument("--sf", type=int, default=7)
    ap.add_argument("--pwr", type=int, default=22)
    ap.add_argument("--key", type=int, default=0)
    ap.add_argument("--rssi", action="store_true")
    ap.add_argument("--lbt", action="store_true")
    ap.add_argument("--read-only", action="store_true")
    args = ap.parse_args()

    cfg = LoraConfig(txch=args.channel, rxch=args.channel, band=args.band,
                     mode=args.mode, addr=args.addr, sf=args.sf,
                     pwr=args.pwr, key=args.key, rssi=args.rssi, lbt=args.lbt)
    async with LoRaDevice(args.port, config=cfg) as dev:
        if args.read_only:
            live = await dev.read_config()
            print(live.to_dict())
            return
        print("writing:", cfg.to_allp())
        await dev.apply_config(cfg)
        print(f"done @ {cfg.tx_freq_mhz()} MHz, rebooted into data mode")


if __name__ == "__main__":
    asyncio.run(main())
