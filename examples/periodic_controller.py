"""Print received data; type temperature or humidity to change future readings.

Configure both radios for stream mode on HF channel 65 first.
Edit PORT to match the controller's radio.
"""

import asyncio

from waveshare_lora import LoraConfig, LoRaDevice

PORT = "/dev/ttyUSB0"  # use /dev/ttyUSB1 if both radios are on the same computer


async def main() -> None:
    cfg = LoraConfig(txch=65, rxch=65, band="HF")
    async with LoRaDevice(PORT, config=cfg) as dev:

        async def receive_data() -> None:
            async for data, _ in dev.messages():
                print("received:", data)

        async def send_commands() -> None:
            while True:
                field = (await asyncio.to_thread(input, "Send temperature or humidity: ")).strip()
                if field in ("temperature", "humidity"):
                    await dev.send_json({"send": field})
                    print("command sent:", field)

        await asyncio.gather(receive_data(), send_commands())


if __name__ == "__main__":
    asyncio.run(main())
