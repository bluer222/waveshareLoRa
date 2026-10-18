"""Send sample data periodically; accept commands choosing which field to send.

Configure both radios for stream mode on HF channel 65 first.
Edit PORT and INTERVAL for your setup. Values below are fake sensor readings.
"""

import asyncio

from waveshare_lora import LoraConfig, LoRaDevice

PORT = "/dev/ttyUSB0"
INTERVAL = 5.0  # seconds between scheduled sends


async def main() -> None:
    field = "temperature"
    cfg = LoraConfig(txch=65, rxch=65, band="HF")
    async with LoRaDevice(PORT, config=cfg) as dev:

        async def send_data() -> None:
            while True:
                started = asyncio.get_running_loop().time()
                readings = {"temperature": 21.5, "humidity": 45.0}
                payload = {field: readings[field]}
                await dev.send_json(payload)
                print("sent:", payload)
                elapsed = asyncio.get_running_loop().time() - started
                await asyncio.sleep(max(0, INTERVAL - elapsed))

        async def receive_commands() -> None:
            nonlocal field
            async for command, _ in dev.messages():
                if isinstance(command, dict) and command.get("send") in (
                    "temperature", "humidity"
                ):
                    field = command["send"]
                    print("next readings will send:", field)

        await asyncio.gather(send_data(), receive_commands())


if __name__ == "__main__":
    asyncio.run(main())
