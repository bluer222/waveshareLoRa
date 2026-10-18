Note: This repository is primarily AI-generated code.

# waveshareLoRa

An async Python library for sending and receiving JSON over Waveshare
USB-TO-LoRa-xF radios with SX1262 modules. It includes AT configuration,
message framing, compression, and examples for two paired radios.

## Brief usage

Requires Python 3.10+ and two radios configured for the same band, channel,
and radio settings. Clone this repository, then run:

```bash
python -m pip install -e .
# Configure each radio, using its serial port and a locally permitted channel.
python examples/configure.py --port /dev/ttyUSB0 --channel 65 --band HF --mode 1
python examples/send_receive.py --port /dev/ttyUSB0 --channel 65 --band HF
```

Run the send/receive example on each computer with its radio's port.
HF channel 65 is 915 MHz. Choose settings permitted in your location.
Passing a `LoraConfig` to `LoRaDevice` sets host expectations; call
`apply_config()` or use the configure script to write settings to the radio.

## More documentation

- [Usage, Python examples, AT parameters, wire format, hardware notes, and tests](docs/USAGE.md)

## License

GNU Affero General Public License v3.0 only, `AGPL-3.0-only`.
See [LICENCE](LICENCE) for the full text.
