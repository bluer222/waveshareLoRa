"""AT-protocol helpers: command bytes, reply parsing, query dispatch.

Device behaviour (from Waveshare wiki + community captures):
  * Data mode is the default. Send ``+++`` + CRLF *in one write* to enter AT mode.
  * Every AT command ends with CRLF (``\\r\\n``).
  * Replies contain ``OK`` on success, ``ERROR``/``ERR`` on failure, sometimes echo.
  * Query form is ``AT+XXX?`` and returns e.g. ``+SF:7`` (tolerant parser below).
  * ``AT+EXIT`` saves, reboots the module (~3s until TXD/RXD blink).
  * ``AT+RESTORE=1`` restores factory settings.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Final

CRLF: Final[bytes] = b"\r\n"
ENTER_AT: Final[bytes] = b"+++\r\n"

# canonical parameter commands in AllP order
PARAM_CMDS: Final[tuple[str, ...]] = (
    "SF",
    "BW",
    "CR",
    "PWR",
    "NETID",
    "LBT",
    "MODE",
    "TXCH",
    "RXCH",
    "RSSI",
    "ADDR",
    "PORT",
    "BAUD",
    "COMM",
    "KEY",
)

AT_EXIT: Final[str] = "AT+EXIT"
AT_RESTORE: Final[str] = "AT+RESTORE"
AT_VER: Final[str] = "AT+VER"
AT_HELP: Final[str] = "AT+HELP"
AT_ALLP_QUERY: Final[str] = "AT+ALLP?"
AT_REBOOT: Final[str] = "AT+REBOOT"  # accepted by newer firmware; harmless otherwise


def encode(cmd: str) -> bytes:
    """Encode an AT command with CRLF (``+++`` included verbatim)."""
    cmd = cmd.strip()
    if cmd == "+++":
        return ENTER_AT
    return (cmd + "\r\n").encode("ascii", errors="strict")


_VALUE_RE = re.compile(r"^[+:A-Za-z]*\+?([A-Za-z]+)\s*[:=]\s*(.+?)\s*$")


@dataclass
class AtReply:
    raw: str
    ok: bool
    value: str | None = None  # value after ':'/'=' if present
    cmd: str | None = None  # command name if identifiable


def parse_reply(raw: bytes | str, expect: str | None = None) -> AtReply:
    """Parse a raw AT reply block.

    Tolerant: handles echo lines, ``+CMD:value``, ``CMD=value``, bare ``OK``.
    """
    text = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw
    lines = [ln.strip().strip("\x00") for ln in text.replace("\r", "\n").split("\n")]
    lines = [ln for ln in lines if ln]
    upper = [ln.upper() for ln in lines]
    ok = any(ln in ("OK", "+OK") or ln.startswith("OK") for ln in upper)
    err = any(ln.startswith(("ERROR", "ERR", "+ERR")) for ln in upper)
    if err:
        ok = False
    value: str | None = None
    cmd: str | None = None
    # find last line carrying ':' or '=' that is not OK/ERROR
    for ln in lines:
        if ln.upper() in ("OK", "+OK", "ERROR", "ERR"):
            continue
        m = _VALUE_RE.match(ln)
        if m:
            cmd, value = m.group(1).upper(), m.group(2).strip().strip('"')
            # keep scanning: last value wins (query echoes first)
    if expect and cmd is None:
        # bare value reply (e.g. version string) — treat whole block minus OK as value
        body = [ln for ln in lines if ln.upper() not in ("OK", "+OK")]
        if body:
            value = body[-1]
    return AtReply(raw=text, ok=ok, value=value, cmd=cmd)


def query_command(param: str) -> str:
    param = param.upper().lstrip("AT+")
    if param == "ALLP":
        return AT_ALLP_QUERY
    return f"AT+{param}?"


def help_command(param: str) -> str:
    return f"AT+{param.upper().lstrip('AT+')}=?"


def set_command(param: str, value: object) -> str:
    """Build a set-command, quoting COMM automatically: ``AT+COMM="8N1"``."""
    p = param.upper().lstrip("AT+")
    if p == "COMM":
        v = str(value).upper().strip().strip('"')
        return f'AT+COMM="{v}"'
    if p == "ALLP":
        return f"AT+AllP={value}"
    return f"AT+{p}={value}"
