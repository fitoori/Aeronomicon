#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Send Discord webhook messages for cron-triggered health pings.

Example cron entry (every 6 hours):
  0 */6 * * * DISCORD_WEBHOOK_URL=... /home/pi/Aeronomicon/util/discord-heartbeat.py alive
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
import urllib.request
from typing import Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DISCORD_WEBHOOK_URL = (
    "https://discord.com/api/webhooks/"
    "1454669304749621282/"
    "8LQlnXje9YyXtImmy7eDvARFAg7tYKqf16vuLiJPkQHW4OFXzJLAPSje7vVAi43jHqop"
)
LTE_SIGNAL_SCRIPT = os.environ.get(
    "LTE_SIGNAL_SCRIPT",
    os.path.join(SCRIPT_DIR, "lte-signal-strength.sh"),
)


def time_of_day_label(now: Optional[dt.datetime] = None) -> str:
    current = now or dt.datetime.now()
    hour = current.hour
    if 5 <= hour < 12:
        return "this morning"
    if 12 <= hour < 17:
        return "this afternoon"
    if 17 <= hour < 21:
        return "this evening"
    return "tonight"


def read_signal_strength_percent() -> str:
    try:
        result = subprocess.run(
            [LTE_SIGNAL_SCRIPT],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10,
        )
    except (subprocess.SubprocessError, FileNotFoundError):
        return "NaN"

    match = re.search(r"Signal Strength: ([0-9.]+)%", result.stdout)
    if not match:
        return "NaN"
    return match.group(1)


def build_message(message_type: str) -> str:
    label = time_of_day_label()
    if message_type == "alive":
        percent = read_signal_strength_percent()
        return f"I'm alive! Signal Strength is {percent}% {label}."
    if message_type == "ping":
        return f"Ping {label}."
    return message_type


def send_webhook(webhook_url: str, content: str, *, no_proxy: bool) -> None:
    payload = json.dumps({"content": content}).encode("utf-8")
    request = urllib.request.Request(
        webhook_url,
        data=payload,
        headers={
            "Content-Type": "application/json",
            "User-Agent": "Aeronomicon-Discord-Heartbeat/1.0",
        },
        method="POST",
    )
    try:
        opener = urllib.request.build_opener()
        if no_proxy:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request, timeout=10) as response:
            if response.status >= 300:
                body = response.read().decode("utf-8", errors="replace")
                raise RuntimeError(f"Discord webhook failed ({response.status}): {body}")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Discord webhook failed ({exc.code}): {body}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send Discord webhook messages.")
    parser.add_argument(
        "message",
        help="Message type (e.g., 'alive') or a literal message to send.",
    )
    parser.add_argument(
        "--webhook-url",
        default=os.environ.get("DISCORD_WEBHOOK_URL", DEFAULT_DISCORD_WEBHOOK_URL),
        help="Discord webhook URL (or set DISCORD_WEBHOOK_URL).",
    )
    parser.add_argument(
        "--no-proxy",
        action="store_true",
        default=os.environ.get("DISCORD_NO_PROXY", "0") == "1",
        help="Bypass proxy settings for the webhook request.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.webhook_url:
        sys.stderr.write("ERROR: DISCORD_WEBHOOK_URL is not set.\n")
        return 2

    content = build_message(args.message)
    try:
        send_webhook(args.webhook_url, content, no_proxy=args.no_proxy)
    except Exception as exc:
        sys.stderr.write(f"ERROR: {exc}\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
