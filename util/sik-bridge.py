#!/usr/bin/env python3
"""
Filtered SiK bridge for MAVLink.

This bridge is meant to keep the SiK radio link lightweight (manual control and
essentials only) while leaving full telemetry on UDP/Tailnet outputs.

Flow:
  MAVProxy --out udp:127.0.0.1:14600  -> (filtered) -> SiK serial
  SiK serial -> (unfiltered) -> MAVProxy --in udp:127.0.0.1:14601
"""

from __future__ import annotations

import argparse
import os
import signal
import threading
import time
from typing import Iterable, Set

from pymavlink import mavutil


DEFAULT_ALLOWLIST = {
    "HEARTBEAT",
    "SYS_STATUS",
    "STATUSTEXT",
    "COMMAND_ACK",
    "EXTENDED_SYS_STATE",
    "SYSTEM_TIME",
    "POWER_STATUS",
    "BATTERY_STATUS",
}


def _parse_allowlist(values: Iterable[str]) -> Set[str]:
    allowlist: Set[str] = set()
    for raw in values:
        if not raw:
            continue
        for token in raw.split(","):
            token = token.strip().upper()
            if token:
                allowlist.add(token)
    return allowlist


def _log(prefix: str, message: str) -> None:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] {prefix} {message}")


def _forward_filtered(stop: threading.Event,
                      src: mavutil.mavfile,
                      dst: mavutil.mavfile,
                      allowlist: Set[str]) -> None:
    while not stop.is_set():
        msg = src.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue
        if allowlist and msg_type not in allowlist:
            continue
        try:
            dst.write(msg.get_msgbuf())
        except Exception as exc:
            _log("WARN", f"SiK write failed ({msg_type}): {exc}")
            time.sleep(0.2)


def _forward_all(stop: threading.Event,
                 src: mavutil.mavfile,
                 dst: mavutil.mavfile) -> None:
    while not stop.is_set():
        msg = src.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue
        try:
            dst.write(msg.get_msgbuf())
        except Exception as exc:
            _log("WARN", f"UDP write failed ({msg_type}): {exc}")
            time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Filtered SiK MAVLink bridge")
    parser.add_argument("--udp-in", default="127.0.0.1:14600",
                        help="UDP host:port to receive MAVLink from MAVProxy")
    parser.add_argument("--udp-out", default="127.0.0.1:14601",
                        help="UDP host:port to send MAVLink back into MAVProxy")
    parser.add_argument("--serial", default="/dev/ttyUSB0",
                        help="Serial device for SiK radio")
    parser.add_argument("--baud", type=int, default=57600,
                        help="Serial baud rate for SiK radio")
    parser.add_argument("--allow", action="append", default=[],
                        help="Comma-separated allowlist of MAVLink message types")
    args = parser.parse_args()

    env_allow = os.getenv("SIK_ALLOWLIST", "")
    allowlist = _parse_allowlist(args.allow)
    allowlist.update(_parse_allowlist([env_allow]))
    if not allowlist:
        allowlist = set(DEFAULT_ALLOWLIST)

    _log("INFO", f"Allowlist: {sorted(allowlist)}")
    _log("INFO", f"UDP in: {args.udp_in} | UDP out: {args.udp_out} | Serial: {args.serial}@{args.baud}")

    stop = threading.Event()

    def _handle_signal(_signo, _frame) -> None:
        stop.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    udp_in = mavutil.mavlink_connection(f"udp:{args.udp_in}")
    udp_out = mavutil.mavlink_connection(f"udpout:{args.udp_out}")
    serial = mavutil.mavlink_connection(args.serial, baud=args.baud, autoreconnect=True)

    threads = [
        threading.Thread(
            target=_forward_filtered,
            args=(stop, udp_in, serial, allowlist),
            daemon=True,
        ),
        threading.Thread(
            target=_forward_all,
            args=(stop, serial, udp_out),
            daemon=True,
        ),
    ]
    for t in threads:
        t.start()

    try:
        while not stop.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop.set()

    _log("INFO", "Shutdown requested; exiting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
