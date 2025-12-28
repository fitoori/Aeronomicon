#!/usr/bin/env python3
import sys
import json
import re
import argparse
import requests

WEBHOOK_RE = re.compile(r"^https://discord\.com/api/webhooks/\d+/[\w-]+$")

def die(msg, code=1):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(code)

def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a message to a Discord webhook"
    )
    parser.add_argument(
        "--webhook",
        required=True,
        help="Discord webhook URL"
    )
    parser.add_argument(
        "message",
        nargs="+",
        help="Message content to send"
    )
    return parser.parse_args()

def main():
    args = parse_args()

    webhook_url = args.webhook.strip()
    message = " ".join(args.message).strip()

    if not webhook_url:
        die("Webhook URL is empty")

    if not WEBHOOK_RE.match(webhook_url):
        die("Invalid Discord webhook URL format")

    if not message:
        die("Message is empty")

    payload = {
        "content": message
    }

    try:
        response = requests.post(
            webhook_url,
            data=json.dumps(payload),
            headers={"Content-Type": "application/json"},
            timeout=5
        )
    except requests.RequestException as e:
        die(f"HTTP request failed: {e}")

    # Discord success = 204 No Content
    if response.status_code != 204:
        die(f"Discord rejected request: HTTP {response.status_code} - {response.text}")

if __name__ == "__main__":
    main()
