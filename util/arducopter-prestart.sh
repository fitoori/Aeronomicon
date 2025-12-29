#!/usr/bin/env bash
set -euo pipefail

/usr/bin/python3 /home/pi/Aeronomicon/util/discord/discord.py \
    --webhook https://discord.com/api/webhooks/1454669304749621282/8LQlnXje9YyXtImmy7eDvARFAg7tYKqf16vuLiJPkQHW4OFXzJLAPSje7vVAi43jHqop \
    "I'm alive!"
