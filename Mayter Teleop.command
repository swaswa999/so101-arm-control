#!/bin/zsh
PROJECT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)" || exit 1
cd "$PROJECT_DIR" || exit 1
if /usr/bin/curl --silent --fail --max-time 1 http://127.0.0.1:8010/status >/dev/null; then
 /usr/bin/open http://127.0.0.1:8010
 exit 0
fi
/usr/bin/open http://127.0.0.1:8010
exec .lelab-venv/bin/python mayter_teleop/loose_motor.py
