#!/bin/zsh
PROJECT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)" || exit 1
cd "$PROJECT_DIR" || exit 1
if /usr/bin/curl --silent --fail --max-time 1 http://127.0.0.1:8010/status >/dev/null; then
 /usr/bin/open http://127.0.0.1:8010
 exit 0
fi
/usr/bin/open http://127.0.0.1:8010
# Keep installed dependencies outside cloud-synced project storage when available.
RUNTIME_PYTHON="$HOME/Library/Application Support/Mayter/so101-runtime/bin/python"
if [[ ! -x "$RUNTIME_PYTHON" ]]; then
 RUNTIME_PYTHON="$PROJECT_DIR/.lelab-venv/bin/python"
fi
exec "$RUNTIME_PYTHON" mayter_teleop/app.py
