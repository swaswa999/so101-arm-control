#!/bin/zsh
PROJECT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)" || exit 1
export PATH="$PROJECT_DIR/.lelab-venv/bin:/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin"
cd "$PROJECT_DIR/lelab" || exit 1
if /usr/bin/curl --silent --fail --max-time 2 http://127.0.0.1:8000/robots | /usr/bin/grep -q 'Hiwonder SO101'; then
    /usr/bin/open http://localhost:8000
    exit 0
fi
exec "$PROJECT_DIR/.lelab-venv/bin/lelab" >> "$PROJECT_DIR/lelab-server.log" 2>&1
