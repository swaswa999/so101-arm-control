#!/bin/zsh
PROJECT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)" || exit 1
export PATH="$PROJECT_DIR/.lelab-venv/bin:/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin"
cd "$PROJECT_DIR/lelab" || exit 1
set -o pipefail
export PYTHONUNBUFFERED=1
"$PROJECT_DIR/.lelab-venv/bin/lelab" --no-open 2>&1 | /usr/bin/tee -a "$PROJECT_DIR/lelab-server.log"
