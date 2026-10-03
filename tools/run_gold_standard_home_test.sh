#!/usr/bin/env bash
# run_gold_standard_home_test.sh - Canonical Entrypoint for Gold Standard HOME Acceptance Test
# Strictly adheres to docs/EXACT_MOTION_CONTRACT.md and rover-exact-motion skill.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source .env if present
if [ -f "${REPO_DIR}/.env" ]; then
    set -a
    source "${REPO_DIR}/.env"
    set +a
elif [ -f "/home/ron/yahboom-encoder/.env" ]; then
    set -a
    source "/home/ron/yahboom-encoder/.env"
    set +a
fi

echo "[LAUNCHER] Executing Gold Standard Acceptance Test on host Python..."
export PYTHONPATH="${REPO_DIR}:${PYTHONPATH:-}"
python3 -m tools.gold_standard_home_test.cli "$@"
