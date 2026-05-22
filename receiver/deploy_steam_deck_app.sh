#!/usr/bin/env bash
set -euo pipefail

DECK_USER="${DECK_USER:-deck}"
DECK_IP="${DECK_IP:-192.168.0.98}"
REMOTE_DIR="${REMOTE_DIR:-~/rotation_sender}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SENDER_DIR="$(cd "${SCRIPT_DIR}/../sender" && pwd)"

echo "Creating ${DECK_USER}@${DECK_IP}:${REMOTE_DIR}"
ssh "${DECK_USER}@${DECK_IP}" "mkdir -p ${REMOTE_DIR}"

echo "Copying Steam Deck sender app files"
scp \
  "${SENDER_DIR}/steam_deck_app.py" \
  "${SENDER_DIR}/tui.py" \
  "${SENDER_DIR}/requirements-steam-deck.txt" \
  "${DECK_USER}@${DECK_IP}:${REMOTE_DIR}/"

cat <<EOF

Copied files to ${DECK_USER}@${DECK_IP}:${REMOTE_DIR}

On the Steam Deck, run:
  cd ${REMOTE_DIR}
  python3 -m venv .venv
  source .venv/bin/activate
  pip install -r requirements-steam-deck.txt
  python3 steam_deck_app.py --port /dev/ttyACM0 --fullscreen

EOF
