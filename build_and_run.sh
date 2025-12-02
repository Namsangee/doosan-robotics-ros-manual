#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

SRC_DIR="${ROOT_DIR}/source"
OUT_DIR="${ROOT_DIR}/_site"
PORT="${1:-8000}"

if ! command -v sphinx-multiversion >/dev/null 2>&1; then
  echo "sphinx-multiversion is not installed."
  echo "Installing dependencies from requirements.txt..."
  pip install -r "${ROOT_DIR}/requirements.txt"
fi

if ss -ltn | grep -q ":${PORT} "; then
  PORT=$((PORT+1))
fi

echo "Cleaning previous build..."
rm -rf "${OUT_DIR}"

echo "Building multi-version documentation..."
sphinx-multiversion "${SRC_DIR}" "${OUT_DIR}"

if [[ -d "${OUT_DIR}/jazzy" ]]; then
  URL="http://localhost:${PORT}/jazzy/index.html"
else
  URL="http://localhost:${PORT}/humble/index.html"
fi

cd "${OUT_DIR}"

echo ""
echo "Serving at: ${URL}"
echo "Press Ctrl+C to stop."
echo ""

if command -v xdg-open >/dev/null 2>&1; then
  (sleep 1 && xdg-open "${URL}" >/dev/null 2>&1) &
elif command -v open >/dev/null 2>&1; then
  (sleep 1 && open "${URL}") &
fi

python3 -m http.server "${PORT}"
