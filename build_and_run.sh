#!/usr/bin/env bash
set -Eeuo pipefail

# 기본값 설정
SRC_DIR="${1:-source}"   # conf.py가 있는 디렉토리
OUT_DIR="${2:-build}"    # HTML 출력 디렉토리
PORT="${3:-8000}"        # 웹서버 포트

# PATH에 ~/.local/bin 추가
export PATH="$HOME/.local/bin:$PATH"

# sphinx-build 실행 경로 탐색
if command -v sphinx-build >/dev/null 2>&1; then
  SPHINX_CMD="sphinx-build"
elif [ -x "$HOME/.local/share/pipx/venvs/sphinx/bin/sphinx-build" ]; then
  SPHINX_CMD="$HOME/.local/share/pipx/venvs/sphinx/bin/sphinx-build"
else
  SPHINX_CMD="python3 -m sphinx"
fi

# conf.py 존재 여부 확인
if [ ! -f "$SRC_DIR/conf.py" ]; then
  echo "conf.py not found in '$SRC_DIR'."
  echo "Usage: $0 <source_dir> <build_dir> [port]"
  exit 1
fi

mkdir -p "$OUT_DIR"

# 문서 빌드
echo "Building Sphinx documentation..."
$SPHINX_CMD -b html "$SRC_DIR" "$OUT_DIR"

# 포트 사용 중이면 종료
if command -v fuser >/dev/null 2>&1; then
  fuser -k "${PORT}/tcp" >/dev/null 2>&1 || true
elif command -v lsof >/dev/null 2>&1; then
  lsof -t -i:"$PORT" | xargs -r kill -9 || true
fi

# 웹서버 실행
echo "Serving documentation at http://localhost:${PORT}"
python3 -m http.server "$PORT" --directory "$OUT_DIR"
