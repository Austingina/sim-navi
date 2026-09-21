#!/usr/bin/env bash
# 拉取宇树官方 Go2 USD（HuggingFace dataset: unitreerobotics/unitree_model）。
#
# 用法（在仓库根目录）:
#   ./scripts/fetch_unitree_go2_model.sh
#   HF_ENDPOINT=https://hf-mirror.com ./scripts/fetch_unitree_go2_model.sh
#
# 说明:
#   - 不用 `hf download`（设 HF_ENDPOINT 时易与 Xet 冲突，只下到半截）。
#   - 用 curl 按文件下载，支持断点续传 (-C -)。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/third_party/unitree_model"
mkdir -p "$OUT/Go2/usd/configuration"

ENDPOINT="${HF_ENDPOINT:-https://huggingface.co}"
ENDPOINT="${ENDPOINT%/}"
# hf-mirror 与官方 resolve 路径相同
BASE="${ENDPOINT}/datasets/unitreerobotics/unitree_model/resolve/main"

FILES=(
  "Go2/usd/go2.usd"
  "Go2/usd/configuration/go2_description_base.usd"
  "Go2/usd/configuration/go2_description_physics.usd"
  "Go2/usd/configuration/go2_description_sensor.usd"
)

# 清理 hf 客户端残留的空 lock，避免误判
find "$OUT" -name '*.lock' -delete 2>/dev/null || true

if [[ -f "$OUT/Go2/usd/go2.usd" \
   && -f "$OUT/Go2/usd/configuration/go2_description_base.usd" \
   && -f "$OUT/Go2/usd/configuration/go2_description_physics.usd" \
   && -f "$OUT/Go2/usd/configuration/go2_description_sensor.usd" ]]; then
  echo "[ok] Go2 USD 已齐全"
  find "$OUT/Go2" -type f -exec ls -lh {} \;
  exit 0
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "需要 curl" >&2
  exit 1
fi

echo "[fetch] BASE=$BASE"
echo "[fetch] OUT=$OUT"

fail=0
for f in "${FILES[@]}"; do
  dest="$OUT/$f"
  mkdir -p "$(dirname "$dest")"
  url="$BASE/$f"
  echo
  echo "[get] $f"
  # -L 跟随跳转；-C - 断点续传；失败不立刻 exit，汇总后再判
  if curl -L --fail --retry 5 --retry-delay 2 --connect-timeout 30 \
       -C - -o "$dest.part" "$url"; then
    mv -f "$dest.part" "$dest"
    ls -lh "$dest"
  else
    echo "[FAIL] $url" >&2
    rm -f "$dest.part"
    fail=1
  fi
done

echo
if [[ $fail -ne 0 ]]; then
  echo "[err] 有文件失败。可换镜像再试:" >&2
  echo "  HF_ENDPOINT=https://hf-mirror.com $0" >&2
  echo "  # 或直连: unset HF_ENDPOINT; $0" >&2
  exit 1
fi

test -f "$OUT/Go2/usd/go2.usd"
echo "[ok] $OUT/Go2/usd/go2.usd"
find "$OUT/Go2" -type f -exec ls -lh {} \;
echo
echo "下一步: unitree_rl_lab 的 UNITREE_MODEL_DIR 应为:"
echo "  $OUT"
echo "然后: ./run_isaacsim.sh --gui scene_daxuecheng_go2.usd"
