#!/usr/bin/env bash
# 从 GitHub Release 下载大学城 + 智城运行资产并解压到仓库根目录。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TAG="${ASSETS_RELEASE_TAG:-assets-v1}"
REMOTE_REPO="${ASSETS_GITHUB_REPO:-Austingina/sim-navi}"
WORKDIR="${TMPDIR:-/tmp}/sim-navi-assets-$$"

# 完整 tar（直接解压）
TARS=(
  assets-daxuecheng-runtime.tar
  assets-zhichengAB-collision.tar
  assets-zhichengAB-collision-smooth-v3.tar
)

# smoother 因体积在慢链路上易超时，Release 拆成分片后本地拼接
SMOOTHER_PARTS=(
  assets-zhichengAB-collision-smoother.tar.part0
  assets-zhichengAB-collision-smoother.tar.part1
  assets-zhichengAB-collision-smoother.tar.part2
)
SMOOTHER_TAR=assets-zhichengAB-collision-smoother.tar

if ! command -v gh >/dev/null 2>&1; then
  echo "[error] 需要 GitHub CLI (gh)。安装: https://cli.github.com/" >&2
  echo "        或手动从 https://github.com/${REMOTE_REPO}/releases/tag/${TAG} 下载附件后，在仓库根目录按 assets/README.md 解压" >&2
  exit 1
fi

mkdir -p "$WORKDIR"
cleanup() { rm -rf "$WORKDIR"; }
trap cleanup EXIT

echo "[info] 下载 Release ${TAG} @ ${REMOTE_REPO} → ${WORKDIR}"
DOWNLOAD_ARGS=()
for f in "${TARS[@]}" "${SMOOTHER_PARTS[@]}"; do
  DOWNLOAD_ARGS+=(-p "$f")
done
(
  cd "$WORKDIR"
  gh release download "$TAG" -R "$REMOTE_REPO" "${DOWNLOAD_ARGS[@]}"
)

echo "[info] 拼接 ${SMOOTHER_TAR}"
cat \
  "$WORKDIR/${SMOOTHER_TAR}.part0" \
  "$WORKDIR/${SMOOTHER_TAR}.part1" \
  "$WORKDIR/${SMOOTHER_TAR}.part2" \
  >"$WORKDIR/$SMOOTHER_TAR"

cd "$REPO_ROOT"
for f in "${TARS[@]}" "$SMOOTHER_TAR"; do
  echo "[info] 解压 $f"
  tar -xf "$WORKDIR/$f"
done

echo "[ok] 校验："
ls -lh \
  assets/daxuecheng/daxuecheng-collision.usdz \
  assets/daxuecheng/daxuecheng-collision-smooth.usdz \
  assets/daxuecheng/daxuecheng-collision-v1-smooth.usdz \
  assets/zhichengAB/zhichengAB-collision.usdz \
  assets/zhichengAB/zhichengAB-collision-smooth-v3.usdz \
  assets/zhichengAB/zhichengAB-collision-smoother.usdz
echo "[ok] 完成。"
