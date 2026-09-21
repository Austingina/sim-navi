# 场景资产（不进 Git）

大文件通过 **GitHub Release** 分发，克隆仓库后需下载并解压到本目录。

Release 标签：见仓库 [Releases](https://github.com/Austingina/sim-navi/releases)（当前为 `assets-v1`）。

## 运行包（仿真必下）

| 附件 | 解压后得到 | 用途 |
|---|---|---|
| `assets-daxuecheng-runtime.tar` | `daxuecheng/daxuecheng-collision*.usdz` | 大学城三场景入口 |
| `assets-zhichengAB-collision.tar` | `zhichengAB/zhichengAB-collision.usdz` | `scene_seg.usd` |
| `assets-zhichengAB-collision-smooth-v3.tar` | `zhichengAB/zhichengAB-collision-smooth-v3.usdz` | `scene.usd` / `scene_seg_smooth.usd` |
| `assets-zhichengAB-collision-smoother.tar` | `zhichengAB/zhichengAB-collision-smoother.usdz` | `scene_seg_nocam.usd` |

### 一键下载（需 [GitHub CLI](https://cli.github.com/)）

在**仓库根目录**执行：

```bash
./scripts/fetch_assets.sh
```

或手动：

```bash
mkdir -p assets && cd assets
# 将 <TAG> 换成 Release 标签，例如 assets-v1
gh release download assets-v1 -R Austingina/sim-navi \
  -p 'assets-daxuecheng-runtime.tar' \
  -p 'assets-zhichengAB-collision.tar' \
  -p 'assets-zhichengAB-collision-smooth-v3.tar' \
  -p 'assets-zhichengAB-collision-smoother.tar'
tar -xf assets-daxuecheng-runtime.tar
tar -xf assets-zhichengAB-collision.tar
tar -xf assets-zhichengAB-collision-smooth-v3.tar
tar -xf assets-zhichengAB-collision-smoother.tar
# tar 内路径已含 assets/...，若在 assets/ 下解压会多一层；推荐在仓库根解压：
# cd .. && tar -xf assets/*.tar
```

**推荐**在仓库根目录解压（tar 内路径为 `assets/...`）：

```bash
cd /path/to/sim-navi
tar -xf /path/to/assets-daxuecheng-runtime.tar
tar -xf /path/to/assets-zhichengAB-collision.tar
tar -xf /path/to/assets-zhichengAB-collision-smooth-v3.tar
tar -xf /path/to/assets-zhichengAB-collision-smoother.tar
ls assets/daxuecheng/daxuecheng-collision.usdz
ls assets/zhichengAB/zhichengAB-collision-smooth-v3.usdz
```

## 场景入口对照

| USD 入口 | 需要的 usdz |
|---|---|
| `scene_daxuecheng.usd` | `assets/daxuecheng/daxuecheng-collision.usdz` |
| `scene_daxuecheng_smooth.usd` | `.../daxuecheng-collision-smooth.usdz` |
| `scene_daxuecheng_v1_smooth.usd` | `.../daxuecheng-collision-v1-smooth.usdz` |
| `scene_seg.usd` | `assets/zhichengAB/zhichengAB-collision.usdz` |
| `scene.usd` / `scene_seg_smooth.usd` | `.../zhichengAB-collision-smooth-v3.usdz` |
| `scene_seg_nocam.usd` | `.../zhichengAB-collision-smoother.usdz` |

地理配准 JSON 已在仓库：`scene_tools/georef_daxuecheng.json`、`georef.json`。

## 换新地图

1. 将 L2Pro 产物放到 `assets/<场景名>/`（PLY / OBJ / 原始 usdz）
2. 用 `scene_tools/add_collision_to_usdz.py` 生成 `*-collision.usdz` 与 georef
3. 复制一份 `scene_daxuecheng.usd`，改 payload 路径为新 usdz
4. bringup 指定对应 `georef_json` 与 `spawn_x/y`

智城重建用超大 PLY（>2GB）未放入 Release；换智城地图需自备 L2Pro 源文件。
