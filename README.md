# UAV Path Planning Prototype / 无人机航迹规划原型

C++17 demo: **risk-aware 3D A\*** + **RRT\*** (optional Informed sampling) for urban AABB buildings and soft radar/EW interference zones.

中文说明见下文；English summary at the end.

---

## 功能概览

| 模块 | 说明 | 论文灵感 |
|------|------|----------|
| 3D AABB 环境 | 楼宇硬障碍 + 雷达/电子干扰软风险区 | Liu SFC (RA-L 2017); Bertram (T-ITS 2023); Atayev (2024) |
| A\* 26 邻域 | 栅格搜索，边代价 = 距离 × 风险倍率 | Hart et al. 1968; Zammit AIAA 2018 |
| RRT\* | 连续空间采样 + ChooseParent/Rewire | Karaman & Frazzoli, IJRR 2011 |
| Informed 采样 | 首解后在椭球内采样（可关） | Gammell et al., ICRA 2014 / TRO 2018 |
| SNR→Pd 雷达 | 雷达方程 + 波束增益 → 检测概率，边代价 ×(1+w·Pd) | Atayev 2024; Alpdemir 2022；公开 SPY-6 / JY-27 文献（教学近似） |

详细论文短名单见 [PAPERS.md](PAPERS.md)。

**不是**完整飞控 / ROS 栈；无动力学约束（Hybrid A\* / SFC+QP 留作扩展）。

---

## 目录结构

```
uav-path-planning/
├── CMakeLists.txt
├── PAPERS.md
├── README.md
├── configs/sample_map.json
├── docs/                 # GitHub Pages demo
│   ├── index.html
│   ├── app.js
│   ├── sample_map.json
│   ├── planner.js
│   └── planner.wasm
├── web/
│   ├── planner_bind.cpp  # Embind glue
│   └── build.sh          # Emscripten build
├── include/uav/
│   ├── vec3.hpp
│   ├── aabb.hpp
│   ├── radar.hpp
│   ├── environment.hpp
│   ├── astar3d.hpp
│   └── rrt_star.hpp
└── src/main.cpp
```

---

## 编译与运行

```bash
cd /workspace/uav-path-planning
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/uav_planner --demo
```

可选：

```bash
./build/uav_planner --demo --csv /tmp/uav_path --no-informed
ctest --test-dir build --output-on-failure
```

成功时退出码为 **0**，并打印两种算法的路径长度、风险代价、节点/迭代数与耗时。

### 地图配置

编辑 `configs/sample_map.json`：

- `buildings`: 硬障碍 AABB（不可穿越）
- `risk_zones`: 软风险区，`risk_multiplier` 放大穿越代价（可穿越但昂贵）
- `radars`: SNR→Pd 教学雷达（见下节）
- `uav_rcs` / `pd_cost_weight`: 默认 RCS 与 Pd 代价权重
- `resolution`: A\* 栅格边长（米）

### 雷达 SNR→Pd 模型（教学近似）

紧凑雷达方程：`SNR = K · σ / R⁴ · G_beam`，其中 `K` 由 `snr_ref_linear` 在 `r_ref_m` / `sigma_ref_m2` 标定；`G_beam` 为方位/俯仰半功率波束宽上的高斯衰落（后瓣与超距为 0）。检测概率用 logistic 门限映射 SNR_dB → Pd。边代价：

`dist × AABB_risk × (1 + pd_cost_weight × max_Pd_on_segment)`

样例地图含两部**实验室缩距**剖面（非实装参数）：

| id | 灵感（公开文献） | 波段标签 | 特点 |
|----|------------------|----------|------|
| `us_spy6_class` | AN/SPY-6 S-band AESA | S | 较窄波束、较高 snr_ref |
| `cn_jy27_class` | JY-27 VHF 预警 | VHF | 更宽波束；`rcs_lo_factor=4` 定性体现米波对隐身不敏感 |

真实系统作用距离为数百公里量级；geo 演示将 `max_range_m` / `r_ref_m` **实验室缩放到几公里局部地图**（约 1–2 km），旧米制样例曾缩到约 100 m 盒内。UI 与 README 均标注 **teaching / scaled / 非实装**。

---

## 浏览器演示 / Browser demo (Cesium + OSM)

在线页面（GitHub Pages）：

**https://sun-zhi.github.io/uav-path-planning/**

**用法：** 任意经纬度（默认北京国贸 CBD：起点约 `39.908°N, 116.460°E`，终点约 `39.912°N, 116.455°E`，高度 80–120 m）→ **加载地图**（Overpass 拉当地 OSM 建筑轮廓，卫星底图）→ **Plan**（WASM A\* / RRT\*）→ **▶模拟飞行**。CN/US 教学雷达开关可过滤代价与场景图标。

默认打开会自动拉一次国贸一带 OSM；改经纬度后请再点「加载地图」。起终点外扩约 1.5 km，单次建筑上限约 800 栋。

本地预览（需先有 `docs/planner.{js,wasm}`）：

```bash
cd docs
python3 -m http.server 8080
# 打开 http://localhost:8080/
```

重新编译 WASM（需 [Emscripten](https://emscripten.org/)；脚本会在缺少时安装到 `/workspace/emsdk`）：

```bash
./web/build.sh
```

**技术栈：** CesiumJS 真地球 + Esri World Imagery（可切 OSM 栅格）+ Overpass OSM 建筑 footprint（`height` / `building:levels`×3 m，缺省 15 m）→ 局部 ENU 米制规划 → 航迹回投 WGS84。

**署名 / Attributions：** © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors；影像 Esri World Imagery（Maxar / Earthstar 等）；地球引擎 [CesiumJS](https://cesium.com/cesiumjs/)。

**雷达免责：** SPY-6 / JY-27 类剖面仅为公开文献启发的 **teaching / lab-scaled** 近似，**非实装参数**；作用距离已缩放到演示局部几公里地图，不可用于真实任务。

---

## 算法对照（本 demo 设计动机）

与 Zammit & van Kampen (AIAA GNC 2018) 类似，本程序在同一张城市样例图上同时跑 A\* 与 RRT\*：

- **A\***：离散最优（在栅格分辨率下），通常路径更短、更稳。
- **RRT\***：连续空间，适合后续加动力学；随机性导致路径长度有波动；开启 Informed 后收敛更快。

风险区建模对齐 Alpdemir (2022) / Atayev (2024) 的「可穿透威胁 + 代价」思想，而非简单二值障碍。

---

## English (short)

Build with CMake (C++17). Run `./build/uav_planner --demo`. Browser demo (Cesium + OSM Overpass, WGS84 lat/lon): https://sun-zhi.github.io/uav-path-planning/ (rebuild WASM with `./web/build.sh`). Default Beijing Guomao; any lat/lon → 加载地图 → Plan → fly. Risk-aware 26-neighbor 3D A\* and RRT\* (optional Informed) on OSM buildings + teaching SNR→Pd radars (SPY-6 / JY-27–class, lab-scaled to few-km map). Attributions: OSM, Esri imagery, CesiumJS. See `PAPERS.md`. No ROS. Exit 0 on success.

---

## License

Prototype for research / education. Cite the papers listed in `PAPERS.md` if you reuse the ideas.
