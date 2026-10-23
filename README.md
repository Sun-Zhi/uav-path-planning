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
- `resolution`: A\* 栅格边长（米）

---

## 浏览器演示 / Browser demo (WASM)

在线页面（GitHub Pages）：

**https://sun-zhi.github.io/uav-path-planning/**

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

页面用 Three.js 显示样例楼宇与风险区，并在浏览器内调用 WASM 版 A\* / RRT\*（可改起点终点）。

---

## 算法对照（本 demo 设计动机）

与 Zammit & van Kampen (AIAA GNC 2018) 类似，本程序在同一张城市样例图上同时跑 A\* 与 RRT\*：

- **A\***：离散最优（在栅格分辨率下），通常路径更短、更稳。
- **RRT\***：连续空间，适合后续加动力学；随机性导致路径长度有波动；开启 Informed 后收敛更快。

风险区建模对齐 Alpdemir (2022) / Atayev (2024) 的「可穿透威胁 + 代价」思想，而非简单二值障碍。

---

## English (short)

Build with CMake (C++17). Run `./build/uav_planner --demo`. Browser WASM demo: https://sun-zhi.github.io/uav-path-planning/ (rebuild with `./web/build.sh`). Implements risk-aware 26-neighbor 3D A\* and RRT\* (optional Informed ellipsoid sampling) on AABB buildings + soft EW/radar zones. See `PAPERS.md` for citations. No ROS. Exit 0 on success.

---

## License

Prototype for research / education. Cite the papers listed in `PAPERS.md` if you reuse the ideas.
