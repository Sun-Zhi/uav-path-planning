# 无人机避障与航迹规划：核心论文短名单

本仓库原型实现受下列顶会/顶刊与高影响力工作启发。实现侧重**可运行的算法正确性**，而非完整飞控栈。

---

## 一、几何 / 栅格规划器（基础算法）

### 1. A* — Hart, Nilsson, Raphael (1968)
- **标题**: A Formal Basis for the Heuristic Determination of Minimum Cost Paths  
- **作者**: Peter E. Hart, Nils J. Nilsson, Bertram Raphael  
- **年份 / 期刊**: 1968, *IEEE Transactions on Systems Science and Cybernetics*  
- **核心观点**: 在图上用 \(f=g+h\) 有序扩展；当启发函数可采纳（admissible）时保证最优。  
- **对本项目的意义**: 我们实现的 **3D 栅格 A\***（26 邻域）直接继承该框架，并把雷达/电磁干扰区建模为**额外边代价**（risk-aware cost），而不是硬障碍。

### 2. Hybrid A* — Dolgov, Thrun, Montemerlo, Diebel (2008/2010)
- **标题**: Practical Search Techniques in Path Planning for Autonomous Driving / Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments  
- **作者**: Dmitri Dolgov et al.  
- **年份 / 会议**: 2008 STAIR；2010 *IJRR*  
- **核心观点**: 在离散搜索节点中保留连续运动学状态，使路径满足转向约束，再经非线性优化平滑。  
- **对本项目的意义**: 说明「栅格搜索 + 连续可行性」是工程主流路线。本原型未做完整 Hybrid A*（无动力学），但 A* 模块与后续可扩展的姿态离散化接口对齐。

### 3. RRT* / PRM* — Karaman & Frazzoli (2011)
- **标题**: Sampling-based Algorithms for Optimal Motion Planning  
- **作者**: Sertac Karaman, Emilio Frazzoli  
- **年份 / 期刊**: 2011, *IJRR*  
- **核心观点**: 经典 RRT 几乎必然不收敛到最优；RRT* 通过邻域重布线（rewire）实现**几乎必然渐近最优**。  
- **对本项目的意义**: 我们实现的 **连续 3D RRT\*** 按该文：随机采样、最近邻扩展、半径邻域内 ChooseParent + Rewire，碰撞检测对 AABB 建筑物做线段相交。

### 4. Informed RRT* — Gammell, Srinivasa, Barfoot (2014/2018)
- **标题**: Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic / Informed Sampling for Asymptotically Optimal Path Planning  
- **作者**: Jonathan D. Gammell et al.  
- **年份 / 会议**: 2014 ICRA；2018 *TRO*  
- **核心观点**: 找到初始解后，仅在能改进代价的**椭球启发式可行域**内采样，加速收敛。  
- **对本项目的意义**: README 中说明可在 RRT* 找到首条路径后切换为 Informed 采样；当前 demo 实现标准 RRT*，代码中预留了 `informed_sampling` 开关与注释。

### 5. BIT* — Gammell, Barfoot, Srinivasa (2015/2020)
- **标题**: Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs  
- **作者**: Jonathan D. Gammell et al.  
- **年份 / 期刊**: 2015 RSS；2020 *IJRR*  
- **核心观点**: 批量采样 + A* 式按启发排序搜索隐式随机几何图，兼具 anytime 与渐近最优。  
- **对本项目的意义**: 作为「下一阶段」推荐算法；本原型优先落地 A* 与 RRT* 以便对照栅格 vs 采样。

---

## 二、城市 / 建筑物三维避障

### 6. Safe Flight Corridors — Liu et al. (2017)
- **标题**: Planning Dynamically Feasible Trajectories for Quadrotors Using Safe Flight Corridors in 3-D Complex Environments  
- **作者**: Sikang Liu, Michael Watterson, Kartik Mohta, …, Vijay Kumar  
- **年份 / 期刊**: 2017, *IEEE RA-L*  
- **核心观点**: 用图搜索得到折线骨架，再凸分解为重叠多面体安全飞行走廊（SFC），在走廊内用 QP 生成动力学可行轨迹。  
- **对本项目的意义**: 城市建筑群用 **AABB 障碍**近似楼宇；规划输出的折线路径可作为后续 SFC/QP 的骨架（本仓库停在几何路径层）。

### 7. Urban UAS 导航与避障 — Bertram, Zambreno, Wei (2023)
- **标题**: Efficient Unmanned Aerial Systems Navigation With Collision Avoidance in Dense Urban Environments  
- **作者**: Josh Bertram, Joseph Zambreno, Peng Wei  
- **年份 / 期刊**: 2023, *IEEE T-ITS*  
- **核心观点**: 中轴图做全局导航，本地用 ORCA / RRT* / FastMDP 避楼与避机；稠密三维城市中 FastMDP 更稳。  
- **对本项目的意义**: 支撑「建筑 AABB + 局部采样规划」场景设计；样本地图模拟低空楼宇峡谷。

### 8. 城市场景 A* vs RRT — Zammit & van Kampen (2018)
- **标题**: Comparison between A* and RRT Algorithms for UAV Path Planning  
- **作者**: Christian Zammit, Erik-Jan van Kampen  
- **年份 / 会议**: 2018, *AIAA GNC*  
- **核心观点**: 多组 3D 场景对比显示 A* 路径更短、耗时更少，适合在线重规划；RRT 更适合自由探索。  
- **对本项目的意义**: 本 demo **同时跑两种算法并打印路径长度/节点/时间**，便于复现该类对比实验。

---

## 三、雷达 / 电磁对抗 / GPS 拒止环境下的规划

### 9. 电磁环境下的 UAV 轨迹优化 — Atayev, Fliege, Zemkoho (2024)
- **标题**: Trajectory optimization of unmanned aerial vehicles in the electromagnetic environment  
- **作者**: Anvarbek Atayev, Jörg Fliege, Alain Zemkoho  
- **年份 / 期刊**: 2024, *Optimization and Engineering*  
- **核心观点**: 从电磁频谱检测原理建模被探测概率；目标可最小化探测概率或最大化敌方定位误差，并可纳入我方干扰（jamming）。  
- **对本项目的意义**: 我们将雷达/电子干扰区抽象为 **软障碍风险场**（进入该 AABB 区域边代价 × `risk_multiplier`），在 A* 与 RRT* 代价中一致体现。

### 10. 雷达威胁下的战术 UAV 路径 — Alpdemir (2022)
- **标题**: Tactical UAV path optimization under radar threat using deep reinforcement learning  
- **作者**: M. Nedim Alpdemir  
- **年份 / 期刊**: 2022, *Neural Computing and Applications*  
- **核心观点**: 雷达探测与 RCS、交会几何相关；威胁区在一定条件下可穿透，适合用概率代价而非纯二值障碍。  
- **对本项目的意义**: 印证「风险区可穿越但昂贵」的建模；本原型用固定 `risk_cost`，不做 RL。

### 11. GPS/通信拒止多机规划 — (Sensors / PMC, 2023)
- **标题**: Multi-UAV Path Planning in GPS and Communication Denial Environment  
- **核心观点**: GPS 拒止下传统基于精确坐标的规划失效，需视觉定位 + 学习策略，并避开防空雷达锥形探测区。  
- **对本项目的意义**: 说明拒止环境中地图/定位不确定时仍需风险场；本仓库假设已知地图，聚焦路径层。

### 12. 雷达盲区突防在线规划 — Ge et al. (2024, AEAT)
- **标题**: Online penetration trajectory planning using blind areas of network radar system for an unmanned combat aerial vehicle  
- **核心观点**: 耦合 DEM、雷达探测概率与非完整运动学，用自适应 Hybrid A* 在线求低探测概率突防路径。  
- **对本项目的意义**: 与我们的「硬障碍（楼）+ 软风险（雷达）」分层一致；完整运动学留给后续 Hybrid A*。

---

## 四、本仓库具体实现了什么

| 模块 | 文件 | 灵感来源 |
|------|------|----------|
| 3D AABB 环境 + 风险区 | `include/uav/environment.hpp` | Liu SFC / Bertram 城市场景；Atayev 电磁风险 |
| 风险感知 26 邻域 A* | `include/uav/astar3d.hpp` | Hart A*；Zammit 对比；Alpdemir 软威胁 |
| 连续 3D RRT* | `include/uav/rrt_star.hpp` | Karaman & Frazzoli；Informed 注释对齐 Gammell |
| 地图 JSON | `configs/sample_map.json` | 简化城市楼宇 + 雷达风险球/盒 |
| CLI / demo | `src/main.cpp` | Zammit 式双算法对照输出 |

**未实现（有意控制范围）**: 完整 Hybrid A*、BIT*、SFC+QP、动力学约束、真实雷达 RCS 模型、ROS。

---

## 五、推荐阅读顺序

1. Hart A* → Karaman RRT*（算法根基）  
2. Liu Safe Flight Corridors（城市四旋翼工程范式）  
3. Atayev / Alpdemir（电磁与雷达风险建模）  
4. Gammell Informed RRT* / BIT*（若需加速采样规划）
