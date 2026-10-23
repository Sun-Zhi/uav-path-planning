#pragma once
/**
 * Risk-aware 3D grid A* with 26-neighbor connectivity.
 * Base: Hart, Nilsson, Raphael (1968).
 * UAV 3D comparison motivation: Zammit & van Kampen, AIAA GNC 2018.
 * Soft radar/EW cost: Alpdemir 2022; Atayev et al. 2024.
 */
#include "uav/environment.hpp"
#include "uav/vec3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace uav {

struct PlannerStats {
  bool success{false};
  double path_length{0};
  double path_cost{0};  // risk-weighted
  int nodes_expanded{0};
  int path_waypoints{0};
  double time_ms{0};
  std::vector<Vec3> path;
};

struct GridIndex {
  int ix, iy, iz;
  bool operator==(const GridIndex& o) const {
    return ix == o.ix && iy == o.iy && iz == o.iz;
  }
};

struct GridIndexHash {
  size_t operator()(const GridIndex& g) const {
    return (static_cast<size_t>(g.ix) * 73856093u) ^
           (static_cast<size_t>(g.iy) * 19349663u) ^
           (static_cast<size_t>(g.iz) * 83492791u);
  }
};

class AStar3D {
 public:
  explicit AStar3D(const Environment& env) : env_(env) {}

  PlannerStats plan(const Vec3& start, const Vec3& goal) const {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    PlannerStats stats;

    const double res = env_.config().resolution;
    const auto& b = env_.bounds();
    const int nx = static_cast<int>(std::floor((b.max_pt.x - b.min_pt.x) / res)) + 1;
    const int ny = static_cast<int>(std::floor((b.max_pt.y - b.min_pt.y) / res)) + 1;
    const int nz = static_cast<int>(std::floor((b.max_pt.z - b.min_pt.z) / res)) + 1;

    auto to_grid = [&](const Vec3& p) -> GridIndex {
      return {static_cast<int>(std::round((p.x - b.min_pt.x) / res)),
              static_cast<int>(std::round((p.y - b.min_pt.y) / res)),
              static_cast<int>(std::round((p.z - b.min_pt.z) / res))};
    };
    auto to_world = [&](const GridIndex& g) -> Vec3 {
      return {b.min_pt.x + g.ix * res, b.min_pt.y + g.iy * res,
              b.min_pt.z + g.iz * res};
    };
    auto valid = [&](const GridIndex& g) {
      return g.ix >= 0 && g.iy >= 0 && g.iz >= 0 && g.ix < nx && g.iy < ny &&
             g.iz < nz && env_.is_free(to_world(g));
    };

    GridIndex gs = to_grid(start);
    GridIndex gg = to_grid(goal);
    if (!valid(gs) || !valid(gg)) {
      stats.time_ms = ms_since(t0);
      return stats;
    }

    struct Node {
      GridIndex idx;
      double f;
      bool operator>(const Node& o) const { return f > o.f; }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<GridIndex, double, GridIndexHash> g_score;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> parent;
    std::unordered_map<GridIndex, bool, GridIndexHash> closed;

    auto heuristic = [&](const GridIndex& a, const GridIndex& c) {
      return Vec3::dist(to_world(a), to_world(c));  // admissible Euclidean
    };

    g_score[gs] = 0;
    open.push({gs, heuristic(gs, gg)});

    // 26-neighbor offsets
    std::vector<GridIndex> offs;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz)
          if (dx || dy || dz) offs.push_back({dx, dy, dz});

    while (!open.empty()) {
      Node cur = open.top();
      open.pop();
      if (closed[cur.idx]) continue;
      closed[cur.idx] = true;
      ++stats.nodes_expanded;

      if (cur.idx == gg) {
        // reconstruct
        std::vector<Vec3> rev;
        GridIndex p = gg;
        rev.push_back(goal);  // snap to exact goal
        while (!(p == gs)) {
          rev.push_back(to_world(p));
          p = parent[p];
        }
        rev.push_back(start);
        std::reverse(rev.begin(), rev.end());
        // dedupe near-duplicates at ends
        stats.path = rev;
        stats.success = true;
        stats.path_waypoints = static_cast<int>(rev.size());
        for (size_t i = 1; i < rev.size(); ++i) {
          stats.path_length += Vec3::dist(rev[i - 1], rev[i]);
          stats.path_cost += env_.edge_cost(rev[i - 1], rev[i]);
        }
        stats.time_ms = ms_since(t0);
        return stats;
      }

      Vec3 pw = to_world(cur.idx);
      for (const auto& o : offs) {
        GridIndex nb{cur.idx.ix + o.ix, cur.idx.iy + o.iy, cur.idx.iz + o.iz};
        if (!valid(nb) || closed[nb]) continue;
        Vec3 nw = to_world(nb);
        if (env_.segment_hits_building(pw, nw)) continue;

        const double tent = g_score[cur.idx] + env_.edge_cost(pw, nw);
        auto it = g_score.find(nb);
        if (it == g_score.end() || tent < it->second) {
          g_score[nb] = tent;
          parent[nb] = cur.idx;
          open.push({nb, tent + heuristic(nb, gg)});
        }
      }
    }

    stats.time_ms = ms_since(t0);
    return stats;
  }

 private:
  const Environment& env_;

  static double ms_since(std::chrono::steady_clock::time_point t0) {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now() - t0).count();
  }
};

}  // namespace uav
