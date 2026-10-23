#pragma once
/**
 * Continuous 3D RRT* with collision checks against AABB buildings
 * and risk-weighted edge cost through EW/radar soft zones.
 *
 * Karaman & Frazzoli, IJRR 2011 — asymptotic optimality via rewiring.
 * Optional informed sampling (Gammell et al., ICRA 2014 / TRO 2018)
 * can be enabled after an initial solution is found.
 */
#include "uav/environment.hpp"
#include "uav/vec3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace uav {

struct RRTStarParams {
  int max_iterations{4000};
  double step_size{3.0};
  double goal_bias{0.08};
  double rewire_radius{8.0};
  double goal_tolerance{3.0};
  bool informed_sampling{true};  // Gammell Informed RRT* style ellipsoid
  unsigned seed{42};
};

class RRTStar3D {
 public:
  RRTStar3D(const Environment& env, RRTStarParams params = {})
      : env_(env), params_(params) {}

  PlannerStats plan(const Vec3& start, const Vec3& goal) {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    PlannerStats stats;
    rng_.seed(params_.seed);

    if (!env_.is_free(start) || !env_.is_free(goal)) {
      stats.time_ms = ms_since(t0);
      return stats;
    }

    struct Node {
      Vec3 pos;
      int parent{-1};
      double cost{0};  // risk-weighted cost from start
    };
    std::vector<Node> nodes;
    nodes.push_back({start, -1, 0.0});

    int best_goal_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();

    auto sample_uniform = [&]() -> Vec3 {
      const auto& b = env_.bounds();
      std::uniform_real_distribution<double> ux(b.min_pt.x, b.max_pt.x);
      std::uniform_real_distribution<double> uy(b.min_pt.y, b.max_pt.y);
      std::uniform_real_distribution<double> uz(b.min_pt.z, b.max_pt.z);
      return {ux(rng_), uy(rng_), uz(rng_)};
    };

    // Informed RRT*: sample in prolate hyperspheroid that can improve c_best
    auto sample_informed = [&](double c_best) -> Vec3 {
      const double c_min = Vec3::dist(start, goal);
      if (c_best < c_min || !std::isfinite(c_best)) return sample_uniform();

      // Sample unit ball then map to ellipsoid (3D)
      std::normal_distribution<double> gauss(0.0, 1.0);
      Vec3 x;
      do {
        x = {gauss(rng_), gauss(rng_), gauss(rng_)};
      } while (x.norm2() > 1.0 || x.norm2() < 1e-12);
      x = x.normalized() * std::cbrt(std::uniform_real_distribution<double>(0, 1)(rng_));

      const double r1 = c_best / 2.0;
      const double r2 = std::sqrt(std::max(0.0, c_best * c_best - c_min * c_min)) / 2.0;
      Vec3 local{x.x * r1, x.y * r2, x.z * r2};

      Vec3 a = (goal - start).normalized();
      // Build orthonormal frame with 'a' as x-axis
      Vec3 tmp = (std::abs(a.z) < 0.9) ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
      Vec3 b1 = Vec3{a.y * tmp.z - a.z * tmp.y, a.z * tmp.x - a.x * tmp.z,
                     a.x * tmp.y - a.y * tmp.x}
                    .normalized();
      Vec3 b2 = Vec3{a.y * b1.z - a.z * b1.y, a.z * b1.x - a.x * b1.z,
                     a.x * b1.y - a.y * b1.x}
                    .normalized();
      Vec3 mid = (start + goal) * 0.5;
      Vec3 world = mid + a * local.x + b1 * local.y + b2 * local.z;
      if (!env_.in_bounds(world)) return sample_uniform();
      return world;
    };

    auto nearest = [&](const Vec3& q) -> int {
      int best = 0;
      double bd = Vec3::dist(nodes[0].pos, q);
      for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
        double d = Vec3::dist(nodes[i].pos, q);
        if (d < bd) {
          bd = d;
          best = i;
        }
      }
      return best;
    };

    auto steer = [&](const Vec3& from, const Vec3& to) -> Vec3 {
      Vec3 d = to - from;
      double n = d.norm();
      if (n < 1e-9) return from;
      if (n <= params_.step_size) return to;
      return from + d * (params_.step_size / n);
    };

    auto collision_free = [&](const Vec3& a, const Vec3& b) {
      return env_.is_free(b) && !env_.segment_hits_building(a, b);
    };

    auto near_indices = [&](const Vec3& p, double radius) {
      std::vector<int> ids;
      for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (Vec3::dist(nodes[i].pos, p) <= radius) ids.push_back(i);
      }
      return ids;
    };

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int it = 0; it < params_.max_iterations; ++it) {
      ++stats.nodes_expanded;  // iterations as expansion proxy

      Vec3 x_rand;
      if (uni(rng_) < params_.goal_bias) {
        x_rand = goal;
      } else if (params_.informed_sampling && best_goal_idx >= 0) {
        x_rand = sample_informed(best_cost);
      } else {
        x_rand = sample_uniform();
      }

      int n_nearest = nearest(x_rand);
      Vec3 x_new = steer(nodes[n_nearest].pos, x_rand);
      if (!collision_free(nodes[n_nearest].pos, x_new)) continue;

      // ChooseParent
      double r = params_.rewire_radius;
      auto near = near_indices(x_new, r);
      int best_parent = n_nearest;
      double best_new_cost =
          nodes[n_nearest].cost + env_.edge_cost(nodes[n_nearest].pos, x_new);

      for (int ni : near) {
        if (!collision_free(nodes[ni].pos, x_new)) continue;
        double c = nodes[ni].cost + env_.edge_cost(nodes[ni].pos, x_new);
        if (c < best_new_cost) {
          best_new_cost = c;
          best_parent = ni;
        }
      }

      int new_idx = static_cast<int>(nodes.size());
      nodes.push_back({x_new, best_parent, best_new_cost});

      // Rewire
      for (int ni : near) {
        if (ni == best_parent) continue;
        if (!collision_free(x_new, nodes[ni].pos)) continue;
        double c = best_new_cost + env_.edge_cost(x_new, nodes[ni].pos);
        if (c + 1e-9 < nodes[ni].cost) {
          nodes[ni].parent = new_idx;
          nodes[ni].cost = c;
        }
      }

      if (Vec3::dist(x_new, goal) <= params_.goal_tolerance &&
          collision_free(x_new, goal)) {
        double c_goal = best_new_cost + env_.edge_cost(x_new, goal);
        if (c_goal < best_cost) {
          if (best_goal_idx < 0) {
            best_goal_idx = static_cast<int>(nodes.size());
            nodes.push_back({goal, new_idx, c_goal});
          } else {
            nodes[best_goal_idx].parent = new_idx;
            nodes[best_goal_idx].cost = c_goal;
          }
          best_cost = c_goal;
        }
      }
    }

    if (best_goal_idx < 0) {
      // try connect nearest to goal
      int n = nearest(goal);
      if (collision_free(nodes[n].pos, goal) &&
          Vec3::dist(nodes[n].pos, goal) <= params_.rewire_radius * 2) {
        best_goal_idx = static_cast<int>(nodes.size());
        nodes.push_back(
            {goal, n, nodes[n].cost + env_.edge_cost(nodes[n].pos, goal)});
        best_cost = nodes.back().cost;
      }
    }

    if (best_goal_idx >= 0) {
      std::vector<Vec3> rev;
      int cur = best_goal_idx;
      while (cur >= 0) {
        rev.push_back(nodes[cur].pos);
        cur = nodes[cur].parent;
      }
      std::reverse(rev.begin(), rev.end());
      stats.path = rev;
      stats.success = true;
      stats.path_waypoints = static_cast<int>(rev.size());
      stats.path_cost = best_cost;
      for (size_t i = 1; i < rev.size(); ++i)
        stats.path_length += Vec3::dist(rev[i - 1], rev[i]);
    }

    stats.time_ms = ms_since(t0);
    return stats;
  }

 private:
  const Environment& env_;
  RRTStarParams params_;
  std::mt19937 rng_;

  static double ms_since(std::chrono::steady_clock::time_point t0) {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now() - t0).count();
  }
};

}  // namespace uav
