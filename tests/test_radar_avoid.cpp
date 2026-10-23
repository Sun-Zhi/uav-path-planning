// Guomao teaching radars must change the path: straight corridor is inside the
// beams, the planned path goes around at flight altitude instead of riding the
// ceiling of the bounds.
#include "uav/astar3d.hpp"
#include "uav/environment.hpp"
#include "uav/radar.hpp"
#include "uav/rrt_star.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

uav::MapConfig load_sample() {
  try {
    return uav::load_map_json("configs/sample_map.json");
  } catch (...) {
    return uav::load_map_json("../configs/sample_map.json");
  }
}

uav::RRTStarParams rrt_params(const uav::MapConfig& cfg) {
  uav::RRTStarParams rp;
  rp.informed_sampling = true;
  rp.seed = 42;
  const double dx = cfg.bounds.max_pt.x - cfg.bounds.min_pt.x;
  const double dy = cfg.bounds.max_pt.y - cfg.bounds.min_pt.y;
  const double span = std::max(dx, dy);
  rp.max_iterations = span > 500.0 ? 8000 : 5000;
  rp.step_size = std::max(3.0, cfg.resolution * 1.25);
  rp.rewire_radius = std::max(10.0, cfg.resolution * 4.0);
  rp.goal_tolerance = std::max(4.0, cfg.resolution * 1.5);
  return rp;
}

double high_fraction(const std::vector<uav::Vec3>& path, double z) {
  if (path.empty()) return 1.0;
  int n = 0;
  for (const auto& p : path) {
    if (p.z > z) ++n;
  }
  return static_cast<double>(n) / static_cast<double>(path.size());
}

}  // namespace

int main() {
  using namespace uav;
  const MapConfig cfg = load_sample();
  if (cfg.radars.size() != 2) {
    std::cerr << "expected 2 radars\n";
    return 1;
  }
  if (cfg.buildings.empty()) {
    std::cerr << "expected buildings\n";
    return 1;
  }

  Environment indexed(cfg);
  Vec3 inside = cfg.buildings[0].center();
  inside.z = std::min(1.0, cfg.buildings[0].max_pt.z * 0.5);
  if (!indexed.collides_hard(inside)) {
    std::cerr << "building index missed a point inside the first footprint\n";
    return 1;
  }
  if (indexed.collides_hard(cfg.start) || indexed.collides_hard(cfg.goal)) {
    std::cerr << "start or goal is inside a building\n";
    return 1;
  }
  Vec3 before = cfg.buildings[0].min_pt;
  before.x -= 40;
  before.z = inside.z;
  Vec3 after = cfg.buildings[0].max_pt;
  after.x += 40;
  after.z = inside.z;
  if (!indexed.segment_hits_building(before, after)) {
    std::cerr << "building index missed a segment through the first footprint\n";
    return 1;
  }

  std::vector<Vec3> line;
  for (int i = 0; i <= 20; ++i) {
    const double t = i / 20.0;
    line.push_back(cfg.start + (cfg.goal - cfg.start) * t);
  }
  double straight_mean = 0, straight_max = 0;
  path_pd_stats(cfg.radars, line, cfg.uav_rcs, straight_mean, straight_max);
  if (straight_mean < 0.35 || straight_max < 0.7) {
    std::cerr << "straight corridor is not inside the beams: mean " << straight_mean
              << " max " << straight_max << "\n";
    return 1;
  }

  const auto with_a = AStar3D(indexed).plan(cfg.start, cfg.goal);
  double with_mean = 0, with_max = 0;
  path_pd_stats(cfg.radars, with_a.path, cfg.uav_rcs, with_mean, with_max);

  MapConfig bare = cfg;
  bare.radars.clear();
  Environment open(bare);
  const auto without_a = AStar3D(open).plan(bare.start, bare.goal);

  if (!with_a.success || !without_a.success) {
    std::cerr << "A* failed\n";
    return 1;
  }
  if (!(with_a.path_length > without_a.path_length * 1.7)) {
    std::cerr << "A* did not detour: with " << with_a.path_length << " without "
              << without_a.path_length << "\n";
    return 1;
  }
  if (!(with_mean < 0.08)) {
    std::cerr << "A* path still sits in the beams, mean_pd " << with_mean << "\n";
    return 1;
  }
  if (high_fraction(with_a.path, 190.0) > 0.2) {
    std::cerr << "A* escaped by riding the altitude ceiling\n";
    return 1;
  }

  const auto with_r = RRTStar3D(indexed, rrt_params(cfg)).plan(cfg.start, cfg.goal);
  const auto without_r = RRTStar3D(open, rrt_params(bare)).plan(bare.start, bare.goal);
  double rrt_mean = 0, rrt_max = 0;
  path_pd_stats(cfg.radars, with_r.path, cfg.uav_rcs, rrt_mean, rrt_max);
  if (!with_r.success || !without_r.success) {
    std::cerr << "RRT* failed\n";
    return 1;
  }
  if (!(with_r.path_length > without_r.path_length * 1.4) || !(rrt_mean < 0.15)) {
    std::cerr << "RRT* did not avoid: with " << with_r.path_length << " without "
              << without_r.path_length << " mean_pd " << rrt_mean << "\n";
    return 1;
  }

  std::cout << "radar avoid ok A* " << without_a.path_length << " -> " << with_a.path_length
            << " mean_pd " << straight_mean << " -> " << with_mean
            << " RRT* " << without_r.path_length << " -> " << with_r.path_length
            << " mean_pd " << rrt_mean << "\n";
  return 0;
}
