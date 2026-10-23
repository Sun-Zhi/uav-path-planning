#include "uav/astar3d.hpp"
#include "uav/environment.hpp"
#include "uav/radar.hpp"
#include "uav/rrt_star.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace {

void print_stats(const char* name, const uav::PlannerStats& s, double mean_pd,
                 double max_pd) {
  std::cout << "=== " << name << " ===\n";
  std::cout << "  success:        " << (s.success ? "yes" : "no") << "\n";
  std::cout << "  path_length_m:  " << s.path_length << "\n";
  std::cout << "  path_cost_risk: " << s.path_cost << "\n";
  std::cout << "  waypoints:      " << s.path_waypoints << "\n";
  std::cout << "  nodes/iters:    " << s.nodes_expanded << "\n";
  std::cout << "  time_ms:        " << s.time_ms << "\n";
  std::cout << "  mean_pd:        " << mean_pd << "\n";
  std::cout << "  max_pd:         " << max_pd << "\n";
}

bool write_csv(const std::string& path, const std::vector<uav::Vec3>& pts) {
  std::ofstream out(path);
  if (!out) return false;
  out << "x,y,z\n";
  for (const auto& p : pts) out << p.x << "," << p.y << "," << p.z << "\n";
  return true;
}

std::string default_map_path() {
  const char* candidates[] = {"../configs/sample_map.json",
                              "configs/sample_map.json",
                              "/workspace/uav-path-planning/configs/sample_map.json"};
  for (const char* c : candidates) {
    std::ifstream in(c);
    if (in.good()) return c;
  }
  return "configs/sample_map.json";
}

void usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [--demo] [--map FILE] [--csv PREFIX] [--no-informed]\n"
      << "  --demo          Run both planners on sample map (default)\n"
      << "  --map FILE      Map JSON path\n"
      << "  --csv PREFIX    Write PREFIX_astar.csv and PREFIX_rrtstar.csv\n"
      << "  --no-informed   Disable Informed RRT* ellipsoid sampling\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path = default_map_path();
  std::string csv_prefix;
  bool informed = true;
  bool demo = true;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--demo") {
      demo = true;
    } else if (a == "--help" || a == "-h") {
      usage(argv[0]);
      return 0;
    } else if (a == "--map" && i + 1 < argc) {
      map_path = argv[++i];
    } else if (a == "--csv" && i + 1 < argc) {
      csv_prefix = argv[++i];
    } else if (a == "--no-informed") {
      informed = false;
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      usage(argv[0]);
      return 2;
    }
  }

  (void)demo;
  std::cout << "UAV Path Planning Demo (C++17)\n";
  std::cout << "Map: " << map_path << "\n";

  uav::MapConfig cfg;
  try {
    cfg = uav::load_map_json(map_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load map: " << e.what() << "\n";
    return 1;
  }

  uav::Environment env(cfg);
  std::cout << "Buildings: " << cfg.buildings.size()
            << "  Risk zones: " << cfg.risk_zones.size()
            << "  Radars: " << cfg.radars.size()
            << "  uav_rcs: " << cfg.uav_rcs
            << "  pd_cost_weight: " << cfg.pd_cost_weight
            << "  resolution: " << cfg.resolution << " m\n";
  for (const auto& r : cfg.radars) {
    std::cout << "  radar " << r.id << " [" << r.band << "] @ " << r.position
              << " yaw=" << r.yaw_deg << "° range=" << r.max_range_m << " m\n";
  }
  std::cout << "Start: " << cfg.start << "  Goal: " << cfg.goal << "\n\n";

  // Straight-line reference Pd (ignore buildings) for comparison
  {
    double mean_sl = 0, max_sl = 0;
    std::vector<uav::Vec3> line;
    for (int i = 0; i <= 20; ++i) {
      double t = i / 20.0;
      line.push_back(cfg.start + (cfg.goal - cfg.start) * t);
    }
    uav::path_pd_stats(cfg.radars, line, cfg.uav_rcs, mean_sl, max_sl);
    std::cout << "Straight-line ref (ignore obstacles): mean_pd=" << mean_sl
              << " max_pd=" << max_sl << "\n\n";
  }

  uav::AStar3D astar(env);
  auto astar_stats = astar.plan(cfg.start, cfg.goal);
  double a_mean = 0, a_max = 0;
  uav::path_pd_stats(cfg.radars, astar_stats.path, cfg.uav_rcs, a_mean, a_max);
  print_stats("A* 3D (26-neighbor, risk+Pd-aware)", astar_stats, a_mean, a_max);

  // Scale RRT* steps to map size / resolution (geo CBD maps are ~km-scale).
  uav::RRTStarParams rp;
  rp.informed_sampling = informed;
  const double res = cfg.resolution;
  const double dx = cfg.bounds.max_pt.x - cfg.bounds.min_pt.x;
  const double dy = cfg.bounds.max_pt.y - cfg.bounds.min_pt.y;
  const double span = std::max(dx, dy);
  rp.max_iterations = span > 500.0 ? 8000 : 5000;
  rp.step_size = std::max(3.0, res * 1.25);
  rp.rewire_radius = std::max(10.0, res * 4.0);
  rp.goal_tolerance = std::max(4.0, res * 1.5);
  uav::RRTStar3D rrt(env, rp);
  auto rrt_stats = rrt.plan(cfg.start, cfg.goal);
  double r_mean = 0, r_max = 0;
  uav::path_pd_stats(cfg.radars, rrt_stats.path, cfg.uav_rcs, r_mean, r_max);
  print_stats(informed ? "RRT* 3D (Informed after first solution)"
                       : "RRT* 3D (uniform sampling)",
              rrt_stats, r_mean, r_max);

  if (!csv_prefix.empty()) {
    if (astar_stats.success)
      write_csv(csv_prefix + "_astar.csv", astar_stats.path);
    if (rrt_stats.success)
      write_csv(csv_prefix + "_rrtstar.csv", rrt_stats.path);
    std::cout << "\nCSV written with prefix: " << csv_prefix << "\n";
  }

  if (!astar_stats.success || !rrt_stats.success) {
    std::cerr << "\nDemo incomplete: one or more planners failed.\n";
    return 1;
  }

  std::cout << "\nDemo OK — both planners found a path.\n";
  return 0;
}
