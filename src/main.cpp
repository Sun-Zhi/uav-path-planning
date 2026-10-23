#include "uav/astar3d.hpp"
#include "uav/environment.hpp"
#include "uav/rrt_star.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace {

void print_stats(const char* name, const uav::PlannerStats& s) {
  std::cout << "=== " << name << " ===\n";
  std::cout << "  success:        " << (s.success ? "yes" : "no") << "\n";
  std::cout << "  path_length_m:  " << s.path_length << "\n";
  std::cout << "  path_cost_risk: " << s.path_cost << "\n";
  std::cout << "  waypoints:      " << s.path_waypoints << "\n";
  std::cout << "  nodes/iters:    " << s.nodes_expanded << "\n";
  std::cout << "  time_ms:        " << s.time_ms << "\n";
}

bool write_csv(const std::string& path, const std::vector<uav::Vec3>& pts) {
  std::ofstream out(path);
  if (!out) return false;
  out << "x,y,z\n";
  for (const auto& p : pts) out << p.x << "," << p.y << "," << p.z << "\n";
  return true;
}

std::string default_map_path() {
  // Prefer configs next to build/, then source tree relative paths.
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
            << "  resolution: " << cfg.resolution << " m\n";
  std::cout << "Start: " << cfg.start << "  Goal: " << cfg.goal << "\n\n";

  uav::AStar3D astar(env);
  auto astar_stats = astar.plan(cfg.start, cfg.goal);
  print_stats("A* 3D (26-neighbor, risk-aware)", astar_stats);

  uav::RRTStarParams rp;
  rp.informed_sampling = informed;
  rp.max_iterations = 5000;
  rp.step_size = 3.0;
  rp.rewire_radius = 10.0;
  rp.goal_tolerance = 4.0;
  uav::RRTStar3D rrt(env, rp);
  auto rrt_stats = rrt.plan(cfg.start, cfg.goal);
  print_stats(informed ? "RRT* 3D (Informed after first solution)"
                       : "RRT* 3D (uniform sampling)",
              rrt_stats);

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
