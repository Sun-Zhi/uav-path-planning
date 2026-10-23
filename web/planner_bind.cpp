#include <emscripten/bind.h>
#include "uav/astar3d.hpp"
#include "uav/rrt_star.hpp"
#include <cmath>
#include <sstream>
#include <string>

namespace {

std::string serialize_result(const uav::PlannerStats& st) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(4);
  out << "{"
      << "\"success\":" << (st.success ? "true" : "false") << ","
      << "\"path_length\":" << st.path_length << ","
      << "\"path_cost\":" << st.path_cost << ","
      << "\"waypoints\":" << st.path_waypoints << ","
      << "\"nodes\":" << st.nodes_expanded << ","
      << "\"time_ms\":" << st.time_ms << ","
      << "\"path\":[";
  for (size_t i = 0; i < st.path.size(); ++i) {
    const auto& p = st.path[i];
    if (i) out << ",";
    out << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z << "}";
  }
  out << "]}";
  return out.str();
}

std::string plan(std::string map_json, std::string algo, bool informed) {
  try {
    auto cfg = uav::load_map_json_string(map_json);
    uav::Environment env(cfg);
    uav::PlannerStats st;
    if (algo == "rrtstar") {
      uav::RRTStarParams params;
      params.informed_sampling = informed;
      params.max_iterations = 5000;
      params.seed = 42;
      uav::RRTStar3D rrt(env, params);
      st = rrt.plan(cfg.start, cfg.goal);
    } else {
      uav::AStar3D astar(env);
      st = astar.plan(cfg.start, cfg.goal);
    }
    return serialize_result(st);
  } catch (const std::exception& e) {
    std::ostringstream out;
    out << "{\"success\":false,\"path_length\":0,\"path_cost\":0,"
        << "\"waypoints\":0,\"nodes\":0,\"time_ms\":0,\"path\":[],"
        << "\"error\":\"";
    for (char c : std::string(e.what())) {
      if (c == '"' || c == '\\') out << '\\';
      out << c;
    }
    out << "\"}";
    return out.str();
  }
}

}  // namespace

EMSCRIPTEN_BINDINGS(uav_planner) {
  emscripten::function("plan", &plan);
}
