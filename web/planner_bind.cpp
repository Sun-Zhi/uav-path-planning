#include <emscripten/bind.h>
#include "uav/astar3d.hpp"
#include "uav/geo.hpp"
#include "uav/radar.hpp"
#include "uav/rrt_star.hpp"
#include <cmath>
#include <sstream>
#include <string>

namespace {

std::string serialize_result(const uav::MapConfig& cfg,
                             const uav::PlannerStats& st, double mean_pd,
                             double max_pd) {
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
      << "\"mean_pd\":" << mean_pd << ","
      << "\"max_pd\":" << max_pd << ",";
  if (cfg.geo_frame) {
    out << "\"origin\":{\"lat\":" << cfg.origin.lat << ",\"lon\":"
        << cfg.origin.lon << ",\"alt\":" << cfg.origin.alt << "},";
  }
  out << "\"path\":[";
  for (size_t i = 0; i < st.path.size(); ++i) {
    const auto& p = st.path[i];
    if (i) out << ",";
    if (cfg.geo_frame) {
      uav::GeoPoint g = uav::geo_from_enu(cfg.origin, p);
      out << "{\"lat\":" << g.lat << ",\"lon\":" << g.lon << ",\"alt\":" << g.alt
          << "}";
    } else {
      out << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z << "}";
    }
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
    double mean_pd = 0, max_pd = 0;
    uav::path_pd_stats(cfg.radars, st.path, cfg.uav_rcs, mean_pd, max_pd);
    return serialize_result(cfg, st, mean_pd, max_pd);
  } catch (const std::exception& e) {
    std::ostringstream out;
    out << "{\"success\":false,\"path_length\":0,\"path_cost\":0,"
        << "\"waypoints\":0,\"nodes\":0,\"time_ms\":0,"
        << "\"mean_pd\":0,\"max_pd\":0,\"path\":[],"
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
