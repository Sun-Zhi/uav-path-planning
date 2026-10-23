#pragma once
/**
 * 3D environment: hard AABB buildings + soft radar/EW risk zones.
 * Inspired by urban UAS scenes (Bertram et al., T-ITS 2023) and
 * electromagnetic / radar risk modeling (Atayev et al. 2024; Alpdemir 2022).
 */
#include "uav/aabb.hpp"
#include "uav/vec3.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace uav {

struct Bounds {
  Vec3 min_pt{0, 0, 0};
  Vec3 max_pt{100, 100, 40};
};

struct MapConfig {
  Bounds bounds;
  Vec3 start{5, 5, 10};
  Vec3 goal{95, 95, 10};
  std::vector<AABB> buildings;
  std::vector<AABB> risk_zones;
  double resolution{2.0};  // grid cell size for A*
};

class Environment {
 public:
  explicit Environment(MapConfig cfg) : cfg_(std::move(cfg)) {}

  const MapConfig& config() const { return cfg_; }
  const Bounds& bounds() const { return cfg_.bounds; }

  bool in_bounds(const Vec3& p) const {
    const auto& b = cfg_.bounds;
    return p.x >= b.min_pt.x && p.x <= b.max_pt.x && p.y >= b.min_pt.y &&
           p.y <= b.max_pt.y && p.z >= b.min_pt.z && p.z <= b.max_pt.z;
  }

  bool collides_hard(const Vec3& p) const {
    for (const auto& b : cfg_.buildings) {
      if (b.contains(p)) return true;
    }
    return false;
  }

  bool segment_hits_building(const Vec3& a, const Vec3& b) const {
    for (const auto& box : cfg_.buildings) {
      if (box.intersects_segment(a, b)) return true;
    }
    return false;
  }

  /** Multiplicative risk factor along segment (>=1). Soft zones only. */
  double segment_risk_factor(const Vec3& a, const Vec3& b) const {
    double factor = 1.0;
    for (const auto& z : cfg_.risk_zones) {
      if (z.intersects_segment(a, b) || z.contains(a) || z.contains(b)) {
        factor = std::max(factor, z.risk_multiplier);
      }
    }
    return factor;
  }

  /** Edge cost = Euclidean length * risk (risk-aware planning). */
  double edge_cost(const Vec3& a, const Vec3& b) const {
    return Vec3::dist(a, b) * segment_risk_factor(a, b);
  }

  bool is_free(const Vec3& p) const { return in_bounds(p) && !collides_hard(p); }

 private:
  MapConfig cfg_;
};

/** Minimal JSON-ish loader for our sample_map format (no external deps). */
inline std::string extract_string(const std::string& s, const std::string& key) {
  const std::string pat = "\"" + key + "\"";
  auto pos = s.find(pat);
  if (pos == std::string::npos) return {};
  pos = s.find(':', pos);
  pos = s.find('"', pos);
  auto end = s.find('"', pos + 1);
  return s.substr(pos + 1, end - pos - 1);
}

inline double extract_number_after(const std::string& s, size_t from,
                                   const std::string& key, double def = 0) {
  auto pos = s.find("\"" + key + "\"", from);
  if (pos == std::string::npos || pos > from + 500) return def;
  pos = s.find(':', pos);
  size_t i = pos + 1;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
  try {
    return std::stod(s.substr(i));
  } catch (...) {
    return def;
  }
}

inline Vec3 parse_vec3_block(const std::string& block) {
  return {extract_number_after(block, 0, "x"),
          extract_number_after(block, 0, "y"),
          extract_number_after(block, 0, "z")};
}

inline std::vector<std::string> split_objects(const std::string& arr) {
  std::vector<std::string> out;
  int depth = 0;
  size_t start = std::string::npos;
  for (size_t i = 0; i < arr.size(); ++i) {
    if (arr[i] == '{') {
      if (depth == 0) start = i;
      ++depth;
    } else if (arr[i] == '}') {
      --depth;
      if (depth == 0 && start != std::string::npos) {
        out.push_back(arr.substr(start, i - start + 1));
        start = std::string::npos;
      }
    }
  }
  return out;
}

inline MapConfig load_map_json_string(const std::string& s) {
  MapConfig cfg;
  auto bounds_pos = s.find("\"bounds\"");
  if (bounds_pos != std::string::npos) {
    auto min_pos = s.find("\"min\"", bounds_pos);
    auto max_pos = s.find("\"max\"", bounds_pos);
    auto min_end = s.find('}', min_pos);
    auto max_end = s.find('}', max_pos);
    cfg.bounds.min_pt = parse_vec3_block(s.substr(min_pos, min_end - min_pos + 1));
    cfg.bounds.max_pt = parse_vec3_block(s.substr(max_pos, max_end - max_pos + 1));
  }

  auto start_pos = s.find("\"start\"");
  if (start_pos != std::string::npos) {
    auto end = s.find('}', start_pos);
    cfg.start = parse_vec3_block(s.substr(start_pos, end - start_pos + 1));
  }
  auto goal_pos = s.find("\"goal\"");
  if (goal_pos != std::string::npos) {
    auto end = s.find('}', goal_pos);
    cfg.goal = parse_vec3_block(s.substr(goal_pos, end - goal_pos + 1));
  }

  cfg.resolution = extract_number_after(s, 0, "resolution", 2.0);

  auto parse_aabb_list = [&](const std::string& key, bool hard) {
    auto kpos = s.find("\"" + key + "\"");
    if (kpos == std::string::npos) return;
    auto abracket = s.find('[', kpos);
    auto ebracket = s.find(']', abracket);
    for (const auto& obj : split_objects(s.substr(abracket, ebracket - abracket + 1))) {
      AABB box;
      box.name = extract_string(obj, "name");
      auto min_p = obj.find("\"min\"");
      auto max_p = obj.find("\"max\"");
      auto min_e = obj.find('}', min_p);
      auto max_e = obj.find('}', max_p);
      box.min_pt = parse_vec3_block(obj.substr(min_p, min_e - min_p + 1));
      box.max_pt = parse_vec3_block(obj.substr(max_p, max_e - max_p + 1));
      box.hard_obstacle = hard;
      box.risk_multiplier =
          extract_number_after(obj, 0, "risk_multiplier", hard ? 1.0 : 5.0);
      if (hard)
        cfg.buildings.push_back(box);
      else
        cfg.risk_zones.push_back(box);
    }
  };

  parse_aabb_list("buildings", true);
  parse_aabb_list("risk_zones", false);
  return cfg;
}

inline MapConfig load_map_json(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open map: " + path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return load_map_json_string(buffer.str());
}

}  // namespace uav
