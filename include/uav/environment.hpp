#pragma once
/**
 * 3D environment: hard AABB buildings + soft radar/EW risk zones + SNR→Pd radars.
 * Inspired by urban UAS scenes (Bertram et al., T-ITS 2023) and
 * electromagnetic / radar risk modeling (Atayev et al. 2024; Alpdemir 2022).
 *
 * Map JSON may be legacy meter ENU (x/y/z) or WGS84 ("coord_frame":"wgs84")
 * with lat/lon/alt; geo maps are converted to local ENU via geo.hpp.
 */
#include "uav/aabb.hpp"
#include "uav/geo.hpp"
#include "uav/radar.hpp"
#include "uav/vec3.hpp"
#include <algorithm>
#include <cstdint>
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
  std::vector<RadarSite> radars;
  double uav_rcs{0.1};
  double pd_cost_weight{25.0};
  double resolution{2.0};  // grid cell size for A*
  GeoOrigin origin{};
  bool geo_frame{false};
};

class Environment {
 public:
  explicit Environment(MapConfig cfg) : cfg_(std::move(cfg)) { build_building_index(); }

  const MapConfig& config() const { return cfg_; }
  const Bounds& bounds() const { return cfg_.bounds; }

  bool in_bounds(const Vec3& p) const {
    const auto& b = cfg_.bounds;
    return p.x >= b.min_pt.x && p.x <= b.max_pt.x && p.y >= b.min_pt.y &&
           p.y <= b.max_pt.y && p.z >= b.min_pt.z && p.z <= b.max_pt.z;
  }

  bool collides_hard(const Vec3& p) const {
    if (bins_.empty()) {
      for (const auto& b : cfg_.buildings) {
        if (b.contains(p)) return true;
      }
      return false;
    }
    const auto& ids = bins_[static_cast<size_t>(iy_of(p.y) * nx_ + ix_of(p.x))];
    for (int id : ids) {
      if (cfg_.buildings[static_cast<size_t>(id)].contains(p)) return true;
    }
    return false;
  }

  bool segment_hits_building(const Vec3& a, const Vec3& b) const {
    if (cfg_.buildings.empty()) return false;
    if (bins_.empty()) {
      for (const auto& box : cfg_.buildings) {
        if (box.intersects_segment(a, b)) return true;
      }
      return false;
    }
    const int x0 = ix_of(std::min(a.x, b.x));
    const int x1 = ix_of(std::max(a.x, b.x));
    const int y0 = iy_of(std::min(a.y, b.y));
    const int y1 = iy_of(std::max(a.y, b.y));
    if (++seen_clock_ == 0) {
      std::fill(seen_.begin(), seen_.end(), 0);
      seen_clock_ = 1;
    }
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        for (int id : bins_[static_cast<size_t>(y * nx_ + x)]) {
          if (seen_[static_cast<size_t>(id)] == seen_clock_) continue;
          seen_[static_cast<size_t>(id)] = seen_clock_;
          if (cfg_.buildings[static_cast<size_t>(id)].intersects_segment(a, b)) return true;
        }
      }
    }
    return false;
  }

  /** Multiplicative risk factor along segment (>=1). Soft AABB zones only. */
  double segment_risk_factor(const Vec3& a, const Vec3& b) const {
    double factor = 1.0;
    for (const auto& z : cfg_.risk_zones) {
      if (z.intersects_segment(a, b) || z.contains(a) || z.contains(b)) {
        factor = std::max(factor, z.risk_multiplier);
      }
    }
    return factor;
  }

  /** Edge cost = dist * AABB risk * (1 + w * max Pd on segment). */
  double edge_cost(const Vec3& a, const Vec3& b) const {
    const double pd =
        max_pd_on_segment(cfg_.radars, a, b, cfg_.uav_rcs, /*samples=*/8);
    return Vec3::dist(a, b) * segment_risk_factor(a, b) *
           (1.0 + cfg_.pd_cost_weight * pd);
  }

  bool is_free(const Vec3& p) const { return in_bounds(p) && !collides_hard(p); }

 private:
  MapConfig cfg_;
  // Uniform grid so OSM-sized building lists are not scanned per A* edge.
  double cell_{80.0};
  double origin_x_{0};
  double origin_y_{0};
  int nx_{0};
  int ny_{0};
  std::vector<std::vector<int>> bins_;
  mutable std::vector<std::uint32_t> seen_;
  mutable std::uint32_t seen_clock_{0};

  static int clamp_index(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  int ix_of(double x) const {
    return clamp_index(static_cast<int>(std::floor((x - origin_x_) / cell_)), 0, nx_ - 1);
  }
  int iy_of(double y) const {
    return clamp_index(static_cast<int>(std::floor((y - origin_y_) / cell_)), 0, ny_ - 1);
  }

  void build_building_index() {
    if (cfg_.buildings.empty()) return;
    double minx = cfg_.bounds.min_pt.x;
    double maxx = cfg_.bounds.max_pt.x;
    double miny = cfg_.bounds.min_pt.y;
    double maxy = cfg_.bounds.max_pt.y;
    for (const auto& b : cfg_.buildings) {
      minx = std::min(minx, b.min_pt.x);
      maxx = std::max(maxx, b.max_pt.x);
      miny = std::min(miny, b.min_pt.y);
      maxy = std::max(maxy, b.max_pt.y);
    }
    origin_x_ = minx;
    origin_y_ = miny;
    nx_ = std::max(1, static_cast<int>(std::floor((maxx - minx) / cell_)) + 1);
    ny_ = std::max(1, static_cast<int>(std::floor((maxy - miny) / cell_)) + 1);
    if (nx_ > 4000 || ny_ > 4000) {
      nx_ = ny_ = 0;
      return;
    }
    bins_.assign(static_cast<size_t>(nx_ * ny_), {});
    for (int i = 0; i < static_cast<int>(cfg_.buildings.size()); ++i) {
      const auto& b = cfg_.buildings[static_cast<size_t>(i)];
      const int x0 = ix_of(b.min_pt.x);
      const int x1 = ix_of(b.max_pt.x);
      const int y0 = iy_of(b.min_pt.y);
      const int y1 = iy_of(b.max_pt.y);
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          bins_[static_cast<size_t>(y * nx_ + x)].push_back(i);
        }
      }
    }
    seen_.assign(cfg_.buildings.size(), 0);
  }
};

/** Minimal JSON-ish loader for our sample_map format (no external deps). */
inline std::string extract_string(const std::string& s, const std::string& key) {
  const std::string pat = "\"" + key + "\"";
  auto pos = s.find(pat);
  if (pos == std::string::npos) return {};
  pos = s.find(':', pos);
  pos = s.find('"', pos);
  if (pos == std::string::npos) return {};
  auto end = s.find('"', pos + 1);
  if (end == std::string::npos) return {};
  return s.substr(pos + 1, end - pos - 1);
}

inline double extract_number_after(const std::string& s, size_t from,
                                   const std::string& key, double def = 0) {
  auto pos = s.find("\"" + key + "\"", from);
  if (pos == std::string::npos || pos > from + 800) return def;
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

inline GeoPoint parse_geo_block(const std::string& block) {
  return {extract_number_after(block, 0, "lat"),
          extract_number_after(block, 0, "lon"),
          extract_number_after(block, 0, "alt", 0.0)};
}

/** Find matching closing bracket for `[` or `{` at open_pos. */
inline size_t matching_bracket(const std::string& s, size_t open_pos) {
  if (open_pos >= s.size()) return std::string::npos;
  const char open = s[open_pos];
  const char close = (open == '[') ? ']' : '}';
  int depth = 0;
  for (size_t i = open_pos; i < s.size(); ++i) {
    if (s[i] == open) ++depth;
    else if (s[i] == close) {
      --depth;
      if (depth == 0) return i;
    }
  }
  return std::string::npos;
}

/** Object body after `"key"` through its matching `}`. */
inline std::string extract_object_after(const std::string& s, const std::string& key,
                                        size_t from = 0) {
  auto kpos = s.find("\"" + key + "\"", from);
  if (kpos == std::string::npos) return {};
  auto brace = s.find('{', kpos);
  if (brace == std::string::npos) return {};
  auto end = matching_bracket(s, brace);
  if (end == std::string::npos) return {};
  return s.substr(brace, end - brace + 1);
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

inline void parse_radar_fields(const std::string& obj, RadarSite& r) {
  r.id = extract_string(obj, "id");
  r.label = extract_string(obj, "label");
  r.band = extract_string(obj, "band");
  r.yaw_deg = extract_number_after(obj, 0, "yaw_deg", 0);
  r.pitch_deg = extract_number_after(obj, 0, "pitch_deg", 0);
  r.az_hpbw_deg = extract_number_after(obj, 0, "az_hpbw_deg", 20);
  r.el_hpbw_deg = extract_number_after(obj, 0, "el_hpbw_deg", 15);
  r.max_range_m = extract_number_after(obj, 0, "max_range_m", 50);
  r.snr_ref_linear = extract_number_after(obj, 0, "snr_ref_linear", 100);
  r.r_ref_m = extract_number_after(obj, 0, "r_ref_m", 25);
  r.sigma_ref_m2 = extract_number_after(obj, 0, "sigma_ref_m2", 0.1);
  r.snr_thresh_db = extract_number_after(obj, 0, "snr_thresh_db", 10);
  r.rcs_lo_factor = extract_number_after(obj, 0, "rcs_lo_factor", 1.0);
}

inline MapConfig load_map_json_string_legacy(const std::string& s) {
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
  cfg.uav_rcs = extract_number_after(s, 0, "uav_rcs", 0.1);
  cfg.pd_cost_weight = extract_number_after(s, 0, "pd_cost_weight", 25.0);
  cfg.geo_frame = false;

  auto parse_aabb_list = [&](const std::string& key, bool hard) {
    auto kpos = s.find("\"" + key + "\"");
    if (kpos == std::string::npos) return;
    auto abracket = s.find('[', kpos);
    auto ebracket = matching_bracket(s, abracket);
    if (ebracket == std::string::npos) return;
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

  auto rpos = s.find("\"radars\"");
  if (rpos != std::string::npos) {
    auto abracket = s.find('[', rpos);
    auto ebracket = matching_bracket(s, abracket);
    if (ebracket != std::string::npos) {
      for (const auto& obj :
           split_objects(s.substr(abracket, ebracket - abracket + 1))) {
        RadarSite r;
        parse_radar_fields(obj, r);
        auto ppos = obj.find("\"position\"");
        if (ppos != std::string::npos) {
          auto pend = obj.find('}', ppos);
          r.position = parse_vec3_block(obj.substr(ppos, pend - ppos + 1));
        }
        cfg.radars.push_back(std::move(r));
      }
    }
  }

  return cfg;
}

inline MapConfig load_map_json_string_geo(const std::string& s) {
  MapConfig cfg;
  cfg.geo_frame = true;
  cfg.resolution = extract_number_after(s, 0, "resolution", 15.0);
  cfg.uav_rcs = extract_number_after(s, 0, "uav_rcs", 0.1);
  cfg.pd_cost_weight = extract_number_after(s, 0, "pd_cost_weight", 25.0);

  GeoPoint start_g{}, goal_g{};
  auto start_obj = extract_object_after(s, "start");
  if (!start_obj.empty()) start_g = parse_geo_block(start_obj);
  auto goal_obj = extract_object_after(s, "goal");
  if (!goal_obj.empty()) goal_g = parse_geo_block(goal_obj);

  auto origin_obj = extract_object_after(s, "origin");
  if (!origin_obj.empty()) {
    GeoPoint og = parse_geo_block(origin_obj);
    cfg.origin = {og.lat, og.lon, og.alt};
  } else {
    cfg.origin = {start_g.lat, start_g.lon, 0.0};
  }

  cfg.start = enu_from_geo(cfg.origin, start_g);
  cfg.goal = enu_from_geo(cfg.origin, goal_g);

  auto bounds_pos = s.find("\"bounds\"");
  if (bounds_pos != std::string::npos) {
    auto min_obj = extract_object_after(s, "min", bounds_pos);
    auto max_obj = extract_object_after(s, "max", bounds_pos);
    if (!min_obj.empty())
      cfg.bounds.min_pt = enu_from_geo(cfg.origin, parse_geo_block(min_obj));
    if (!max_obj.empty())
      cfg.bounds.max_pt = enu_from_geo(cfg.origin, parse_geo_block(max_obj));
  }

  auto bpos = s.find("\"buildings\"");
  if (bpos != std::string::npos) {
    auto abracket = s.find('[', bpos);
    auto ebracket = matching_bracket(s, abracket);
    if (ebracket != std::string::npos) {
      for (const auto& obj :
           split_objects(s.substr(abracket, ebracket - abracket + 1))) {
        const double height_m = extract_number_after(obj, 0, "height_m", 15.0);
        std::vector<GeoPoint> ring;
        auto fpos = obj.find("\"footprint\"");
        if (fpos != std::string::npos) {
          auto fa = obj.find('[', fpos);
          auto fe = matching_bracket(obj, fa);
          if (fe != std::string::npos) {
            for (const auto& pt :
                 split_objects(obj.substr(fa, fe - fa + 1))) {
              ring.push_back(parse_geo_block(pt));
            }
          }
        }
        // Skip empty/missing footprints: avoid degenerate obstacles at origin.
        if (ring.empty()) continue;
        AABB box = aabb_from_footprint(cfg.origin, ring, height_m);
        box.name = extract_string(obj, "name");
        box.hard_obstacle = true;
        box.risk_multiplier = 1.0;
        cfg.buildings.push_back(std::move(box));
      }
    }
  }

  // Optional soft risk zones still as geo AABB via footprint+height, or skip.
  auto rzpos = s.find("\"risk_zones\"");
  if (rzpos != std::string::npos) {
    auto abracket = s.find('[', rzpos);
    auto ebracket = matching_bracket(s, abracket);
    if (ebracket != std::string::npos) {
      for (const auto& obj :
           split_objects(s.substr(abracket, ebracket - abracket + 1))) {
        // Prefer footprint; fall back to geo min/max if present.
        auto fpos = obj.find("\"footprint\"");
        if (fpos != std::string::npos) {
          const double height_m = extract_number_after(obj, 0, "height_m", 30.0);
          std::vector<GeoPoint> ring;
          auto fa = obj.find('[', fpos);
          auto fe = matching_bracket(obj, fa);
          if (fe != std::string::npos) {
            for (const auto& pt :
                 split_objects(obj.substr(fa, fe - fa + 1))) {
              ring.push_back(parse_geo_block(pt));
            }
          }
          AABB box = aabb_from_footprint(cfg.origin, ring, height_m);
          box.name = extract_string(obj, "name");
          box.hard_obstacle = false;
          box.risk_multiplier =
              extract_number_after(obj, 0, "risk_multiplier", 5.0);
          cfg.risk_zones.push_back(std::move(box));
        } else {
          auto min_obj = extract_object_after(obj, "min");
          auto max_obj = extract_object_after(obj, "max");
          if (!min_obj.empty() && !max_obj.empty()) {
            AABB box;
            box.min_pt = enu_from_geo(cfg.origin, parse_geo_block(min_obj));
            box.max_pt = enu_from_geo(cfg.origin, parse_geo_block(max_obj));
            box.name = extract_string(obj, "name");
            box.hard_obstacle = false;
            box.risk_multiplier =
                extract_number_after(obj, 0, "risk_multiplier", 5.0);
            cfg.risk_zones.push_back(std::move(box));
          }
        }
      }
    }
  }

  auto rpos = s.find("\"radars\"");
  if (rpos != std::string::npos) {
    auto abracket = s.find('[', rpos);
    auto ebracket = matching_bracket(s, abracket);
    if (ebracket != std::string::npos) {
      for (const auto& obj :
           split_objects(s.substr(abracket, ebracket - abracket + 1))) {
        RadarSite r;
        parse_radar_fields(obj, r);
        auto pos_obj = extract_object_after(obj, "position");
        if (!pos_obj.empty()) {
          r.position = enu_from_geo(cfg.origin, parse_geo_block(pos_obj));
        }
        cfg.radars.push_back(std::move(r));
      }
    }
  }

  return cfg;
}

inline MapConfig load_map_json_string(const std::string& s) {
  const std::string frame = extract_string(s, "coord_frame");
  if (frame == "wgs84") return load_map_json_string_geo(s);
  return load_map_json_string_legacy(s);
}

inline MapConfig load_map_json(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open map: " + path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return load_map_json_string(buffer.str());
}

}  // namespace uav
