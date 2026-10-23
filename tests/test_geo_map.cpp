// tests/test_geo_map.cpp
#include "uav/environment.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

int main() {
  using namespace uav;

  const std::string geo_json = R"({
    "coord_frame": "wgs84",
    "origin": {"lat": 39.908, "lon": 116.460, "alt": 0},
    "resolution": 20.0,
    "uav_rcs": 0.1,
    "pd_cost_weight": 25.0,
    "bounds": {
      "min": {"lat": 39.905, "lon": 116.455, "alt": 0},
      "max": {"lat": 39.912, "lon": 116.465, "alt": 150}
    },
    "start": {"lat": 39.908, "lon": 116.460, "alt": 90},
    "goal":  {"lat": 39.910, "lon": 116.458, "alt": 100},
    "buildings": [
      {
        "name": "box",
        "height_m": 80,
        "footprint": [
          {"lat": 39.9085, "lon": 116.4585},
          {"lat": 39.9085, "lon": 116.4595},
          {"lat": 39.9092, "lon": 116.4595},
          {"lat": 39.9092, "lon": 116.4585}
        ]
      }
    ],
    "radars": [
      {
        "id": "r1",
        "label": "test",
        "band": "S",
        "position": {"lat": 39.907, "lon": 116.461, "alt": 5},
        "yaw_deg": 90,
        "pitch_deg": 5,
        "az_hpbw_deg": 20,
        "el_hpbw_deg": 15,
        "max_range_m": 800,
        "snr_ref_linear": 100,
        "r_ref_m": 300,
        "sigma_ref_m2": 0.1,
        "snr_thresh_db": 10,
        "rcs_lo_factor": 1.0
      }
    ]
  })";

  MapConfig cfg = load_map_json_string(geo_json);
  assert(cfg.geo_frame);
  assert(std::fabs(cfg.origin.lat - 39.908) < 1e-9);
  assert(std::fabs(cfg.origin.lon - 116.460) < 1e-9);
  assert(std::fabs(cfg.start.x) < 1.0);
  assert(std::fabs(cfg.start.y) < 1.0);
  assert(std::fabs(cfg.start.z - 90.0) < 1e-6);
  assert(cfg.goal.x < 0);   // west of origin
  assert(cfg.goal.y > 0);   // north of origin
  assert(cfg.buildings.size() == 1);
  assert(cfg.buildings[0].name == "box");
  assert(cfg.buildings[0].max_pt.z >= 79.9);
  assert(cfg.buildings[0].max_pt.x > cfg.buildings[0].min_pt.x);
  assert(cfg.radars.size() == 1);
  assert(cfg.radars[0].position.z == 5.0);
  assert(cfg.resolution == 20.0);
  assert(cfg.bounds.min_pt.x < cfg.bounds.max_pt.x);
  assert(cfg.bounds.min_pt.y < cfg.bounds.max_pt.y);

  // Legacy meter path still works
  const std::string meter_json = R"({
    "resolution": 2.0,
    "bounds": {"min": {"x": 0, "y": 0, "z": 0}, "max": {"x": 50, "y": 50, "z": 20}},
    "start": {"x": 5, "y": 5, "z": 10},
    "goal": {"x": 40, "y": 40, "z": 10},
    "buildings": [
      {"name": "t", "min": {"x": 20, "y": 20, "z": 0}, "max": {"x": 25, "y": 25, "z": 15}}
    ],
    "radars": []
  })";
  MapConfig legacy = load_map_json_string(meter_json);
  assert(!legacy.geo_frame);
  assert(legacy.start.x == 5 && legacy.goal.x == 40);
  assert(legacy.buildings.size() == 1);

  // File load Guomao sample if present
  try {
    MapConfig file_cfg = load_map_json("configs/sample_map.json");
    assert(file_cfg.geo_frame);
    assert(file_cfg.buildings.size() >= 2);
    assert(file_cfg.radars.size() == 2);
    Environment env(file_cfg);
    assert(env.is_free(file_cfg.start));
    assert(env.is_free(file_cfg.goal));
  } catch (const std::exception&) {
    // Allow running from build/ without configs nearby
    try {
      MapConfig file_cfg = load_map_json("../configs/sample_map.json");
      assert(file_cfg.geo_frame);
      assert(file_cfg.buildings.size() >= 2);
      Environment env(file_cfg);
      assert(env.is_free(file_cfg.start));
      assert(env.is_free(file_cfg.goal));
    } catch (...) {
      std::cerr << "warn: sample_map.json not found for file load check\n";
    }
  }

  std::cout << "test_geo_map OK\n";
  return 0;
}
