// tests/test_geo.cpp
#include "uav/geo.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  using namespace uav;
  GeoOrigin o{39.908, 116.460, 0};  // Guomao-ish
  GeoPoint p = geo_from_enu(o, {100, 0, 0});  // +100 m East
  Vec3 back = enu_from_geo(o, p);
  assert(std::fabs(back.x - 100) < 0.5);
  assert(std::fabs(back.y) < 0.5);
  assert(std::fabs(back.z) < 0.5);

  // ~111.3 km per deg lat
  GeoPoint north{39.908 + 0.001, 116.460, 0};
  Vec3 n = enu_from_geo(o, north);
  assert(std::fabs(n.y - 111.3) < 2.0);  // ~111 m North
  assert(std::fabs(n.x) < 2.0);

  std::vector<GeoPoint> ring = {
      {39.9080, 116.4600, 0},
      {39.9080, 116.4605, 0},
      {39.9084, 116.4605, 0},
      {39.9084, 116.4600, 0},
  };
  AABB box = aabb_from_footprint(o, ring, 50);
  assert(box.max_pt.z >= 49.9 && box.min_pt.z <= 0.1);
  assert(box.max_pt.x > box.min_pt.x && box.max_pt.y > box.min_pt.y);
  std::cout << "test_geo OK\n";
  return 0;
}
