#pragma once
#include "uav/aabb.hpp"
#include "uav/vec3.hpp"
#include <cmath>
#include <vector>

namespace uav {

struct GeoPoint {
  double lat{0};  // degrees
  double lon{0};
  double alt{0};  // meters above origin ellipsoid/local up
};

struct GeoOrigin {
  double lat{0};
  double lon{0};
  double alt{0};
};

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kWgs84A = 6378137.0;
inline constexpr double kWgs84F = 1.0 / 298.257223563;
inline constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

inline double meters_per_deg_lat() {
  return deg2rad(1.0) * kWgs84A * (1.0 - kWgs84E2);  // approx; fine for <few km
}

inline double meters_per_deg_lon(double lat_deg) {
  const double s = std::sin(deg2rad(lat_deg));
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
  return deg2rad(1.0) * n * std::cos(deg2rad(lat_deg));
}

/** Local ENU: x=East, y=North, z=Up (meters), relative to origin. */
inline Vec3 enu_from_geo(const GeoOrigin& o, const GeoPoint& g) {
  const double mlat = meters_per_deg_lat();
  const double mlon = meters_per_deg_lon(o.lat);
  return {(g.lon - o.lon) * mlon, (g.lat - o.lat) * mlat, g.alt - o.alt};
}

inline GeoPoint geo_from_enu(const GeoOrigin& o, const Vec3& e) {
  const double mlat = meters_per_deg_lat();
  const double mlon = meters_per_deg_lon(o.lat);
  return {o.lat + e.y / mlat, o.lon + e.x / mlon, o.alt + e.z};
}

inline AABB aabb_from_footprint(const GeoOrigin& o,
                                const std::vector<GeoPoint>& ring,
                                double height_m) {
  AABB box;
  if (ring.empty()) {
    box.min_pt = {0, 0, 0};
    box.max_pt = {0, 0, height_m};
    return box;
  }
  Vec3 p0 = enu_from_geo(o, ring[0]);
  box.min_pt = box.max_pt = p0;
  for (const auto& g : ring) {
    Vec3 p = enu_from_geo(o, g);
    box.min_pt.x = std::min(box.min_pt.x, p.x);
    box.min_pt.y = std::min(box.min_pt.y, p.y);
    box.min_pt.z = std::min(box.min_pt.z, 0.0);
    box.max_pt.x = std::max(box.max_pt.x, p.x);
    box.max_pt.y = std::max(box.max_pt.y, p.y);
    box.max_pt.z = std::max(box.max_pt.z, height_m);
  }
  return box;
}

inline double estimate_height_m(double height_tag, double levels_tag) {
  if (height_tag > 0) return height_tag;
  if (levels_tag > 0) return levels_tag * 3.0;
  return 15.0;
}

}  // namespace uav
