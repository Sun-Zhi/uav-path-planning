#pragma once
#include "uav/vec3.hpp"
#include <algorithm>
#include <string>

namespace uav {

/** Axis-aligned bounding box: building (hard) or risk zone (soft). */
struct AABB {
  Vec3 min_pt;
  Vec3 max_pt;
  std::string name;
  bool hard_obstacle{true};   // buildings: true; radar/EW soft zones: false
  double risk_multiplier{1.0};  // cost scale when path segment intersects soft zone

  bool contains(const Vec3& p) const {
    return p.x >= min_pt.x && p.x <= max_pt.x && p.y >= min_pt.y &&
           p.y <= max_pt.y && p.z >= min_pt.z && p.z <= max_pt.z;
  }

  /** Segment–AABB intersection (Liang–Barsky style slab test). */
  bool intersects_segment(const Vec3& a, const Vec3& b) const {
    Vec3 d = b - a;
    double t0 = 0.0, t1 = 1.0;
    const double dims[3] = {d.x, d.y, d.z};
    const double orig[3] = {a.x, a.y, a.z};
    const double bmin[3] = {min_pt.x, min_pt.y, min_pt.z};
    const double bmax[3] = {max_pt.x, max_pt.y, max_pt.z};

    for (int i = 0; i < 3; ++i) {
      if (std::abs(dims[i]) < 1e-12) {
        if (orig[i] < bmin[i] || orig[i] > bmax[i]) return false;
      } else {
        double inv = 1.0 / dims[i];
        double tNear = (bmin[i] - orig[i]) * inv;
        double tFar = (bmax[i] - orig[i]) * inv;
        if (tNear > tFar) std::swap(tNear, tFar);
        t0 = std::max(t0, tNear);
        t1 = std::min(t1, tFar);
        if (t0 > t1) return false;
      }
    }
    return true;
  }

  Vec3 center() const {
    return {(min_pt.x + max_pt.x) * 0.5, (min_pt.y + max_pt.y) * 0.5,
            (min_pt.z + max_pt.z) * 0.5};
  }
};

}  // namespace uav
