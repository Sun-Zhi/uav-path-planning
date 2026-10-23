#pragma once
#include <cmath>
#include <ostream>

namespace uav {

struct Vec3 {
  double x{0}, y{0}, z{0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3& operator+=(const Vec3& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  double norm2() const { return x * x + y * y + z * z; }
  double norm() const { return std::sqrt(norm2()); }

  Vec3 normalized() const {
    const double n = norm();
    if (n < 1e-12) return {0, 0, 0};
    return (*this) * (1.0 / n);
  }

  static double dist(const Vec3& a, const Vec3& b) { return (a - b).norm(); }
};

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
  return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

}  // namespace uav
