#pragma once
/**
 * Teaching radar SNR → Pd model (lab-scaled, not real system parameters).
 * Compact radar equation: SNR = K * σ / R^4 * G_beam, K from snr_ref at r_ref.
 */
#include "uav/vec3.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace uav {

struct RadarSite {
  std::string id;
  std::string label;
  std::string band;
  Vec3 position{0, 0, 0};
  double yaw_deg{0};    // 0 = +X, 90 = +Y
  double pitch_deg{0};  // 0 = horizontal, + = look up
  double az_hpbw_deg{20};
  double el_hpbw_deg{15};
  double max_range_m{50};
  double snr_ref_linear{100};
  double r_ref_m{25};
  double sigma_ref_m2{0.1};
  double snr_thresh_db{10};
  double rcs_lo_factor{1.0};  // >1 for VHF-style reduced stealth benefit (teaching)
};

inline Vec3 cross3(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/** Boresight unit vector from yaw/pitch (degrees). */
inline Vec3 boresight_dir(double yaw_deg, double pitch_deg) {
  const double yaw = yaw_deg * M_PI / 180.0;
  const double pitch = pitch_deg * M_PI / 180.0;
  const double cp = std::cos(pitch);
  return {cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
}

/**
 * Beam gain in [0,1]: product of Gaussian az/el falloffs (half-power at HPBW/2).
 * Outside max_range or in backlobe (forward component ≤ 0) → 0.
 */
inline double beam_gain(const RadarSite& r, const Vec3& p) {
  Vec3 rel = p - r.position;
  double R = rel.norm();
  if (R < 1e-3) R = 1e-3;
  if (R > r.max_range_m) return 0.0;

  const Vec3 dir = rel * (1.0 / R);
  const Vec3 fwd = boresight_dir(r.yaw_deg, r.pitch_deg);
  const double f = dir.dot(fwd);
  if (f <= 0.0) return 0.0;  // backlobe / behind

  // Local right / up (handle near-vertical boresight)
  Vec3 up_ref{0, 0, 1};
  if (std::fabs(fwd.dot(up_ref)) > 0.95) up_ref = {0, 1, 0};
  Vec3 right = cross3(fwd, up_ref);
  const double rn = right.norm();
  if (rn < 1e-12) return 0.0;
  right = right * (1.0 / rn);
  const Vec3 up = cross3(right, fwd);

  const double rr = dir.dot(right);
  const double uu = dir.dot(up);
  const double az_deg = std::atan2(rr, f) * 180.0 / M_PI;
  const double el_deg = std::atan2(uu, std::sqrt(f * f + rr * rr)) * 180.0 / M_PI;

  const double haz = std::max(r.az_hpbw_deg * 0.5, 1e-6);
  const double hel = std::max(r.el_hpbw_deg * 0.5, 1e-6);
  const double gaz = std::exp(-std::log(2.0) * (az_deg / haz) * (az_deg / haz));
  const double gel = std::exp(-std::log(2.0) * (el_deg / hel) * (el_deg / hel));
  return gaz * gel;
}

inline double snr_linear(const RadarSite& r, const Vec3& p, double sigma) {
  Vec3 rel = p - r.position;
  double R = rel.norm();
  if (R < 1e-3) R = 1e-3;
  if (R > r.max_range_m) return 0.0;

  const double G = beam_gain(r, p);
  if (G <= 0.0) return 0.0;

  const double sigma_ref = std::max(r.sigma_ref_m2, 1e-12);
  const double K = r.snr_ref_linear * std::pow(r.r_ref_m, 4.0) / sigma_ref;
  const double sigma_eff = sigma * std::max(r.rcs_lo_factor, 0.0);
  return K * sigma_eff / std::pow(R, 4.0) * G;
}

/** Swerling-1 style logistic gate on SNR_dB. */
inline double pd_from_snr(double snr_lin, double thresh_db, double steepness = 0.35) {
  if (snr_lin <= 1e-30) return 0.0;
  const double snr_db = 10.0 * std::log10(snr_lin);
  const double x = -steepness * (snr_db - thresh_db);
  // stable logistic
  if (x > 40.0) return 0.0;
  if (x < -40.0) return 1.0;
  return 1.0 / (1.0 + std::exp(x));
}

inline double pd_at_point(const RadarSite& r, const Vec3& p, double sigma) {
  return pd_from_snr(snr_linear(r, p, sigma), r.snr_thresh_db);
}

/** Max Pd over radars, sampled along segment (inclusive endpoints). */
inline double max_pd_on_segment(const std::vector<RadarSite>& radars, const Vec3& a,
                                const Vec3& b, double sigma, int samples = 8) {
  if (radars.empty()) return 0.0;
  const int n = std::max(samples, 2);
  double mx = 0.0;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const Vec3 p = a + (b - a) * t;
    for (const auto& r : radars) {
      mx = std::max(mx, pd_at_point(r, p, sigma));
    }
  }
  return mx;
}

/** Mean / max of (max over radars Pd) at each waypoint. */
inline void path_pd_stats(const std::vector<RadarSite>& radars, const std::vector<Vec3>& path,
                          double sigma, double& mean_pd, double& max_pd) {
  mean_pd = 0.0;
  max_pd = 0.0;
  if (path.empty()) return;
  if (radars.empty()) return;
  double sum = 0.0;
  for (const auto& p : path) {
    double pd = 0.0;
    for (const auto& r : radars) pd = std::max(pd, pd_at_point(r, p, sigma));
    sum += pd;
    max_pd = std::max(max_pd, pd);
  }
  mean_pd = sum / static_cast<double>(path.size());
}

}  // namespace uav
