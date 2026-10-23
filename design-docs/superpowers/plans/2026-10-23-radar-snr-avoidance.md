# Radar SNR Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add SNR→Pd radar cost model with scaled US SPY-6–class and CN JY-27–class teaching profiles, wire into planners + WASM demo.

**Architecture:** New `radar.hpp` computes Pd; `Environment::edge_cost` multiplies by `(1+w*Pd)`; sample map hosts two radars; rebuild WASM; Three.js draws sectors and shows Pd stats.

**Tech Stack:** C++17 headers, existing A*/RRT*, Emscripten Embind, Three.js pages

## Global Constraints

- Teaching approximations only; every user-visible name includes “teaching” / “scaled” / “非实装”.
- Do not invent classified ERP numbers; use abstract `snr_ref` calibrated so Pd is high on boresight mid-range and low outside beam.
- Keep native `./build/uav_planner --demo` exit 0.
- Rebuild `docs/planner.{js,wasm}` via `./web/build.sh` and push Pages.
- Git author: Sun-Zhi / Sun-Zhi@users.noreply.github.com.

---

### Task 1: `radar.hpp` + Environment integration

**Files:**
- Create: `include/uav/radar.hpp`
- Modify: `include/uav/environment.hpp` (MapConfig fields, edge_cost, JSON parse)
- Modify: `configs/sample_map.json`

**Interfaces:**
- Produces:
  - `struct RadarSite { std::string id, label, band; Vec3 position; double yaw_deg, pitch_deg; double az_hpbw_deg, el_hpbw_deg; double max_range_m; double snr_ref_linear; double r_ref_m; double sigma_ref_m2; double snr_thresh_db; double rcs_lo_factor; };`
  - `double beam_gain(const RadarSite&, const Vec3& p);`
  - `double snr_linear(const RadarSite&, const Vec3& p, double sigma);`
  - `double pd_from_snr(double snr_linear, double thresh_db, double steepness=0.35);`
  - `double pd_at_point(const RadarSite&, const Vec3& p, double sigma);`
  - `double max_pd_on_segment(const std::vector<RadarSite>&, const Vec3& a, const Vec3& b, double sigma, int samples=8);`
- MapConfig adds: `std::vector<RadarSite> radars; double uav_rcs{0.1}; double pd_cost_weight{25.0};`
- `edge_cost`: `dist * segment_risk_factor(a,b) * (1.0 + cfg_.pd_cost_weight * max_pd_on_segment(...))`

**Sample radars (place so paths must choose):**
- `us_spy6_class` at ~(40,5,2), yaw toward map center (~90°), az_hpbw 12, el 10, max_range 55, snr_ref ~200, r_ref 25, sigma_ref 0.1, thresh 10 dB, rcs_lo_factor 1
- `cn_jy27_class` at ~(5,50,2), yaw ~0°, az 35, el 25, max_range 80, snr_ref ~80, r_ref 35, thresh 8 dB, rcs_lo_factor 4
- Keep existing buildings; can keep or thin old AABB risk_zones (prefer keep for backward compat).

- [ ] Implement headers + JSON parse keys matching sample_map
- [ ] Rebuild native; `--demo` succeeds
- [ ] Commit

### Task 2: CLI stats for Pd

**Files:**
- Modify: `src/main.cpp`

- [ ] After each plan, compute mean/max Pd along `stats.path` and print `mean_pd` / `max_pd`
- [ ] Commit

### Task 3: WASM + frontend

**Files:**
- Modify: `web/planner_bind.cpp` — add mean_pd, max_pd to JSON
- Run: `./web/build.sh`
- Modify: `docs/app.js`, `docs/index.html` — draw radar wedges (Three.js), legend, show Pd; copy updated `docs/sample_map.json`
- Modify: `README.md`, `PAPERS.md` — short section + disclaimer + open lit links

- [ ] Rebuild WASM artifacts into docs/
- [ ] UI smoke: map loads radars array
- [ ] Commit + push origin master
- [ ] Confirm Pages still `master`/`docs`

### Task 4: Verify

- [ ] Native demo OK
- [ ] Node or quick check that plan JSON includes mean_pd
- [ ] Report Pages URL to parent

## Spec coverage

- SNR/Pd model ✓ T1
- CN+US profiles ✓ T1 sample_map
- Cost in A*/RRT* ✓ T1 edge_cost
- Demo viz + stats ✓ T3
- Disclaimers ✓ T3 docs
