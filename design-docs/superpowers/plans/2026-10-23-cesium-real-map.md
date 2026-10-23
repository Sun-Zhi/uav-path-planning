# Cesium Real-World Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a CesiumJS demo where start/goal are WGS84 lat/lon/alt, buildings come from OSM Overpass, ground is satellite imagery (default Beijing Guomao), and the existing A*/RRT*+radar WASM planner accepts/returns geographic coordinates via an internal ENU frame.

**Architecture:** Add `geo.hpp` (WGS84 ↔ ENU). Extend JSON map schema to geo (`lat`/`lon`/`alt`, building footprints). C++ loader builds ENU `MapConfig` for planners; WASM serializes path as lat/lon/alt. Replace Three.js Pages UI with CesiumJS + Overpass fetch; keep Plan / ▶模拟飞行 UX.

**Tech Stack:** C++17, Emscripten WASM, CesiumJS (CDN), OpenStreetMap Overpass, GitHub Pages (`docs/`).

## Global Constraints

- Spec: `design-docs/superpowers/specs/2026-10-23-cesium-real-map-design.md`
- No markdown under `docs/` (Pages/Jekyll); plans/specs stay in `design-docs/`
- Building cap ~800; bbox = start/goal + 1.5 km pad
- Heights: OSM `height` else `building:levels×3` else 15 m
- Radar ranges stay lab-scaled teaching profiles
- Auto-load Guomao once on first open; **加载地图** reloads after edits
- No paid Mapbox/Cesium ion requirement for v1

## File map

| File | Responsibility |
|------|----------------|
| `include/uav/geo.hpp` | WGS84 ↔ ENU, footprint → ENU AABB |
| `include/uav/environment.hpp` | Geo JSON loader → ENU MapConfig |
| `web/planner_bind.cpp` | plan() I/O lat/lon/alt path |
| `tests/test_geo.cpp` | Native geo conversion tests |
| `CMakeLists.txt` | `uav_test_geo` + ctest |
| `configs/archive/sample_map_meters.json` | Old meter sample |
| `configs/sample_map.json` | Guomao geo sample |
| `docs/sample_map.json` | Copy for Pages |
| `docs/index.html` | Cesium shell + lat/lon UI |
| `docs/app.js` | Cesium scene, Overpass, WASM wire-up |
| `docs/planner.js` / `planner.wasm` | Rebuild after bind change |
| `README.md` | Geo demo notes + attributions |

---

### Task 1: WGS84 ↔ ENU (`geo.hpp`) + tests

**Files:**
- Create: `include/uav/geo.hpp`
- Create: `tests/test_geo.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `struct GeoPoint { double lat, lon, alt; };` `struct GeoOrigin { double lat, lon, alt; };` `Vec3 enu_from_geo(const GeoOrigin&, const GeoPoint&)`; `GeoPoint geo_from_enu(const GeoOrigin&, const Vec3&)`; `AABB aabb_from_footprint(const GeoOrigin&, const std::vector<GeoPoint>& ring, double height_m)`; `double meters_per_deg_lat()`; `double meters_per_deg_lon(double lat_deg)`

- [ ] **Step 1: Write failing test**

```cpp
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
```

- [ ] **Step 2: Add CMake target and run (expect fail / missing header)**

```cmake
add_executable(uav_test_geo tests/test_geo.cpp)
target_include_directories(uav_test_geo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_test(NAME geo COMMAND uav_test_geo)
```

Run: `cmake -S . -B build && cmake --build build --target uav_test_geo && ./build/uav_test_geo`  
Expected: compile error — `geo.hpp` missing.

- [ ] **Step 3: Implement `include/uav/geo.hpp`**

```cpp
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
```

(If `AABB` uses different field names, match `include/uav/aabb.hpp` exactly.)

- [ ] **Step 4: Run tests**

Run: `cmake --build build --target uav_test_geo && ctest --test-dir build -R geo --output-on-failure`  
Expected: PASS / `test_geo OK`

- [ ] **Step 5: Commit**

```bash
git add include/uav/geo.hpp tests/test_geo.cpp CMakeLists.txt
git commit -m "feat(geo): WGS84↔ENU helpers and unit test"
```

---

### Task 2: Geo JSON schema + loader → ENU MapConfig

**Files:**
- Modify: `include/uav/environment.hpp` (`load_map_json_string`)
- Create: `configs/archive/sample_map_meters.json` (copy of current meter map)
- Modify: `configs/sample_map.json` (Guomao geo)
- Test: extend `tests/test_geo.cpp` or add `tests/test_geo_map.cpp`

**Interfaces:**
- Consumes: `geo.hpp`
- Produces: If JSON has `"coord_frame":"wgs84"`, parse geo start/goal/buildings/radars; set `GeoOrigin` from start (or explicit `"origin"`); fill ENU `MapConfig` as today. Path planning unchanged.

**Geo JSON shape (v1):**

```json
{
  "coord_frame": "wgs84",
  "origin": {"lat": 39.908, "lon": 116.460, "alt": 0},
  "resolution": 15.0,
  "uav_rcs": 0.1,
  "pd_cost_weight": 25.0,
  "bounds": {
    "min": {"lat": 39.900, "lon": 116.445, "alt": 0},
    "max": {"lat": 39.915, "lon": 116.470, "alt": 200}
  },
  "start": {"lat": 39.908, "lon": 116.460, "alt": 90},
  "goal":  {"lat": 39.912, "lon": 116.455, "alt": 110},
  "buildings": [
    {
      "name": "sample",
      "height_m": 80,
      "footprint": [
        {"lat": 39.9090, "lon": 116.4580},
        {"lat": 39.9090, "lon": 116.4590},
        {"lat": 39.9098, "lon": 116.4590},
        {"lat": 39.9098, "lon": 116.4580}
      ]
    }
  ],
  "radars": [
    {
      "id": "us_spy6_class",
      "label": "US SPY-6–class (teaching, scaled)",
      "band": "S",
      "position": {"lat": 39.910, "lon": 116.462, "alt": 5},
      "yaw_deg": 225,
      "pitch_deg": 8,
      "az_hpbw_deg": 12,
      "el_hpbw_deg": 10,
      "max_range_m": 1200,
      "snr_ref_linear": 200,
      "r_ref_m": 400,
      "sigma_ref_m2": 0.1,
      "snr_thresh_db": 10,
      "rcs_lo_factor": 1.0
    }
  ]
}
```

Scale radar `max_range_m` / `r_ref_m` to the ~few-km Guomao bbox (lab-scaled, not real). Keep two profiles (US+CN). Drop or empty `risk_zones` for geo sample unless easy.

- [ ] **Step 1: Archive meter sample**

```bash
mkdir -p configs/archive
cp configs/sample_map.json configs/archive/sample_map_meters.json
```

- [ ] **Step 2: Extend loader**

In `load_map_json_string`:
1. Detect `"coord_frame"` == `wgs84` (else keep legacy x/y/z path for archive file).
2. Parse `origin`, `start`, `goal`, `bounds` as lat/lon/alt → ENU via `enu_from_geo`.
3. For each building: parse `footprint[]` + `height_m` → `aabb_from_footprint`.
4. For each radar: parse `position` lat/lon/alt → ENU `RadarSite.position`; other fields unchanged.
5. Set `resolution` default ~15 m for larger maps (A* grid must stay tractable — if bounds ~3 km, resolution 15–25 m).

Helper parsers: `parse_geo_block` reading `"lat"`,`"lon"`,`"alt"` (alt default 0).

- [ ] **Step 3: Write Guomao `configs/sample_map.json`** with at least 2–4 hand-authored footprints near Guomao so `--demo` works offline without Overpass.

- [ ] **Step 4: Run native demo**

```bash
cmake --build build && ./build/uav_planner --demo
```

Expected: both A* and RRT* succeed on geo sample.

- [ ] **Step 5: Commit**

```bash
git add include/uav/environment.hpp configs/
git commit -m "feat(map): WGS84 JSON loader with Guomao sample"
```

---

### Task 3: WASM bind — path as lat/lon/alt

**Files:**
- Modify: `web/planner_bind.cpp`
- Modify: `web/build.sh` if needed
- Rebuild: `docs/planner.js`, `docs/planner.wasm`

**Interfaces:**
- Consumes: geo-aware `load_map_json_string`
- Produces: `plan(map_json, algo, informed)` returns path points `{"lat","lon","alt"}` plus existing stats; include `"origin"` echo for debugging

- [ ] **Step 1: Change serializer**

After planning in ENU, convert each `st.path[i]` with `geo_from_enu(origin, p)` where `origin` was parsed during load. Store origin on `MapConfig` (add `GeoOrigin origin{}` and `bool geo_frame{false}` fields).

```cpp
// serialize path element when cfg.geo_frame:
out << "{\"lat\":" << g.lat << ",\"lon\":" << g.lon << ",\"alt\":" << g.alt << "}";
// else legacy x,y,z
```

- [ ] **Step 2: Rebuild WASM**

```bash
bash web/build.sh
# copies planner.js/wasm into docs/
```

- [ ] **Step 3: Smoke with node or quick HTML note** — at minimum ensure `planner.js` size nonzero and `plan` export exists.

- [ ] **Step 4: Commit**

```bash
git add web/planner_bind.cpp include/uav/environment.hpp docs/planner.js docs/planner.wasm
git commit -m "feat(wasm): return WGS84 path waypoints"
```

---

### Task 4: Cesium page shell + lat/lon UI

**Files:**
- Modify: `docs/index.html` (replace Three.js imports with Cesium CDN; lat/lon inputs; 加载地图 / Plan / Fly)
- Rewrite: `docs/app.js` (Cesium Viewer; camera to Guomao; draw start/goal; no OSM yet — use `sample_map.json`)

**Interfaces:**
- Produces: `loadSampleMap()` fetches `sample_map.json`, builds Cesium entities for buildings (boxes from footprint), calls WASM `plan`, draws polyline in cartographic degrees; Fly samples lat/lon/alt track.

- [ ] **Step 1: HTML**

- Cesium CSS/JS from official CDN (pinned version, e.g. 1.122 or current stable).
- `#cesiumContainer` full-bleed in `#view`.
- Inputs: `slat slon salt` / `glat glon galt` (number, step any).
- Buttons: `loadMapBtn` 加载地图, `planBtn`, `flyBtn`, `resetBtn`.
- Keep algo select + radar checkboxes + stats + disclaimer footer (OSM ©, teaching radar).

- [ ] **Step 2: Minimal `app.js`**

```javascript
// Pseudocode structure — implement fully:
const viewer = new Cesium.Viewer('cesiumContainer', {
  animation: false, timeline: false, baseLayerPicker: true,
  geocoder: false, homeButton: false, sceneModePicker: false,
});
// Prefer satellite-ish imagery if available without token; else default + OSM fallback note in footer.

async function loadSampleAndShow() {
  const map = await (await fetch('sample_map.json')).json();
  fillLatLon(map);
  rebuildCesiumBuildings(map);
  flyCameraTo(map);
}

function rebuildCesiumBuildings(map) {
  viewer.entities.removeAll();
  for (const b of map.buildings || []) {
    const hierarchy = b.footprint.map(p => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
    viewer.entities.add({
      polygon: {
        hierarchy,
        extrudedHeight: b.height_m || 15,
        material: Cesium.Color.GRAY.withAlpha(0.75),
        outline: true,
      },
    });
  }
  // start/goal point entities
}
```

Wire Plan → `Module.plan(JSON.stringify(map), algo, informed)` → parse path → `PolylineGraphics` with `Cesium.Cartesian3.fromDegreesArrayHeights`.

Fly: reuse prior icon-along-path logic but update a billboard position each frame from lat/lon/alt samples.

- [ ] **Step 3: Manual check locally**

```bash
cd docs && python3 -m http.server 8765
# open http://127.0.0.1:8765 — Guomao sample buildings + Plan works
```

- [ ] **Step 4: Commit**

```bash
git add docs/index.html docs/app.js docs/sample_map.json
git commit -m "feat(demo): Cesium shell with geo sample map"
```

---

### Task 5: Overpass OSM fetch + 加载地图

**Files:**
- Modify: `docs/app.js` — `fetchOsmBuildings(bbox)`, height parse, cap 800, merge into map JSON for WASM + Cesium

**Interfaces:**
- Produces: `async function fetchOsmBuildings(minLat, minLon, maxLat, maxLon): Building[]`  
  Building = `{ name, height_m, footprint: [{lat,lon}, ...] }`

- [ ] **Step 1: Bbox helper**

```javascript
function padBbox(start, goal, padM = 1500) {
  const midLat = (start.lat + goal.lat) / 2;
  const mPerDegLat = 111320;
  const mPerDegLon = 111320 * Math.cos(midLat * Math.PI / 180);
  const dLat = padM / mPerDegLat;
  const dLon = padM / mPerDegLon;
  return {
    minLat: Math.min(start.lat, goal.lat) - dLat,
    maxLat: Math.max(start.lat, goal.lat) + dLat,
    minLon: Math.min(start.lon, goal.lon) - dLon,
    maxLon: Math.max(start.lon, goal.lon) + dLon,
  };
}
```

- [ ] **Step 2: Overpass query**

```javascript
const query = `[out:json][timeout:25];
(
  way["building"](${minLat},${minLon},${maxLat},${maxLon});
  relation["building"](${minLat},${minLon},${maxLat},${maxLon});
);
out body; >; out skel qt;`;
// POST to https://overpass-api.de/api/interpreter
// on fail retry https://overpass.kumi.systems/api/interpreter
```

Parse ways → closed rings; `height` / `building:levels` → `height_m` via same rules as C++; sort by footprint area desc; slice(0, 800).

- [ ] **Step 3: 加载地图**

On click / first page load: read lat/lon inputs → padBbox → fetchOsm → set `baseMap.buildings` → set bounds from bbox → place two teaching radars in geo near corridor → `rebuildCesiumBuildings` + update `docs`-side map for Plan.

On Overpass error: `setStats(error, 'err')`, do not clear last good buildings.

- [ ] **Step 4: Manual test** — Guomao shows dense real footprints; change goal slightly, 加载地图 refreshes.

- [ ] **Step 5: Commit**

```bash
git add docs/app.js
git commit -m "feat(demo): Overpass OSM buildings for any lat/lon"
```

---

### Task 6: Radar entities, polish, README, push Pages

**Files:**
- Modify: `docs/app.js` — radar billboards / small cones at geo sites; respect CN/US toggles when building plan JSON
- Modify: `README.md` — how to use geo demo, attributions (OSM, imagery, Cesium), radar disclaimer
- Copy `configs/sample_map.json` → `docs/sample_map.json` if drifted
- Push `master` (Pages from `/docs`)

- [ ] **Step 1: Radar toggle** — when building `currentMapJson()`, filter `radars` by checkbox; positions stay geo.

- [ ] **Step 2: README section** “Browser demo (Cesium + OSM)” with Guomao defaults and “任意经纬度 → 加载地图 → Plan → 模拟飞行”.

- [ ] **Step 3: Verify no `.md` under `docs/`**

```bash
find docs -name '*.md'
# expected: empty
```

- [ ] **Step 4: End-to-end**

1. Open Pages URL (after push).  
2. Guomao auto-load shows satellite + buildings.  
3. Plan succeeds; Fly runs.  
4. Optional: Tokyo short hop lat/lon → 加载地图 → Plan.

- [ ] **Step 5: Commit + push**

```bash
git add docs/ README.md configs/docs as needed
git commit -m "feat(demo): Cesium real-map polish and docs"
git push origin master
```

---

## Spec coverage check

| Spec item | Task |
|-----------|------|
| Cesium + satellite | 4 |
| OSM Overpass on demand | 5 |
| height/levels/15 m | 5 (+ geo.hpp estimate) |
| Guomao default + any lat/lon | 4–5 |
| C++ lat/lon API, internal ENU | 1–3 |
| 1.5 km pad, ~800 cap | 5 |
| Auto once + 加载地图 | 5 |
| Lab-scaled geo radars | 2, 6 |
| Plan / Fly UX | 4 |
| Pages, no md in docs/ | 6 |
| Archive meter sample | 2 |

## Placeholder / consistency review

- ENU convention locked: **x=East, y=North, z=Up** (matches prior map x/y ground, z up).
- Path JSON keys after Task 3: **`lat`/`lon`/`alt` only** in geo mode (no mixed x/y/z).
- A* `resolution` for ~3 km maps: **15–25 m** (Task 2).
