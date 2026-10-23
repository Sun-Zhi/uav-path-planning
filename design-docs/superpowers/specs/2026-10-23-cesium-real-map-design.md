# Cesium Real-World Map (lat/lon) — Design

**Date:** 2026-10-23  
**Status:** Approved (CesiumJS + OSM on-demand + C++ lat/lon API)

## Goal

Replace the synthetic local-meter demo map with a **real Earth view**: start/goal and radars in **WGS84 lat/lon/alt**, buildings from **OpenStreetMap**, ground from **satellite imagery**, default scene **Beijing Guomao CBD**. Any other place works by changing coordinates. The planning core still searches in meters, but **C++ accepts and returns geographic coordinates** (internal ENU conversion).

## Decisions (locked)

| Topic | Choice |
|-------|--------|
| Interaction | Any lat/lon: enter start/goal → pull local buildings |
| Look | Extruded 3D buildings + satellite ground |
| Heights | OSM `height` / `building:levels`; else estimate (`levels×3 m`, default 15 m) |
| Default scene | Beijing Guomao CBD |
| Engine | **CesiumJS** (not MapLibre; not keep Three.js as primary globe) |
| Planner coords | **C++ lat/lon/alt API**; internal ENU around start (or bbox origin) |
| Fetch bbox | Start/goal padded by **~1.5 km** |
| Building cap | **~800** footprints (prefer larger area if over cap) |
| Map load | Auto once on first open; **加载地图** to reload after edits (not on every keystroke) |

## Architecture

```
[UI: lat/lon/alt] → [Overpass OSM buildings] → GeoJSON footprints + heights
                 ↘
[CesiumJS globe + satellite imagery] ← draw buildings, path, radars, drone
                 ↗
[WASM planner]  ← JSON (geo) → internal ENU → A*/RRT* + radar Pd → JSON path (geo)
```

1. **Frontend (CesiumJS)**  
   - Replace Three.js scene as the primary map (Three.js may be removed from the demo page).  
   - Imagery: Cesium default / Ion-free public satellite or OSM raster fallback (no paid Mapbox required).  
   - Buildings: extrude OSM polygons as Cesium entities or a single primitive; translucent gray.  
   - Path: `Polyline` / `CallbackProperty` in cartographic degrees.  
   - Drone icon: sample along WGS84 track for ▶模拟飞行 (same UX as today: Plan first, then fly).  
   - Controls: orbit/tilt/zoom via Cesium camera; keep Plan / Fly / A*|RRT* / CN|US radar toggles.

2. **OSM fetch**  
   - Overpass query for `building=*` ways/relations inside bbox from start–goal + 1.5 km pad.  
   - Parse tags: `height`, `building:levels`, `name` (optional).  
   - Send building rings to WASM as lat/lon polygons + height_m; C++ converts to ENU AABBs (v1) with one shared origin so JS and planner never disagree on the frame.  
   - Cap at ~800 buildings; on Overpass timeout/fail: show error, keep last good map.

3. **C++ / WASM API (breaking)**  
   - New types: `GeoPoint { lat, lon, alt_m }`, `GeoOrigin` (lat/lon/alt of ENU frame).  
   - `enu_from_geo` / `geo_from_enu` (WGS84 ellipsoid, local tangent plane).  
   - `MapConfig` buildings/radars/start/goal expressed in geo in JSON; loader builds ENU world for `Environment`, `AStar3D`, `RRTStar`.  
   - WASM `plan(json)` input/output use lat/lon/alt for path waypoints; keep `mean_pd` / `max_pd`.  
   - CLI `--demo` and `configs/sample_map.json` migrate to Guomao geo sample; old meter sample archived under `configs/archive/`.

4. **Radars**  
   - Sites specified by lat/lon/alt + existing teaching profiles (`us_spy6_class`, `cn_jy27_class`).  
   - Ranges remain **lab-scaled to the local ~few-km map** (not real hundreds of km), with UI disclaimer unchanged.  
   - Beam geometry evaluated in ENU after conversion.

5. **Default Guomao sample** (indicative; tune at implement time)  
   - Start ≈ `39.908°N, 116.460°E`, alt 80–100 m  
   - Goal ≈ `39.912°N, 116.455°E`, alt 100–120 m  
   - Two teaching radars placed near the corridor in geo.

## Data flow

1. Page load → fill Guomao defaults → **auto-load once** (Overpass + imagery) → Cesium scene ready.  
2. User edits lat/lon → clicks **加载地图** → refresh buildings/obstacles (no fetch on every keystroke).  
3. **Plan** → geo JSON → WASM → geo path → Cesium polyline + stats.  
4. **▶模拟飞行** → interpolate along geo path; no auto-fly after plan.

## UI changes

- Replace x/y/z meter inputs with **lat / lon / alt (m)** for start and goal.  
- Add **加载地图**.  
- Retain algorithm select, radar toggles, Plan, Fly, stats (length, Pd).  
- Footer: OSM © contributors; imagery attribution; radar teaching disclaimer.

## Non-goals

- Full-planet resident dataset or offline global tileset.  
- Cesium ion paid 3D Tiles city meshes (optional later).  
- True long-range radar geometry (hundreds of km).  
- Editing OSM or uploading proprietary 测绘 files in v1.  
- Rewriting search to operate in raw degrees or ECEF without an ENU frame.

## Risks & mitigations

| Risk | Mitigation |
|------|------------|
| Overpass rate limit / timeout | Short timeout, one retry, alternate public endpoint; keep last map |
| Satellite / Cesium asset size | Load Cesium from CDN; our repo hosts only app JS + WASM |
| Sparse OSM heights in CBD | levels×3 m / 15 m fallback; silhouette still real |
| Breaking JSON / WASM API | Archive meter sample; bump README; Pages demo only geo |
| Too many buildings → jank | Cap ~800; simplify rings; AABB in planner |

## Success criteria

- Guomao default: satellite ground, real building footprints, flyable Plan path.  
- Change start/goal to another city (e.g. short Tokyo segment), **加载地图**, Plan succeeds.  
- Overpass failure: clear message, no crash.  
- A*/RRT* and CN/US radar toggles still change the path / Pd stats.  
- ▶模拟飞行 follows the WGS84 path end-to-end.  
- Design lives under `design-docs/` (not `docs/`) so GitHub Pages stays unblocked.

## Implementation notes (for plan skill)

- New: `include/uav/geo.hpp` (conversions).  
- Update: `environment` JSON schema, `planner_bind.cpp`, `docs/index.html` + new `app.js` (Cesium), `sample_map.json`.  
- Remove or stop shipping Three.js demo path once Cesium page is stable.  
- Verify Pages build: no markdown under `docs/`.
