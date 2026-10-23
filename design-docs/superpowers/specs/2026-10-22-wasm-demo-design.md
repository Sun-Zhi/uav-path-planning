# WASM UAV Path Planning Demo — Design

**Date:** 2026-10-22  
**Status:** Approved by user (static Three.js page + Emscripten WASM, editable start/goal)

## Goal

A browser demo on GitHub Pages that loads the sample urban map, runs the existing C++ A* / Informed RRT* planners in WebAssembly, and visualizes buildings, EW risk zones, and resulting paths in 3D.

## Architecture

1. **Planner core (unchanged algorithms):** Keep header-only planners under `include/uav/`. Add `load_map_json_string(const std::string&)` so maps can be passed from JS without filesystem. Optional start/goal override from the page (replace `cfg.start` / `cfg.goal` after parse).
2. **WASM glue:** `web/planner_bind.cpp` exports `plan(mapJson, algorithm, informed)` via Embind. Returns a JSON string: `{success, path_length, path_cost, waypoints, nodes, time_ms, path:[{x,y,z},...]}`. Algorithms: `"astar"` | `"rrtstar"`.
3. **Build:** `web/build.sh` installs/uses Emscripten, produces `docs/planner.js` + `docs/planner.wasm` (and optional `.wasm` sidecar). CMake native CLI remains the default build.
4. **Frontend:** `docs/index.html` (+ small `docs/app.js` if needed) with Three.js (CDN). Renders AABB buildings (solid), risk zones (transparent), start/goal markers, and one or both paths. Controls: algorithm toggle, Plan button, start/goal xyz fields, reset to sample map.
5. **Data:** Embed or fetch `configs/sample_map.json` as `docs/sample_map.json` (copied at build time).
6. **Hosting:** GitHub Pages from `/docs` on `master`.

## Non-goals

Map editor, multi-map gallery, ROS/dynamics, offline-first CDN mirror, custom lighting/shaders beyond a clear demo look.

## Success criteria

- Open Pages URL → see map → click Plan → path appears and stats show within a few seconds.
- Native `./build/uav_planner --demo` still passes.
- Repo documents how to rebuild WASM.

## Risks

- Emscripten not preinstalled on the box → install via emsdk in `web/build.sh`.
- Large A* node expansions in browser → keep sample resolution 2 m; show spinner while planning.
