/* Cesium demo — Guomao WGS84 + OSM Overpass buildings. */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const statsEl = $('stats');
  const spinner = $('spinner');
  const planBtn = $('planBtn');

  let baseMap = null;
  let offlineTemplate = null;
  let viewer = null;
  let pathEntity = null;
  let droneEntity = null;
  let lastPath = [];
  let lastAlgo = 'astar';
  let flight = { active: false, t: 0, pts: [], dist: [], total: 0, speed: 40, _hold: 0 };
  let lastTick = performance.now();
  let buildingPrimitive = null;
  let userSteered = false;
  let followCam = false;
  let loadSeq = 0;
  let resizeTimer = 0;

  function setStats(text, cls) {
    statsEl.textContent = text;
    statsEl.className = cls || '';
  }

  function setChip(text, show) {
    const el = $('mapChip');
    if (!el) return;
    el.textContent = text || '';
    el.classList.toggle('show', !!show && !!text);
  }

  function withTimeout(promise, ms, message) {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(message)), ms);
      Promise.resolve(promise).then(
        (value) => {
          clearTimeout(timer);
          resolve(value);
        },
        (err) => {
          clearTimeout(timer);
          reject(err);
        }
      );
    });
  }

  let planWorker = null;
  let planToken = 0;

  function runPlan(mapJson, algo, informed, timeoutMs) {
    if (!planWorker) planWorker = new Worker('planner_worker.js?v=ux2');
    const worker = planWorker;
    const id = ++planToken;
    return new Promise((resolve, reject) => {
      let settled = false;
      const finish = (fn, value) => {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        worker.removeEventListener('message', onMsg);
        worker.removeEventListener('error', onErr);
        fn(value);
      };
      const timer = setTimeout(() => {
        try { worker.terminate(); } catch (e) { /* already dead */ }
        if (planWorker === worker) planWorker = null;
        finish(reject, new Error(
          '规划超时（' + Math.round(timeoutMs / 1000) +
            's），已中止并恢复页面。\nPlanning timed out and was stopped.'
        ));
      }, timeoutMs);
      const onMsg = (ev) => {
        if (!ev.data || ev.data.id !== id) return;
        if (ev.data.error) finish(reject, new Error(ev.data.error));
        else finish(resolve, ev.data.raw);
      };
      const onErr = (ev) => {
        try { worker.terminate(); } catch (e) { /* already dead */ }
        if (planWorker === worker) planWorker = null;
        finish(reject, new Error(ev.message || 'Planner worker failed'));
      };
      worker.addEventListener('message', onMsg);
      worker.addEventListener('error', onErr);
      worker.postMessage({ id: id, mapJson: mapJson, algo: algo, informed: informed });
    });
  }

  function num(id, fallback) {
    const v = parseFloat($(id).value);
    return Number.isFinite(v) ? v : fallback;
  }

  function fillLatLon(map) {
    $('slat').value = map.start.lat;
    $('slon').value = map.start.lon;
    $('salt').value = map.start.alt;
    $('glat').value = map.goal.lat;
    $('glon').value = map.goal.lon;
    $('galt').value = map.goal.alt;
  }

  function isUsRadar(r) {
    const id = (r.id || '').toLowerCase();
    return id.includes('us_') || id.includes('spy');
  }

  function isCnRadar(r) {
    const id = (r.id || '').toLowerCase();
    return id.includes('cn_') || id.includes('jy');
  }

  function filteredRadars(map) {
    const us = $('radarUs').checked;
    const cn = $('radarCn').checked;
    return (map.radars || []).filter((r) => {
      if (isUsRadar(r)) return us;
      if (isCnRadar(r)) return cn;
      return us || cn;
    });
  }

  // True if form lat/lon differ from the loaded map (needs 加载地图 first).
  function latLonStale() {
    if (!baseMap) return false;
    const eps = 1e-5;
    const d = (id, v) => Math.abs(num(id, v) - v) > eps;
    return (
      d('slat', baseMap.start.lat) || d('slon', baseMap.start.lon) ||
      d('glat', baseMap.goal.lat) || d('glon', baseMap.goal.lon)
    );
  }

  function currentMapJson() {
    const map = structuredClone(baseMap);
    if (latLonStale()) {
      // Buildings belong to baseMap's corridor: keep its lat/lon, allow alt edits.
      const sAlt = num('salt', map.start.alt);
      const gAlt = num('galt', map.goal.alt);
      fillLatLon(baseMap);
      $('salt').value = sAlt;
      $('galt').value = gAlt;
    }
    map.start = {
      lat: num('slat', map.start.lat),
      lon: num('slon', map.start.lon),
      alt: num('salt', map.start.alt),
    };
    map.goal = {
      lat: num('glat', map.goal.lat),
      lon: num('glon', map.goal.lon),
      alt: num('galt', map.goal.alt),
    };
    map.radars = filteredRadars(map);
    if (!map.origin) {
      map.origin = { lat: map.start.lat, lon: map.start.lon, alt: 0 };
    }
    map.coord_frame = map.coord_frame || 'wgs84';
    return map;
  }


  function padBbox(start, goal, padM = 1500) {
    const midLat = (start.lat + goal.lat) / 2;
    const mPerDegLat = 111320;
    const mPerDegLon = 111320 * Math.cos((midLat * Math.PI) / 180);
    const dLat = padM / mPerDegLat;
    const dLon = padM / Math.max(mPerDegLon, 1e-6);
    return {
      minLat: Math.min(start.lat, goal.lat) - dLat,
      maxLat: Math.max(start.lat, goal.lat) + dLat,
      minLon: Math.min(start.lon, goal.lon) - dLon,
      maxLon: Math.max(start.lon, goal.lon) + dLon,
    };
  }

  function parseHeightM(tags) {
    if (!tags) return 15;
    if (tags.height != null && tags.height !== '') {
      const raw = String(tags.height).replace(/,/g, '.').trim();
      const m = parseFloat(raw.replace(/m$/i, '').trim());
      if (Number.isFinite(m) && m > 0) return m;
    }
    const levels = tags['building:levels'];
    if (levels != null && levels !== '') {
      const n = parseFloat(String(levels).replace(/,/g, '.'));
      if (Number.isFinite(n) && n > 0) return n * 3;
    }
    return 15;
  }

  function footprintAreaM2(fp) {
    if (!fp || fp.length < 3) return 0;
    let lat0 = 0;
    for (const p of fp) lat0 += p.lat;
    lat0 /= fp.length;
    const mLat = 111320;
    const mLon = 111320 * Math.cos((lat0 * Math.PI) / 180);
    let a = 0;
    for (let i = 0; i < fp.length; i++) {
      const j = (i + 1) % fp.length;
      const xi = fp[i].lon * mLon;
      const yi = fp[i].lat * mLat;
      const xj = fp[j].lon * mLon;
      const yj = fp[j].lat * mLat;
      a += xi * yj - xj * yi;
    }
    return Math.abs(a) / 2;
  }

  function wayToFootprint(way, nodes) {
    if (Array.isArray(way.geometry) && way.geometry.length && globalThis.DemoFraming) {
      return DemoFraming.footprintFromLatLonList(way.geometry);
    }
    const fp = [];
    for (const id of way.nodes || []) {
      const n = nodes.get(id);
      if (!n) continue;
      const last = fp[fp.length - 1];
      if (last && last.lat === n.lat && last.lon === n.lon) continue;
      fp.push({ lat: n.lat, lon: n.lon });
    }
    if (fp.length >= 2) {
      const a = fp[0];
      const b = fp[fp.length - 1];
      if (a.lat === b.lat && a.lon === b.lon) fp.pop();
    }
    return fp;
  }

  function memberFootprint(member, ways, nodes) {
    if (Array.isArray(member.geometry) && member.geometry.length && globalThis.DemoFraming) {
      return DemoFraming.footprintFromLatLonList(member.geometry);
    }
    const w = ways.get(member.ref);
    if (!w) return [];
    return wayToFootprint(w, nodes);
  }

  function parseOverpassToBuildings(elements) {
    const nodes = new Map();
    const ways = new Map();
    for (const e of elements || []) {
      if (e.type === 'node' && e.lat != null && e.lon != null) {
        nodes.set(e.id, { lat: e.lat, lon: e.lon });
      } else if (e.type === 'way') {
        ways.set(e.id, e);
      }
    }

    const buildings = [];
    const usedWays = new Set();

    for (const e of elements || []) {
      if (e.type !== 'relation' || !e.tags || e.tags.building == null) continue;
      let best = null;
      let bestArea = -1;
      for (const m of e.members || []) {
        if (m.type !== 'way') continue;
        if (m.role && m.role !== 'outer') continue;
        const fp = memberFootprint(m, ways, nodes);
        if (fp.length < 3) continue;
        const area = footprintAreaM2(fp);
        if (area > bestArea) {
          bestArea = area;
          best = fp;
        }
        if (m.ref != null) usedWays.add(m.ref);
      }
      if (!best) continue;
      buildings.push({
        name: e.tags.name || `osm_rel_${e.id}`,
        height_m: parseHeightM(e.tags),
        footprint: best,
        _area: bestArea,
      });
    }

    for (const w of ways.values()) {
      if (!w.tags || w.tags.building == null) continue;
      if (usedWays.has(w.id)) continue;
      const fp = wayToFootprint(w, nodes);
      if (fp.length < 3) continue;
      buildings.push({
        name: w.tags.name || `osm_way_${w.id}`,
        height_m: parseHeightM(w.tags),
        footprint: fp,
        _area: footprintAreaM2(fp),
      });
    }

    buildings.sort((a, b) => b._area - a._area);
    return buildings.slice(0, 800).map((b) => ({
      name: b.name,
      height_m: b.height_m,
      footprint: b.footprint,
    }));
  }

  async function postOverpass(url, query, signal) {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 18000);
    const onAbort = () => ctrl.abort();
    if (signal) {
      if (signal.aborted) ctrl.abort();
      else signal.addEventListener('abort', onAbort, { once: true });
    }
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8',
          Accept: 'application/json',
        },
        body: 'data=' + encodeURIComponent(query),
        signal: ctrl.signal,
      });
      if (!res.ok) {
        const tip = await res.text().catch(() => '');
        throw new Error(`HTTP ${res.status}${tip ? ': ' + tip.slice(0, 80) : ''}`);
      }
      const data = await res.json();
      if (!data || !Array.isArray(data.elements)) {
        throw new Error('invalid Overpass JSON');
      }
      return data;
    } finally {
      clearTimeout(timer);
      if (signal) signal.removeEventListener('abort', onAbort);
    }
  }

  function overpassQuery(minLat, minLon, maxLat, maxLon) {
    return (
      `[out:json][timeout:18];\n` +
      `(\n` +
      `  way["building"](${minLat},${minLon},${maxLat},${maxLon});\n` +
      `  relation["building"](${minLat},${minLon},${maxLat},${maxLon});\n` +
      `);\n` +
      `out geom;`
    );
  }

  function bboxClose(a, b) {
    if (!a || !b) return false;
    const eps = 2e-4;
    return ['minLat', 'minLon', 'maxLat', 'maxLon'].every(
      (k) => Math.abs(Number(a[k]) - Number(b[k])) < eps
    );
  }

  async function firstOverpass(query) {
    const endpoints = [
      'https://overpass-api.de/api/interpreter',
      'https://overpass.kumi.systems/api/interpreter',
      'https://lz4.overpass-api.de/api/interpreter',
    ];
    const controllers = endpoints.map(() => new AbortController());
    const tasks = endpoints.map((url, i) => new Promise((resolve, reject) => {
      setTimeout(() => {
        postOverpass(url, query, controllers[i].signal).then(resolve, (e) => {
          console.warn('Overpass failed:', url, e);
          reject(e);
        });
      }, i * 2200);
    }));
    try {
      if (typeof Promise.any === 'function') return await Promise.any(tasks);
      return await new Promise((resolve, reject) => {
        let fails = 0;
        tasks.forEach((p) => p.then(resolve, () => {
          if (++fails === tasks.length) reject(new Error('Overpass failed on all mirrors'));
        }));
      });
    } finally {
      controllers.forEach((c) => c.abort());
    }
  }

  async function fetchOsmBuildings(minLat, minLon, maxLat, maxLon) {
    const bbox = { minLat, minLon, maxLat, maxLon };
    const pre = globalThis.__osmPrefetch;
    const meta = globalThis.__osmPrefetchMeta;
    if (pre && bboxClose(bbox, meta)) {
      globalThis.__osmPrefetch = null;
      try {
        const data = await pre;
        return parseOverpassToBuildings(data.elements);
      } catch (e) {
        console.warn('OSM prefetch failed, retrying:', e);
      }
    }
    const data = await firstOverpass(overpassQuery(minLat, minLon, maxLat, maxLon));
    return parseOverpassToBuildings(data.elements);
  }

  function bearingDeg(from, to) {
    const midLat = (from.lat + to.lat) / 2;
    const mPerDegLat = 111320;
    const mPerDegLon = 111320 * Math.cos((midLat * Math.PI) / 180);
    const east = (to.lon - from.lon) * mPerDegLon;
    const north = (to.lat - from.lat) * mPerDegLat;
    return ((Math.atan2(east, north) * 180) / Math.PI + 360) % 360;
  }

  function placeTeachingRadars(start, goal, templates) {
    const list = templates && templates.length
      ? templates
      : offlineTemplate && offlineTemplate.radars
        ? offlineTemplate.radars
        : [];
    const midLat = (start.lat + goal.lat) / 2;
    const midLon = (start.lon + goal.lon) / 2;
    const mPerDegLat = 111320;
    const mPerDegLon = 111320 * Math.cos((midLat * Math.PI) / 180);
    let dx = (goal.lon - start.lon) * mPerDegLon;
    let dy = (goal.lat - start.lat) * mPerDegLat;
    const len = Math.hypot(dx, dy) || 1;
    dx /= len;
    dy /= len;
    const px = -dy;
    const py = dx;
    const along = 250;
    const side = 500;

    function offset(base, eM, nM) {
      return {
        lat: base.lat + nM / mPerDegLat,
        lon: base.lon + eM / mPerDegLon,
        alt: 5,
      };
    }

    const usPos = offset(start, -dx * along + px * side, -dy * along + py * side);
    const cnPos = offset(goal, dx * along - px * side, dy * along - py * side);
    const mid = { lat: midLat, lon: midLon };

    const usT =
      list.find((r) => isUsRadar(r)) ||
      list[0] || {
        id: 'us_spy6_class',
        label: 'US SPY-6–class (teaching, scaled) / 教学近似',
        band: 'S (~3 GHz label)',
        yaw_deg: 135,
        pitch_deg: 8,
        az_hpbw_deg: 12,
        el_hpbw_deg: 10,
        max_range_m: 1200,
        snr_ref_linear: 200,
        r_ref_m: 400,
        sigma_ref_m2: 0.1,
        snr_thresh_db: 10,
        rcs_lo_factor: 1.0,
      };
    const cnT =
      list.find((r) => isCnRadar(r)) ||
      list[1] ||
      list[0] || {
        id: 'cn_jy27_class',
        label: 'CN JY-27–class (teaching, scaled) / 教学近似',
        band: 'VHF (~300 MHz label)',
        yaw_deg: 315,
        pitch_deg: 5,
        az_hpbw_deg: 35,
        el_hpbw_deg: 25,
        max_range_m: 1500,
        snr_ref_linear: 80,
        r_ref_m: 500,
        sigma_ref_m2: 0.1,
        snr_thresh_db: 8,
        rcs_lo_factor: 4.0,
      };

    function cloneAt(t, pos) {
      const r = structuredClone(t);
      r.position = pos;
      r.yaw_deg = bearingDeg(pos, mid);
      return r;
    }

    return [cloneAt(usT, usPos), cloneAt(cnT, cnPos)];
  }

  function makeRadarCanvas(isUs) {
    const size = 96;
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = size;
    const ctx = canvas.getContext('2d');
    const accent = isUs ? '#4aa3ff' : '#ffb347';
    ctx.clearRect(0, 0, size, size);
    ctx.fillStyle = 'rgba(20,28,40,0.85)';
    ctx.beginPath();
    ctx.arc(48, 52, 40, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#c5d0de';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(48, 82);
    ctx.lineTo(48, 48);
    ctx.stroke();
    ctx.strokeStyle = accent;
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.arc(48, 42, 22, Math.PI * 0.15, Math.PI * 0.95);
    ctx.stroke();
    ctx.fillStyle = accent;
    ctx.beginPath();
    ctx.arc(48, 42, 5, 0, Math.PI * 2);
    ctx.fill();
    return canvas;
  }

  function initViewer() {
    // Prefer Esri World Imagery without Cesium Ion token; OSM as picker fallback via baseLayerPicker.
    Cesium.Ion.defaultAccessToken = undefined;

    const esri = new Cesium.UrlTemplateImageryProvider({
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      maximumLevel: 19,
      credit: 'Esri, Maxar, Earthstar Geographics, and the GIS User Community',
    });

    viewer = new Cesium.Viewer('cesiumContainer', {
      animation: false,
      timeline: false,
      baseLayerPicker: true,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
      infoBox: false,
      selectionIndicator: false,
      baseLayer: new Cesium.ImageryLayer(esri),
      terrainProvider: new Cesium.EllipsoidTerrainProvider(),
      requestRenderMode: false,
    });

    // Morning in Beijing so building walls catch the sun instead of a flat noon wash.
    viewer.clock.currentTime = Cesium.JulianDate.fromIso8601('2026-06-21T02:00:00Z');
    viewer.clock.shouldAnimate = false;
    // Globe imagery stays unlit so Esri tiles are not a dark disk while buildings
    // still take the sun (PerInstanceColorAppearance uses scene.light).
    viewer.scene.globe.enableLighting = false;
    viewer.scene.globe.depthTestAgainstTerrain = true;
    viewer.scene.globe.baseColor = Cesium.Color.fromCssColorString('#6a7d55');
    if (viewer.scene.light && 'intensity' in viewer.scene.light) {
      viewer.scene.light.intensity = 2.4;
    }

    const dronePos = new Cesium.CallbackProperty(() => droneCartesian(), false);
    droneEntity = viewer.entities.add({
      id: 'uav-drone',
      show: false,
      position: dronePos,
      orientation: new Cesium.CallbackProperty(() => droneOrientation(), false),
      point: {
        pixelSize: 11,
        color: Cesium.Color.fromCssColorString('#3dd68c'),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      box: {
        dimensions: new Cesium.Cartesian3(48, 22, 9),
        material: Cesium.Color.fromCssColorString('#3dd68c').withAlpha(0.95),
        outline: true,
        outlineColor: Cesium.Color.WHITE,
      },
      label: {
        text: 'UAV',
        font: 'bold 13px sans-serif',
        fillColor: Cesium.Color.WHITE,
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 3,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -18),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });

    const canvas = viewer.scene.canvas;
    const markSteered = () => {
      userSteered = true;
      if (followCam) {
        followCam = false;
        viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
      }
    };
    canvas.addEventListener('pointerdown', markSteered, true);
    canvas.addEventListener('wheel', markSteered, { capture: true, passive: true });
    window.addEventListener('resize', scheduleResize);
    if (window.visualViewport) window.visualViewport.addEventListener('resize', scheduleResize);

    viewer.resize();
    frameCorridor({
      start: { lat: 39.908, lon: 116.46, alt: 90 },
      goal: { lat: 39.912, lon: 116.455, alt: 110 },
    });

    viewer.scene.preUpdate.addEventListener(() => {
      const now = performance.now();
      const dt = Math.min(0.05, (now - lastTick) / 1000);
      lastTick = now;
      updateFlight(dt);
    });
  }

  function removePathEntity() {
    for (const id of ['uav-path', 'uav-path-wall', 'uav-trail']) {
      const byId = viewer.entities.getById(id);
      if (byId) viewer.entities.remove(byId);
    }
    pathEntity = null;
  }

  function clearBuildingPrimitive() {
    if (buildingPrimitive && viewer) {
      viewer.scene.primitives.remove(buildingPrimitive);
    }
    buildingPrimitive = null;
  }

  function buildingColor(heightM) {
    const t = Math.max(0, Math.min(1, (heightM || 15) / 200));
    return Cesium.Color.fromBytes(
      Math.round(176 - t * 50),
      Math.round(186 - t * 36),
      Math.round(198 - t * 8),
      255
    );
  }

  function addBuildingPrimitive(buildings) {
    clearBuildingPrimitive();
    const instances = [];
    for (const b of buildings || []) {
      const fp = b.footprint || [];
      if (fp.length < 3) continue;
      try {
        const positions = fp.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
        const h = Math.max(3, Number(b.height_m) || 15);
        instances.push(new Cesium.GeometryInstance({
          geometry: new Cesium.PolygonGeometry({
            polygonHierarchy: new Cesium.PolygonHierarchy(positions),
            height: 0,
            extrudedHeight: h,
            vertexFormat: Cesium.PerInstanceColorAppearance.VERTEX_FORMAT,
          }),
          attributes: {
            color: Cesium.ColorGeometryInstanceAttribute.fromColor(buildingColor(h)),
          },
        }));
      } catch (err) {
        console.warn('skip building', b.name, err);
      }
    }
    if (!instances.length) return;
    buildingPrimitive = viewer.scene.primitives.add(new Cesium.Primitive({
      geometryInstances: instances,
      appearance: new Cesium.PerInstanceColorAppearance({
        closed: true,
        translucent: false,
        flat: false,
      }),
      asynchronous: instances.length > 40,
    }));
  }

  function scheduleResize() {
    if (resizeTimer) cancelAnimationFrame(resizeTimer);
    resizeTimer = requestAnimationFrame(() => {
      resizeTimer = 0;
      if (!viewer || viewer.isDestroyed()) return;
      viewer.resize();
      if (!userSteered && !flight.active && baseMap) frameCorridor(baseMap);
    });
  }

  function frameCorridor(map) {
    if (!viewer || !map || !map.start || !map.goal || !globalThis.DemoFraming) return;
    const canvas = viewer.scene.canvas;
    if (!canvas || canvas.clientWidth < 2 || canvas.clientHeight < 2) return;
    const f = DemoFraming.corridorFocus(map);
    const center = Cesium.Cartesian3.fromDegrees(f.lon, f.lat, f.alt);
    const sphere = new Cesium.BoundingSphere(center, f.radius);
    viewer.camera.viewBoundingSphere(
      sphere,
      new Cesium.HeadingPitchRange(
        Cesium.Math.toRadians(f.headingDeg),
        Cesium.Math.toRadians(f.pitchDeg),
        0
      )
    );
    // viewBoundingSphere locks the camera to the target frame. Unlock so the
    // user can still orbit, pan, and zoom out to the whole globe.
    viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
  }

  function clearMapEntities() {
    removePathEntity();
    clearBuildingPrimitive();
    const keep = new Set(['uav-drone']);
    const toRemove = [];
    viewer.entities.values.forEach((e) => {
      if (!keep.has(e.id)) toRemove.push(e);
    });
    toRemove.forEach((e) => viewer.entities.remove(e));
  }

  function rebuildCesiumScene(map, keepCamera) {
    clearMapEntities();
    addBuildingPrimitive(map.buildings || []);

    const radars = filteredRadars(map);
    for (const r of radars) {
      const us = isUsRadar(r);
      const pos = r.position;
      const accent = us
        ? Cesium.Color.fromCssColorString('#4aa3ff')
        : Cesium.Color.fromCssColorString('#ffb347');
      const rangeM = Math.max(80, Number(r.max_range_m) || 800);
      const siteAlt = pos.alt || 0;
      // Soft ground footprint + short translucent cylinder (lab-scaled range).
      viewer.entities.add({
        name: (r.id || 'radar') + '-footprint',
        position: Cesium.Cartesian3.fromDegrees(pos.lon, pos.lat, siteAlt),
        ellipse: {
          semiMajorAxis: rangeM,
          semiMinorAxis: rangeM,
          height: siteAlt,
          material: accent.withAlpha(0.12),
          outline: true,
          outlineColor: accent.withAlpha(0.45),
        },
      });
      viewer.entities.add({
        name: (r.id || 'radar') + '-cone',
        position: Cesium.Cartesian3.fromDegrees(pos.lon, pos.lat, siteAlt + 40),
        cylinder: {
          length: 80,
          topRadius: 8,
          bottomRadius: Math.min(220, rangeM * 0.18),
          material: accent.withAlpha(0.18),
          outline: true,
          outlineColor: accent.withAlpha(0.5),
        },
      });
      viewer.entities.add({
        name: r.label || r.id || 'radar',
        position: Cesium.Cartesian3.fromDegrees(pos.lon, pos.lat, siteAlt + 25),
        billboard: {
          image: makeRadarCanvas(us),
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          scale: 0.7,
        },
        label: {
          text: us ? 'US' : 'CN',
          font: 'bold 12px sans-serif',
          fillColor: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 2,
          style: Cesium.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new Cesium.Cartesian2(0, -42),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      });
    }

    const s = map.start;
    const g = map.goal;
    viewer.entities.add({
      name: 'START',
      position: Cesium.Cartesian3.fromDegrees(s.lon, s.lat, s.alt || 0),
      point: {
        pixelSize: 14,
        color: Cesium.Color.fromCssColorString('#3dd68c'),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: 'START',
        font: 'bold 12px sans-serif',
        fillColor: Cesium.Color.fromCssColorString('#3dd68c'),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -18),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
    viewer.entities.add({
      name: 'GOAL',
      position: Cesium.Cartesian3.fromDegrees(g.lon, g.lat, g.alt || 0),
      point: {
        pixelSize: 14,
        color: Cesium.Color.fromCssColorString('#e040a0'),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: 'GOAL',
        font: 'bold 12px sans-serif',
        fillColor: Cesium.Color.fromCssColorString('#e040a0'),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new Cesium.Cartesian2(0, -18),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });

    removePathEntity();

    if (!keepCamera) frameCorridor(map);
  }

  function drawPath(points, algo) {
    removePathEntity();
    if (!points || points.length < 2) return;

    const flat = [];
    for (const p of points) {
      flat.push(p.lon, p.lat, p.alt ?? 0);
    }
    const color =
      algo === 'rrtstar'
        ? Cesium.Color.fromCssColorString('#f0d040')
        : Cesium.Color.fromCssColorString('#40d0e0');

    const ground = [];
    const minH = [];
    const maxH = [];
    for (const p of points) {
      ground.push(p.lon, p.lat);
      minH.push(0);
      maxH.push(Math.max(0, p.alt ?? 0));
    }
    viewer.entities.add({
      id: 'uav-path-wall',
      wall: {
        positions: Cesium.Cartesian3.fromDegreesArray(ground),
        minimumHeights: minH,
        maximumHeights: maxH,
        material: color.withAlpha(0.32),
      },
    });
    pathEntity = viewer.entities.add({
      id: 'uav-path',
      polyline: {
        positions: Cesium.Cartesian3.fromDegreesArrayHeights(flat),
        width: 8,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.18,
          color: color,
        }),
        depthFailMaterial: color.withAlpha(0.65),
        clampToGround: false,
        arcType: Cesium.ArcType.NONE,
      },
    });
  }

  function droneSample() {
    if (!flight.pts || flight.pts.length < 2) return null;
    const here = sampleTrack(flight, flight.t || 0);
    const ahead = sampleTrack(flight, Math.min((flight.t || 0) + 25, flight.total || 0));
    return { here, ahead };
  }

  function droneCartesian() {
    const s = droneSample();
    if (!s) return Cesium.Cartesian3.fromDegrees(116.46, 39.908, 90);
    return Cesium.Cartesian3.fromDegrees(s.here.lon, s.here.lat, s.here.alt);
  }

  function droneOrientation() {
    const s = droneSample();
    if (!s) return Cesium.Quaternion.IDENTITY;
    const heading = Cesium.Math.toRadians(bearingDeg(s.here, s.ahead));
    const pos = Cesium.Cartesian3.fromDegrees(s.here.lon, s.here.lat, s.here.alt);
    return Cesium.Transforms.headingPitchRollQuaternion(
      pos,
      new Cesium.HeadingPitchRoll(heading, 0, 0)
    );
  }

  function updateChaseCamera() {
    if (!followCam || !flight.active || !viewer) return;
    const s = droneSample();
    if (!s) return;
    const heading = Cesium.Math.toRadians(bearingDeg(s.here, s.ahead));
    const pos = Cesium.Cartesian3.fromDegrees(s.here.lon, s.here.lat, s.here.alt);
    // Behind and above the aircraft, looking along its heading. trackedEntity
    // ignores a plain Cartesian3 viewFrom, so the chase is applied directly.
    viewer.camera.lookAt(
      pos,
      new Cesium.HeadingPitchRange(heading, Cesium.Math.toRadians(-28), 460)
    );
  }

  function trailPositions() {
    if (!flight.pts || flight.pts.length < 2) return [];
    const d = Math.min(Math.max(flight.t || 0, 0), flight.total || 0);
    const out = [];
    for (let i = 0; i < flight.pts.length; i++) {
      if (flight.dist[i] > d) break;
      const p = flight.pts[i];
      out.push(Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.alt));
    }
    const cur = sampleTrack(flight, d);
    out.push(Cesium.Cartesian3.fromDegrees(cur.lon, cur.lat, cur.alt));
    if (out.length < 2 && flight.pts[0]) {
      const p = flight.pts[0];
      out.push(Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.alt));
    }
    return out;
  }

  function haversineM(a, b) {
    const R = 6371000;
    const toR = Math.PI / 180;
    const dLat = (b.lat - a.lat) * toR;
    const dLon = (b.lon - a.lon) * toR;
    const la1 = a.lat * toR;
    const la2 = b.lat * toR;
    const h =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(la1) * Math.cos(la2) * Math.sin(dLon / 2) ** 2;
    const horiz = 2 * R * Math.asin(Math.min(1, Math.sqrt(h)));
    const dAlt = (b.alt || 0) - (a.alt || 0);
    return Math.hypot(horiz, dAlt);
  }

  function buildFlightTrack(points) {
    const pts = points.map((p) => ({ lat: p.lat, lon: p.lon, alt: p.alt ?? 0 }));
    const dist = [0];
    let total = 0;
    for (let i = 1; i < pts.length; i++) {
      total += haversineM(pts[i - 1], pts[i]);
      dist.push(total);
    }
    return { pts, dist, total: Math.max(total, 1e-6) };
  }

  function sampleTrack(track, s) {
    const { pts, dist, total } = track;
    const d = Math.min(Math.max(s, 0), total);
    let i = 1;
    while (i < dist.length && dist[i] < d) i++;
    const i0 = Math.max(0, i - 1);
    const i1 = Math.min(pts.length - 1, i);
    const span = dist[i1] - dist[i0] || 1;
    const u = (d - dist[i0]) / span;
    const a = pts[i0];
    const b = pts[i1];
    return {
      lat: a.lat + (b.lat - a.lat) * u,
      lon: a.lon + (b.lon - a.lon) * u,
      alt: a.alt + (b.alt - a.alt) * u,
    };
  }

  function setFlyButtons(flying) {
    const label = flying ? '■ 停止飞行' : '▶ 模拟飞行';
    for (const id of ['flyBtn', 'flyFab']) {
      const el = $(id);
      if (!el) continue;
      el.textContent = label;
      el.classList.toggle('flying', flying);
    }
  }

  function stopFlight() {
    flight.active = false;
    flight.t = 0;
    flight._hold = 0;
    followCam = false;
    if (viewer) {
      viewer.trackedEntity = undefined;
      viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    }
    if (droneEntity) droneEntity.show = false;
    const trail = viewer && viewer.entities.getById('uav-trail');
    if (trail) viewer.entities.remove(trail);
    setFlyButtons(false);
  }

  function startFlight(points) {
    if (!points || points.length < 2) {
      setStats('没有可飞路径，请先点「规划」\nNo path — Plan first', 'err');
      return false;
    }
    const track = buildFlightTrack(points);
    const speed = Math.max(28, track.total / 16);
    flight = { active: true, t: 0, ...track, speed, _hold: 0 };
    followCam = true;
    if (droneEntity) droneEntity.show = true;
    updateChaseCamera();
    const oldTrail = viewer.entities.getById('uav-trail');
    if (oldTrail) viewer.entities.remove(oldTrail);
    viewer.entities.add({
      id: 'uav-trail',
      polyline: {
        positions: new Cesium.CallbackProperty(() => trailPositions(), false),
        width: 5,
        material: Cesium.Color.fromCssColorString('#ffffff').withAlpha(0.9),
        clampToGround: false,
        arcType: Cesium.ArcType.NONE,
      },
    });
    setFlyButtons(true);
    setStats('模拟飞行中：镜头跟随无人机，白线是已飞航迹。拖拽可改为自由观察。', 'ok');
    return true;
  }

  function updateFlight(dt) {
    if (!flight.active || !droneEntity) return;
    flight.t += dt * flight.speed;
    updateChaseCamera();
    if (flight.t >= flight.total) {
      flight.t = flight.total;
      flight._hold = (flight._hold || 0) + dt;
      if (flight._hold > 1.2) {
        stopFlight();
        setStats('飞行结束。可再次点击「模拟飞行」重播。', 'ok');
      }
    } else {
      flight._hold = 0;
    }
  }

  async function loadSampleAndShow() {
    setChip('正在打开离线样例…', true);
    setStats('Loading offline sample…', '');
    const res = await fetch('./sample_map.json');
    if (!res.ok) throw new Error('Cannot load sample_map.json');
    baseMap = await res.json();
    offlineTemplate = structuredClone(baseMap);
    fillLatLon(baseMap);
    userSteered = false;
    rebuildCesiumScene(baseMap, false);
    const nB = (baseMap.buildings || []).length;
    const nR = filteredRadars(baseMap).length;
    setChip('样例地图已显示 · OSM 建筑后台加载中', true);
    setStats(
      `样例已就绪，可直接规划 · buildings: ${nB}  radars: ${nR}\n` +
        `OSM 在后台加载（Overpass 慢时不必等待）`,
      'ok'
    );
  }

  function readStartGoalFromInputs() {
    const fallbackS = (baseMap && baseMap.start) || { lat: 39.908, lon: 116.46, alt: 90 };
    const fallbackG = (baseMap && baseMap.goal) || { lat: 39.912, lon: 116.455, alt: 110 };
    return {
      start: {
        lat: num('slat', fallbackS.lat),
        lon: num('slon', fallbackS.lon),
        alt: num('salt', fallbackS.alt),
      },
      goal: {
        lat: num('glat', fallbackG.lat),
        lon: num('glon', fallbackG.lon),
        alt: num('galt', fallbackG.alt),
      },
    };
  }

  async function onLoadMap(opts) {
    const background = !!(opts && opts.background);
    if (!baseMap) {
      try {
        await loadSampleAndShow();
      } catch (e) {
        console.error(e);
        setStats(`Load error: ${e.message || e}`, 'err');
        setChip('样例地图加载失败', true);
        return;
      }
    }

    const seq = ++loadSeq;
    const { start, goal } = readStartGoalFromInputs();
    if (!background) {
      setChip('正在从 OSM 加载这一带的建筑…', true);
      setStats('正在从 OSM Overpass 加载建筑…\n样例场景仍可查看；规划会在加载完成后使用新建筑。', '');
    }
    if ($('loadMapBtn')) $('loadMapBtn').disabled = true;

    try {
      const bbox = padBbox(start, goal, 1500);
      const osmJob = fetchOsmBuildings(
        bbox.minLat,
        bbox.minLon,
        bbox.maxLat,
        bbox.maxLon
      );
      osmJob.catch(() => {});
      const buildings = await withTimeout(
        osmJob,
        22000,
        'Overpass 超时（22s）。仍显示当前建筑，可稍后重试「加载地图」。'
      );
      if (seq !== loadSeq) return;
      if (!buildings.length) throw new Error('Overpass returned no building footprints');

      const hadPath = lastPath.length > 0 || flight.active;
      stopFlight();
      lastPath = [];
      removePathEntity();
      baseMap.start = start;
      baseMap.goal = goal;
      baseMap.origin = { lat: start.lat, lon: start.lon, alt: 0 };
      baseMap.coord_frame = 'wgs84';
      baseMap.buildings = buildings;
      baseMap.bounds = {
        min: { lat: bbox.minLat, lon: bbox.minLon, alt: 0 },
        max: { lat: bbox.maxLat, lon: bbox.maxLon, alt: 200 },
      };
      baseMap.radars = placeTeachingRadars(
        start,
        goal,
        (offlineTemplate && offlineTemplate.radars) || baseMap.radars
      );

      fillLatLon(baseMap);
      // Background refresh of the same corridor should not yank a camera the user moved.
      const keepCamera = background || userSteered;
      if (!background) userSteered = false;
      rebuildCesiumScene(baseMap, keepCamera);
      if (!keepCamera) userSteered = false;
      const nR = filteredRadars(baseMap).length;
      setChip(`OSM · ${buildings.length} 栋建筑`, true);
      setStats(
        `OSM 已加载 · buildings: ${buildings.length}  radars: ${nR}\n` +
          `bbox ≈ ${(bbox.maxLat - bbox.minLat).toFixed(3)}° lat × ` +
          `${(bbox.maxLon - bbox.minLon).toFixed(3)}° lon\n` +
          `点击「规划」 / Click Plan` +
          (hadPath ? '\n建筑已更新，请重新规划。' : ''),
        'ok'
      );
    } catch (e) {
      if (seq !== loadSeq) return;
      console.error(e);
      setChip('OSM 加载失败 · 仍显示当前建筑', true);
      setStats(
        `Overpass 失败（保留上次建筑）: ${e.message || e}\n` +
          `可稍后重试「加载地图」。样例场景可以先规划。`,
        'err'
      );
      if (baseMap) fillLatLon(baseMap);
    } finally {
      if (seq === loadSeq && $('loadMapBtn')) $('loadMapBtn').disabled = false;
    }
  }

  async function onPlan() {
    if (planBtn.disabled) return;
    planBtn.disabled = true;
    spinner.textContent = 'Planning…';
    spinner.classList.add('show');
    const stale = latLonStale();
    setStats('Planning…', '');
    const planSeq = loadSeq;
    let tick = 0;
    try {
      stopFlight();
      const map = currentMapJson();
      rebuildCesiumScene(map, true);
      const algo = $('algo').value;
      lastAlgo = algo;
      const informed = $('informed').checked;
      const t0 = performance.now();
      tick = setInterval(() => {
        spinner.textContent = 'Planning… ' + Math.floor((performance.now() - t0) / 1000) + 's';
      }, 400);
      // WASM runs in a worker. A 20s cap always clears this overlay.
      const raw = await runPlan(JSON.stringify(map), algo, informed, 20000);
      const result = JSON.parse(raw);
      if (planSeq !== loadSeq) {
        setStats('建筑已在规划期间更新，请再点一次「规划」。\nBuildings changed during planning — Plan again.', 'err');
        return;
      }
      if (!result.success) {
        lastPath = [];
        drawPath([], algo);
        setStats(
          `Failed / 失败\n` +
            (result.error ? `error: ${result.error}\n` : '') +
            `time: ${result.time_ms?.toFixed?.(2) ?? result.time_ms} ms`,
          'err'
        );
        return;
      }
      lastPath = result.path || [];
      drawPath(lastPath, algo);
      const meanPd = result.mean_pd ?? 0;
      const maxPd = result.max_pd ?? 0;
      setStats(
        `OK · ${algo}${algo === 'rrtstar' && informed ? ' (informed)' : ''}\n` +
          `length: ${Number(result.path_length).toFixed(2)} m\n` +
          `cost: ${Number(result.path_cost).toFixed(2)}\n` +
          `mean_pd: ${Number(meanPd).toFixed(3)}  max_pd: ${Number(maxPd).toFixed(3)}\n` +
          `waypoints: ${result.waypoints}\n` +
          `nodes: ${result.nodes}\n` +
          `time: ${Number(result.time_ms).toFixed(2)} ms\n` +
          `→ 点「模拟飞行」让无人机图标沿路线飞` +
          (stale
            ? `\n⚠ 经纬度已改但未加载地图，已按当前地图起终点规划；请先点「加载地图」\n` +
              `⚠ Lat/lon changed without 加载地图 — planned with loaded map's start/goal`
            : ''),
        stale ? 'err' : 'ok'
      );
    } catch (e) {
      console.error(e);
      setStats(`Error: ${e.message || e}`, 'err');
    } finally {
      if (tick) clearInterval(tick);
      spinner.classList.remove('show');
      spinner.textContent = 'Planning…';
      planBtn.disabled = false;
    }
  }


  function refreshRadarVisibility() {
    if (!baseMap || !viewer) return;
    const savedPath = lastPath.slice();
    const savedAlgo = lastAlgo;
    rebuildCesiumScene(baseMap, true);
    if (savedPath.length) drawPath(savedPath, savedAlgo);
  }

  function onReset() {
    if (!baseMap) return;
    stopFlight();
    userSteered = false;
    fillLatLon(baseMap);
    $('algo').value = 'astar';
    $('informed').checked = true;
    $('radarUs').checked = true;
    $('radarCn').checked = true;
    lastPath = [];
    lastAlgo = 'astar';
    removePathEntity();
    rebuildCesiumScene(baseMap, false);
    drawPath([], 'astar');
    setStats('Reset inputs to current map start/goal. Click Plan / 加载地图.', '');
  }

  async function onFlyToggle() {
    if (flight.active) {
      stopFlight();
      setStats('已停止模拟飞行', '');
      return;
    }
    if (!lastPath.length) {
      setStats('请先点「规划」生成路径，再点「模拟飞行」', 'err');
      return;
    }
    startFlight(lastPath);
  }

  function bindUi() {
    planBtn.addEventListener('click', onPlan);
    $('resetBtn').addEventListener('click', onReset);
    $('loadMapBtn').addEventListener('click', () => {
      onLoadMap({ background: false }).catch((e) => setStats(String(e), 'err'));
    });
    for (const id of ['radarUs', 'radarCn']) {
      $(id).addEventListener('change', refreshRadarVisibility);
    }
    const flyHandler = () => {
      onFlyToggle().catch((e) => setStats(String(e), 'err'));
    };
    if ($('flyBtn')) $('flyBtn').addEventListener('click', flyHandler);
    if ($('flyFab')) $('flyFab').addEventListener('click', flyHandler);
    setFlyButtons(false);
  }

  async function main() {
    if (typeof Cesium === 'undefined') {
      setStats('Cesium failed to load from CDN', 'err');
      setChip('Cesium 未能从 CDN 加载', true);
      return;
    }
    initViewer();
    bindUi();
    try {
      await loadSampleAndShow();
    } catch (e) {
      console.warn('Offline sample failed:', e);
      setChip('离线样例加载失败', true);
    }
    // OSM continues in the background; the sample globe is already interactive.
    onLoadMap({ background: true }).catch((e) => {
      console.warn(e);
      setChip('OSM 加载失败 · 仍显示当前建筑', true);
    });
  }

  main().catch((e) => {
    console.error(e);
    setStats(`Init error: ${e.message || e}`, 'err');
  });
})();
