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
    if (!planWorker) planWorker = new Worker('planner_worker.js?v=ux3');
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
    delete map.buildingSource;
    delete map._bbox;
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

  const OVERPASS_ENDPOINTS = [
    'https://overpass.openstreetmap.fr/api/interpreter',
    'https://overpass.osm.ch/api/interpreter',
    'https://overpass-api.de/api/interpreter',
    'https://lz4.overpass-api.de/api/interpreter',
    'https://overpass.kumi.systems/api/interpreter',
  ];

  async function postOverpass(url, query, signal, timeoutMs) {
    const ctrl = new AbortController();
    // openstreetmap.fr is whitelisted for the GitHub Pages referer and can take
    // ~25s for the Guomao extract. Later mirrors are usually a fast 502.
    const timer = setTimeout(() => ctrl.abort(), timeoutMs || 12000);
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
        referrerPolicy: 'origin',
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
      `[out:json][timeout:25];\n` +
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

  function overpassHost(url) {
    const parts = String(url).split('/');
    return parts[2] || url;
  }

  async function fetchOsmBuildings(minLat, minLon, maxLat, maxLon, isCancelled) {
    const bbox = { minLat, minLon, maxLat, maxLon };
    const pre = globalThis.__osmPrefetch;
    const meta = globalThis.__osmPrefetchMeta;
    if (pre && bboxClose(bbox, meta)) {
      globalThis.__osmPrefetch = null;
      try {
        const data = await pre;
        const buildings = parseOverpassToBuildings(data.elements);
        if (buildings.length) return buildings;
        console.warn('OSM prefetch returned no footprints');
      } catch (e) {
        console.warn('OSM prefetch failed, trying mirrors:', e);
      }
    }
    const query = overpassQuery(minLat, minLon, maxLat, maxLon);
    const errors = [];
    for (let i = 0; i < OVERPASS_ENDPOINTS.length; i++) {
      if (isCancelled && isCancelled()) throw new Error('cancelled');
      const url = OVERPASS_ENDPOINTS[i];
      const host = overpassHost(url);
      setChip(`Overpass ${i + 1}/${OVERPASS_ENDPOINTS.length} · ${host}`, true);
      try {
        const timeoutMs = i === 0 ? 28000 : 12000;
        const data = await postOverpass(url, query, undefined, timeoutMs);
        const buildings = parseOverpassToBuildings(data.elements);
        if (!buildings.length) throw new Error('no building footprints');
        return buildings;
      } catch (e) {
        const aborted = e && (e.name === 'AbortError' || /abort/i.test(String(e.message || e)));
        const msg = aborted ? 'timeout' : ((e && e.message) || String(e));
        errors.push(`${host}: ${msg}`);
        console.warn('Overpass failed:', url, e);
      }
    }
    throw new Error(errors.join(' | ') || 'Overpass failed on all mirrors');
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
    const layout = globalThis.DemoFraming
      ? DemoFraming.teachingRadarPositions(start, goal)
      : [];
    // yaw_deg is ENU (0 = east). Compass bearing points the beam off the corridor.
    function placed(role, index) {
      const pred = role === 'us' ? isUsRadar : isCnRadar;
      const tmpl = list.find(pred) || list[index] || list[0];
      if (!tmpl) return null;
      const lay = layout.find((s) => s.role === role);
      const r = structuredClone(tmpl);
      if (lay) {
        r.position = { lat: lay.position.lat, lon: lay.position.lon, alt: lay.position.alt };
        r.yaw_deg = lay.yawDeg;
        r.pitch_deg = lay.pitchDeg;
      }
      return r;
    }
    return [placed('us', 0), placed('cn', 1)].filter(Boolean);
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
      const halfAz = Math.max(6, Number(r.az_hpbw_deg) || 30) / 2;
      // Draw the half-power fan the planner costs, not a full-range disc.
      if (globalThis.DemoFraming) {
        const ring = DemoFraming.sectorRing(pos.lat, pos.lon, Number(r.yaw_deg) || 0, rangeM, halfAz, 20);
        const ringCartesian = ring.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat, 100));
        viewer.entities.add({
          name: (r.id || 'radar') + '-sector',
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(
              ring.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat))
            ),
            height: 12,
            extrudedHeight: 190,
            material: accent.withAlpha(0.22),
            outline: false,
          },
        });
        viewer.entities.add({
          name: (r.id || 'radar') + '-sector-edge',
          polyline: {
            positions: ringCartesian.concat(ringCartesian[0]),
            width: 2,
            material: accent.withAlpha(0.95),
            arcType: Cesium.ArcType.NONE,
          },
        });
      }
      viewer.entities.add({
        name: (r.id || 'radar') + '-mast',
        position: Cesium.Cartesian3.fromDegrees(pos.lon, pos.lat, siteAlt + 18),
        cylinder: {
          length: 36,
          topRadius: 4,
          bottomRadius: 14,
          material: accent.withAlpha(0.85),
          outline: false,
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

  const BUNDLE_URL = './guomao_osm.json?v=ux3';
  const OSM_CACHE_KEY = 'uav-osm-buildings-v1';

  function buildingBlurb(map) {
    const n = (map && map.buildings && map.buildings.length) || 0;
    const src = (map && map.buildingSource) || 'sample';
    if (src === 'live') return `实时 OSM（${n} 栋）`;
    if (src === 'browser') return `本机上次成功的 OSM（${n} 栋）`;
    if (src === 'bundled') return `随页面打包的国贸 OSM（${n} 栋，不是本次查询）`;
    return `离线手绘样例（${n} 栋盒子，不是 OSM）`;
  }

  function rememberLiveOsm(bbox, buildings) {
    try {
      const payload = JSON.stringify({ bbox: bbox, buildings: buildings, savedAt: new Date().toISOString() });
      if (payload.length < 4500000) localStorage.setItem(OSM_CACHE_KEY, payload);
    } catch (e) {
      console.warn('Could not save OSM cache', e);
    }
  }

  function readOsmCache(bbox) {
    try {
      const raw = localStorage.getItem(OSM_CACHE_KEY);
      if (!raw) return null;
      const data = JSON.parse(raw);
      if (!data || !bboxClose(bbox, data.bbox) || !Array.isArray(data.buildings) || !data.buildings.length) {
        return null;
      }
      return data;
    } catch (e) {
      return null;
    }
  }

  function commitCorridor(start, goal, bbox, buildings, source, background) {
    const hadPath = lastPath.length > 0 || flight.active;
    stopFlight();
    lastPath = [];
    removePathEntity();
    baseMap.start = start;
    baseMap.goal = goal;
    baseMap.origin = { lat: start.lat, lon: start.lon, alt: 0 };
    baseMap.coord_frame = 'wgs84';
    baseMap.buildings = buildings;
    // Search a corridor around start/goal. The Overpass bbox is wider so the
    // city is drawn, but a multi-km box makes RRT* miss the radar detour.
    const planBox = padBbox(start, goal, 900);
    baseMap.bounds = {
      min: { lat: planBox.minLat, lon: planBox.minLon, alt: 0 },
      max: { lat: planBox.maxLat, lon: planBox.maxLon, alt: 200 },
    };
    baseMap._bbox = bbox;
    baseMap.buildingSource = source;
    baseMap.radars = placeTeachingRadars(
      start,
      goal,
      (offlineTemplate && offlineTemplate.radars) || baseMap.radars
    );
    fillLatLon(baseMap);
    const keepCamera = !!background || userSteered;
    if (!background) userSteered = false;
    rebuildCesiumScene(baseMap, keepCamera);
    if (!keepCamera) userSteered = false;
    return hadPath;
  }

  async function applyKnownBuildings(bbox, start, goal, background) {
    if (!baseMap) return;
    const same = baseMap._bbox && bboxClose(bbox, baseMap._bbox);
    if (same && (baseMap.buildingSource === 'live' || baseMap.buildingSource === 'browser' || baseMap.buildingSource === 'bundled')) {
      return;
    }
    const cached = readOsmCache(bbox);
    if (cached) {
      commitCorridor(start, goal, bbox, cached.buildings, 'browser', background);
      setChip(`本机 OSM · ${cached.buildings.length} 栋 · 正在更新`, true);
      return;
    }
    try {
      const res = await fetch(BUNDLE_URL);
      if (!res.ok) return;
      const data = await res.json();
      if (!data || !bboxClose(bbox, data.bbox) || !Array.isArray(data.buildings) || data.buildings.length < 10) return;
      commitCorridor(start, goal, bbox, data.buildings, 'bundled', background);
      setChip(`打包的国贸 OSM · ${data.buildings.length} 栋 · 正在向 Overpass 更新`, true);
    } catch (e) {
      console.warn('Bundled OSM unavailable', e);
    }
  }

  function overpassFailureText(err) {
    const why = (err && err.message) ? err.message : String(err);
    const shown = buildingBlurb(baseMap);
    const sample = !baseMap || (baseMap.buildingSource || 'sample') === 'sample';
    return (
      `Overpass 失败：${why}\n` +
      `画面仍是${shown}。没有把失败当成缓存。\n` +
      `点「加载地图」会按表单里的经纬度再查一次。\n` +
      `Overpass failed. Still showing ${sample ? 'the hand-drawn sample, not OSM' : 'the buildings named above'}. Retry 加载地图.`
    );
  }

  async function loadSampleAndShow() {
    setChip('正在打开离线样例…', true);
    setStats('Loading offline sample…', '');
    const res = await fetch('./sample_map.json');
    if (!res.ok) throw new Error('Cannot load sample_map.json');
    baseMap = await res.json();
    baseMap.buildingSource = 'sample';
    offlineTemplate = structuredClone(baseMap);
    fillLatLon(baseMap);
    userSteered = false;
    rebuildCesiumScene(baseMap, false);
    const nB = (baseMap.buildings || []).length;
    const nR = filteredRadars(baseMap).length;
    setChip('手绘样例 · OSM 加载中', true);
    setStats(
      `手绘样例已就绪，可直接规划 · buildings: ${nB}（不是 OSM）  radars: ${nR}\n` +
        `正在向 Overpass 拉取国贸一带的建筑…`,
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
    const bbox = padBbox(start, goal, 1500);
    if (!background) {
      setChip('正在从 OSM 加载这一带的建筑…', true);
      setStats('正在从 OSM Overpass 加载建筑…\n当前画面可先规划；成功后会换成这一带的建筑。', '');
    }
    if ($('loadMapBtn')) $('loadMapBtn').disabled = true;

    try {
      await applyKnownBuildings(bbox, start, goal, background);
      if (seq !== loadSeq) return;
      const buildings = await withTimeout(
        fetchOsmBuildings(
          bbox.minLat,
          bbox.minLon,
          bbox.maxLat,
          bbox.maxLon,
          () => seq !== loadSeq
        ),
        50000,
        'Overpass 超时（50s，多个镜像都没有在时限内返回建筑）。'
      );
      if (seq !== loadSeq) return;
      if (!buildings.length) throw new Error('Overpass returned no building footprints');

      const hadPath = commitCorridor(start, goal, bbox, buildings, 'live', background);
      rememberLiveOsm(bbox, buildings);
      const nR = filteredRadars(baseMap).length;
      setChip(`OSM 实时 · ${buildings.length} 栋`, true);
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
      if (e && e.message === 'cancelled') return;
      console.error(e);
      setChip(`OSM 加载失败 · ${buildingBlurb(baseMap)}`, true);
      setStats(overpassFailureText(e), 'err');
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
      $(id).addEventListener('change', () => {
        if (lastPath.length && planBtn && !planBtn.disabled) {
          onPlan().catch((e) => setStats(String(e), 'err'));
        } else {
          refreshRadarVisibility();
        }
      });
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
