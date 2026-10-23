/* Cesium demo — Guomao WGS84 + OSM Overpass buildings. */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const statsEl = $('stats');
  const spinner = $('spinner');
  const planBtn = $('planBtn');

  let baseMap = null;
  let offlineTemplate = null;
  let plannerModule = null;
  let viewer = null;
  let pathEntity = null;
  let droneEntity = null;
  let lastPath = [];
  let lastAlgo = 'astar';
  let flight = { active: false, t: 0, pts: [], dist: [], total: 0, speed: 40, _hold: 0 };
  let lastTick = performance.now();

  function setStats(text, cls) {
    statsEl.textContent = text;
    statsEl.className = cls || '';
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

  function currentMapJson() {
    const map = structuredClone(baseMap);
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
        const w = ways.get(m.ref);
        if (!w) continue;
        const fp = wayToFootprint(w, nodes);
        if (fp.length < 3) continue;
        const area = footprintAreaM2(fp);
        if (area > bestArea) {
          bestArea = area;
          best = fp;
        }
        usedWays.add(m.ref);
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

  async function postOverpass(url, query) {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 35000);
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
    }
  }

  async function fetchOsmBuildings(minLat, minLon, maxLat, maxLon) {
    const query =
      `[out:json][timeout:25];\n` +
      `(\n` +
      `  way["building"](${minLat},${minLon},${maxLat},${maxLon});\n` +
      `  relation["building"](${minLat},${minLon},${maxLat},${maxLon});\n` +
      `);\n` +
      `out body; >; out skel qt;`;
    const endpoints = [
      'https://overpass-api.de/api/interpreter',
      'https://overpass.kumi.systems/api/interpreter',
      'https://lz4.overpass-api.de/api/interpreter',
    ];
    let lastErr = null;
    for (const url of endpoints) {
      try {
        const data = await postOverpass(url, query);
        return parseOverpassToBuildings(data.elements);
      } catch (e) {
        lastErr = e;
        console.warn('Overpass failed:', url, e);
      }
    }
    throw lastErr || new Error('Overpass failed on all mirrors');
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

  function makeDroneCanvas() {
    const s = 128;
    const c = document.createElement('canvas');
    c.width = c.height = s;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, s, s);
    const g = ctx.createRadialGradient(64, 64, 6, 64, 64, 60);
    g.addColorStop(0, 'rgba(61,214,140,0.45)');
    g.addColorStop(1, 'rgba(61,214,140,0)');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, s, s);
    ctx.strokeStyle = '#3dd68c';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(24, 24); ctx.lineTo(104, 104);
    ctx.moveTo(104, 24); ctx.lineTo(24, 104);
    ctx.stroke();
    [[24, 24], [104, 24], [24, 104], [104, 104]].forEach(([x, y]) => {
      ctx.beginPath();
      ctx.arc(x, y, 10, 0, Math.PI * 2);
      ctx.fillStyle = '#0d121a';
      ctx.fill();
      ctx.strokeStyle = '#3d9cf0';
      ctx.lineWidth = 3;
      ctx.stroke();
    });
    ctx.fillStyle = '#3d9cf0';
    ctx.beginPath();
    ctx.roundRect(44, 44, 40, 40, 8);
    ctx.fill();
    ctx.fillStyle = '#e7ecf3';
    ctx.font = 'bold 14px system-ui,sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('UAV', 64, 64);
    return c;
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

    viewer.scene.globe.depthTestAgainstTerrain = false;
    viewer.scene.globe.enableLighting = false;
    viewer.clock.shouldAnimate = true;

    droneEntity = viewer.entities.add({
      id: 'uav-drone',
      show: false,
      position: Cesium.Cartesian3.fromDegrees(116.46, 39.908, 90),
      billboard: {
        image: makeDroneCanvas(),
        verticalOrigin: Cesium.VerticalOrigin.CENTER,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        scale: 0.55,
      },
    });

    viewer.scene.preUpdate.addEventListener(() => {
      const now = performance.now();
      const dt = Math.min(0.05, (now - lastTick) / 1000);
      lastTick = now;
      updateFlight(dt);
    });
  }

  function removePathEntity() {
    if (pathEntity) viewer.entities.remove(pathEntity);
    const byId = viewer.entities.getById('uav-path');
    if (byId) viewer.entities.remove(byId);
    pathEntity = null;
  }

  function clearMapEntities() {
    removePathEntity();
    const keep = new Set(['uav-drone']);
    const toRemove = [];
    viewer.entities.values.forEach((e) => {
      if (!keep.has(e.id)) toRemove.push(e);
    });
    toRemove.forEach((e) => viewer.entities.remove(e));
  }

  function rebuildCesiumScene(map, keepCamera) {
    clearMapEntities();

    for (const b of map.buildings || []) {
      const fp = b.footprint || [];
      if (fp.length < 3) continue;
      const positions = fp.map((p) => Cesium.Cartesian3.fromDegrees(p.lon, p.lat));
      viewer.entities.add({
        name: b.name || 'building',
        polygon: {
          hierarchy: new Cesium.PolygonHierarchy(positions),
          extrudedHeight: b.height_m || 15,
          height: 0,
          material: Cesium.Color.fromCssColorString('#8a93a3').withAlpha(0.78),
          outline: true,
          outlineColor: Cesium.Color.BLACK.withAlpha(0.55),
        },
      });
    }

    const radars = filteredRadars(map);
    for (const r of radars) {
      const us = isUsRadar(r);
      const pos = r.position;
      viewer.entities.add({
        name: r.label || r.id || 'radar',
        position: Cesium.Cartesian3.fromDegrees(pos.lon, pos.lat, (pos.alt || 0) + 25),
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

    if (!keepCamera) flyCameraTo(map);
  }

  function flyCameraTo(map) {
    const b = map.bounds || {};
    const min = b.min || map.start;
    const max = b.max || map.goal;
    const midLat = ((min.lat || map.start.lat) + (max.lat || map.goal.lat)) / 2;
    const midLon = ((min.lon || map.start.lon) + (max.lon || map.goal.lon)) / 2;
    viewer.camera.flyTo({
      destination: Cesium.Cartesian3.fromDegrees(midLon, midLat - 0.006, 2200),
      orientation: {
        heading: Cesium.Math.toRadians(0),
        pitch: Cesium.Math.toRadians(-45),
        roll: 0,
      },
      duration: 1.2,
    });
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

    pathEntity = viewer.entities.add({
      id: 'uav-path',
      polyline: {
        positions: Cesium.Cartesian3.fromDegreesArrayHeights(flat),
        width: 6,
        material: color,
        depthFailMaterial: color.withAlpha(0.45),
        clampToGround: false,
        arcType: Cesium.ArcType.NONE,
      },
    });
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
    if (droneEntity) droneEntity.show = false;
    setFlyButtons(false);
  }

  function startFlight(points) {
    if (!points || points.length < 2) {
      setStats('没有可飞路径，请先点「规划」\nNo path — Plan first', 'err');
      return false;
    }
    const track = buildFlightTrack(points);
    const speed = Math.max(40, track.total / 12);
    flight = { active: true, t: 0, ...track, speed, _hold: 0 };
    if (droneEntity) {
      droneEntity.show = true;
      const here = sampleTrack(flight, 0);
      droneEntity.position = Cesium.Cartesian3.fromDegrees(here.lon, here.lat, here.alt);
    }
    setFlyButtons(true);
    setStats('模拟飞行中：无人机图标沿路线移动…', 'ok');
    return true;
  }

  function updateFlight(dt) {
    if (!flight.active || !droneEntity) return;
    flight.t += dt * flight.speed;
    if (flight.t >= flight.total) {
      const end = sampleTrack(flight, flight.total);
      droneEntity.position = Cesium.Cartesian3.fromDegrees(end.lon, end.lat, end.alt);
      flight.t = flight.total;
      flight._hold = (flight._hold || 0) + dt;
      if (flight._hold > 1.0) {
        stopFlight();
        setStats('飞行结束。可再次点击「模拟飞行」重播。', 'ok');
      }
      return;
    }
    flight._hold = 0;
    const here = sampleTrack(flight, flight.t);
    droneEntity.position = Cesium.Cartesian3.fromDegrees(here.lon, here.lat, here.alt);
  }

  async function ensurePlanner() {
    if (plannerModule) return plannerModule;
    if (typeof createPlanner !== 'function') {
      throw new Error('createPlanner not found — rebuild WASM (./web/build.sh)');
    }
    plannerModule = await createPlanner();
    return plannerModule;
  }

  async function loadSampleAndShow() {
    setStats('Loading offline sample…', '');
    const res = await fetch('./sample_map.json');
    if (!res.ok) throw new Error('Cannot load sample_map.json');
    baseMap = await res.json();
    offlineTemplate = structuredClone(baseMap);
    fillLatLon(baseMap);
    rebuildCesiumScene(baseMap, false);
    const nB = (baseMap.buildings || []).length;
    const nR = filteredRadars(baseMap).length;
    setStats(
      `Offline sample ready · buildings: ${nB}  radars: ${nR}\n正在拉取 OSM…`,
      ''
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

  async function onLoadMap() {
    if (!baseMap) {
      try {
        await loadSampleAndShow();
      } catch (e) {
        console.error(e);
        setStats(`Load error: ${e.message || e}`, 'err');
        return;
      }
    }

    const { start, goal } = readStartGoalFromInputs();
    setStats('正在从 OSM Overpass 加载建筑…\nFetching OSM buildings…', '');
    spinner.classList.add('show');
    if (planBtn) planBtn.disabled = true;
    if ($('loadMapBtn')) $('loadMapBtn').disabled = true;

    try {
      const bbox = padBbox(start, goal, 1500);
      const buildings = await fetchOsmBuildings(
        bbox.minLat,
        bbox.minLon,
        bbox.maxLat,
        bbox.maxLon
      );

      // Only mutate map state after Overpass succeeds.
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
      rebuildCesiumScene(baseMap, false);
      const nR = filteredRadars(baseMap).length;
      setStats(
        `OSM 已加载 · buildings: ${buildings.length}  radars: ${nR}\n` +
          `bbox ≈ ${(bbox.maxLat - bbox.minLat).toFixed(3)}° lat × ` +
          `${(bbox.maxLon - bbox.minLon).toFixed(3)}° lon\n` +
          `点击「规划」 / Click Plan`,
        'ok'
      );
    } catch (e) {
      console.error(e);
      setStats(
        `Overpass 失败（保留上次建筑）: ${e.message || e}\n` +
          `可稍后重试「加载地图」`,
        'err'
      );
      // Leave baseMap and scene untouched (last successful map stays).
    } finally {
      spinner.classList.remove('show');
      if (planBtn) planBtn.disabled = false;
      if ($('loadMapBtn')) $('loadMapBtn').disabled = false;
    }
  }

  async function onPlan() {
    planBtn.disabled = true;
    spinner.classList.add('show');
    setStats('Planning…', '');
    stopFlight();
    try {
      const map = currentMapJson();
      rebuildCesiumScene(map, true);
      const mod = await ensurePlanner();
      const algo = $('algo').value;
      lastAlgo = algo;
      const informed = $('informed').checked;
      const raw = mod.plan(JSON.stringify(map), algo, informed);
      const result = JSON.parse(raw);
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
          `→ 点「模拟飞行」让无人机图标沿路线飞`,
        'ok'
      );
    } catch (e) {
      console.error(e);
      setStats(`Error: ${e.message || e}`, 'err');
    } finally {
      spinner.classList.remove('show');
      planBtn.disabled = false;
    }
  }

  function onReset() {
    if (!baseMap) return;
    stopFlight();
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

  async function main() {
    if (typeof Cesium === 'undefined') {
      setStats('Cesium failed to load from CDN', 'err');
      return;
    }
    initViewer();
    try {
      await loadSampleAndShow();
    } catch (e) {
      console.warn('Offline sample failed:', e);
    }
    // Auto-load OSM once for Guomao (or current input) defaults.
    await onLoadMap();
    planBtn.addEventListener('click', onPlan);
    $('resetBtn').addEventListener('click', onReset);
    $('loadMapBtn').addEventListener('click', () => {
      onLoadMap().catch((e) => setStats(String(e), 'err'));
    });
    const flyHandler = () => {
      onFlyToggle().catch((e) => setStats(String(e), 'err'));
    };
    if ($('flyBtn')) $('flyBtn').addEventListener('click', flyHandler);
    if ($('flyFab')) $('flyFab').addEventListener('click', flyHandler);
    setFlyButtons(false);
    ensurePlanner().catch((e) => console.warn('WASM preload:', e));
  }

  main().catch((e) => {
    console.error(e);
    setStats(`Init error: ${e.message || e}`, 'err');
  });
})();
