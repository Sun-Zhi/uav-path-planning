import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const statsEl = $('stats');
const spinner = $('spinner');
const planBtn = $('planBtn');

let baseMap = null;
let plannerModule = null;
let pathObject = null;
let scene, camera, renderer, controls;
let mapGroup = null;
let markersGroup = null;
let droneGroup = null;
let lastPath = [];
let flight = { active: false, t: 0, pts: [], dist: [], total: 0 };
let savedOrbit = null;

function setStats(text, cls) {
  statsEl.textContent = text;
  statsEl.className = cls || '';
}

function num(id, fallback) {
  const v = parseFloat($(id).value);
  return Number.isFinite(v) ? v : fallback;
}

function fillStartGoal(map) {
  $('sx').value = map.start.x;
  $('sy').value = map.start.y;
  $('sz').value = map.start.z;
  $('gx').value = map.goal.x;
  $('gy').value = map.goal.y;
  $('gz').value = map.goal.z;
}

function currentMapJson() {
  const map = structuredClone(baseMap);
  map.start = { x: num('sx', map.start.x), y: num('sy', map.start.y), z: num('sz', map.start.z) };
  map.goal = { x: num('gx', map.goal.x), y: num('gy', map.goal.y), z: num('gz', map.goal.z) };
  return map;
}

function mapToThree(p) {
  return new THREE.Vector3(p.x, p.z, p.y);
}

function aabbMesh(box, color, opacity) {
  const sx = Math.max(0.01, box.max.x - box.min.x);
  const sy = Math.max(0.01, box.max.y - box.min.y);
  const sz = Math.max(0.01, box.max.z - box.min.z);
  const geo = new THREE.BoxGeometry(sx, sz, sy);
  const mat = new THREE.MeshStandardMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    roughness: 0.75,
    metalness: 0.05,
    depthWrite: opacity >= 0.99,
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(
    (box.min.x + box.max.x) / 2,
    (box.min.z + box.max.z) / 2,
    (box.min.y + box.max.y) / 2
  );
  return mesh;
}

/** Flat pad + pin for start/goal (no spheres). */
function markerPad(pos, color, label) {
  const g = new THREE.Group();
  const pad = new THREE.Mesh(
    new THREE.CylinderGeometry(2.2, 2.2, 0.35, 24),
    new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.2 })
  );
  pad.position.set(pos.x, 0.2, pos.y);
  g.add(pad);
  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.18, 0.18, pos.z, 8),
    new THREE.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.4 })
  );
  pole.position.set(pos.x, pos.z / 2, pos.y);
  g.add(pole);
  const flag = makeLabelSprite(label, color, 140, 40);
  flag.scale.set(10, 2.8, 1);
  flag.position.set(pos.x, pos.z + 2.5, pos.y);
  g.add(flag);
  return g;
}

function makeLabelSprite(text, color, w = 256, h = 64) {
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = 'rgba(8,12,20,0.82)';
  roundRect(ctx, 4, 4, w - 8, h - 8, 10);
  ctx.fill();
  ctx.strokeStyle = '#' + (color >>> 0).toString(16).padStart(6, '0');
  ctx.lineWidth = 3;
  roundRect(ctx, 4, 4, w - 8, h - 8, 10);
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = `bold ${Math.floor(h * 0.42)}px system-ui,sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, w / 2, h / 2 + 1);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

/** Classic radar dish icon drawn on canvas (billboard sprite). */
function makeRadarIconTexture(isUs) {
  const size = 256;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, size, size);

  const accent = isUs ? '#4aa3ff' : '#ffb347';
  // soft glow plate
  const grd = ctx.createRadialGradient(128, 140, 20, 128, 140, 110);
  grd.addColorStop(0, 'rgba(20,28,40,0.95)');
  grd.addColorStop(1, 'rgba(20,28,40,0)');
  ctx.fillStyle = grd;
  ctx.fillRect(0, 0, size, size);

  // mast
  ctx.strokeStyle = '#c5d0de';
  ctx.lineWidth = 10;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(128, 210);
  ctx.lineTo(128, 118);
  ctx.stroke();

  // base
  ctx.fillStyle = '#9aa8bc';
  ctx.fillRect(96, 208, 64, 14);
  ctx.fillRect(108, 222, 40, 10);

  if (isUs) {
    // AESA flat panel (SPY-6–class look)
    ctx.save();
    ctx.translate(128, 100);
    ctx.rotate((-18 * Math.PI) / 180);
    ctx.fillStyle = accent;
    ctx.strokeStyle = '#e8f2ff';
    ctx.lineWidth = 3;
    roundRect(ctx, -52, -38, 104, 76, 8);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = 'rgba(8,16,28,0.35)';
    for (let row = 0; row < 5; row++) {
      for (let col = 0; col < 7; col++) {
        ctx.fillRect(-42 + col * 12, -28 + row * 12, 8, 8);
      }
    }
    ctx.restore();
  } else {
    // Parabolic dish (JY-27–class look)
    ctx.save();
    ctx.translate(128, 108);
    ctx.rotate((-25 * Math.PI) / 180);
    ctx.beginPath();
    ctx.ellipse(0, 0, 58, 42, 0, 0, Math.PI * 2);
    ctx.fillStyle = accent;
    ctx.fill();
    ctx.strokeStyle = '#fff3e0';
    ctx.lineWidth = 4;
    ctx.stroke();
    // dish rim highlight
    ctx.beginPath();
    ctx.ellipse(0, 0, 58, 42, 0, -0.2, Math.PI * 0.85);
    ctx.strokeStyle = 'rgba(255,255,255,0.55)';
    ctx.lineWidth = 5;
    ctx.stroke();
    // feed horn
    ctx.fillStyle = '#dde3ea';
    ctx.beginPath();
    ctx.moveTo(8, 0);
    ctx.lineTo(36, -10);
    ctx.lineTo(36, 10);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // radar waves
  ctx.strokeStyle = accent;
  ctx.globalAlpha = 0.85;
  for (let i = 0; i < 3; i++) {
    ctx.lineWidth = 3;
    ctx.beginPath();
    const rr = 70 + i * 18;
    ctx.arc(150, 88, rr, -1.1, -0.25);
    ctx.stroke();
  }
  ctx.globalAlpha = 1;

  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

function radarIconSprite(isUs) {
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: makeRadarIconTexture(isUs),
      transparent: true,
      depthTest: true,
    })
  );
  sprite.scale.set(14, 14, 1);
  return sprite;
}

function radarWedge(radar) {
  const group = new THREE.Group();
  const isUs = (radar.id || '').includes('us_') || (radar.band || '').toUpperCase().startsWith('S');
  const color = isUs ? 0x4aa3ff : 0xffb347;
  const label = isUs ? 'US SPY-6' : 'CN JY-27';

  const r = radar.max_range_m || 50;
  const vizAz = Math.max(radar.az_hpbw_deg || 20, 50);
  const halfAz = (vizAz * Math.PI) / 360;
  const yaw = ((radar.yaw_deg || 0) * Math.PI) / 180;
  const cx = radar.position.x;
  const cy = radar.position.y;
  const cz = Math.max(radar.position.z || 2, 2);
  const segments = 36;
  const toThree = (x, y, z) => [x, z, y];

  // Semi-transparent coverage fan on ground plane height
  const positions = [];
  for (let i = 0; i < segments; i++) {
    const a0 = yaw - halfAz + (2 * halfAz * i) / segments;
    const a1 = yaw - halfAz + (2 * halfAz * (i + 1)) / segments;
    const p0x = cx + r * Math.cos(a0);
    const p0y = cy + r * Math.sin(a0);
    const p1x = cx + r * Math.cos(a1);
    const p1y = cy + r * Math.sin(a1);
    positions.push(...toThree(cx, cy, 0.3), ...toThree(p0x, p0y, 0.3), ...toThree(p1x, p1y, 0.3));
  }
  const fanGeo = new THREE.BufferGeometry();
  fanGeo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  group.add(
    new THREE.Mesh(
      fanGeo,
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.2,
        side: THREE.DoubleSide,
        depthWrite: false,
      })
    )
  );

  const arcPts = [new THREE.Vector3(...toThree(cx, cy, 0.35))];
  for (let i = 0; i <= segments; i++) {
    const a = yaw - halfAz + (2 * halfAz * i) / segments;
    arcPts.push(new THREE.Vector3(...toThree(cx + r * Math.cos(a), cy + r * Math.sin(a), 0.35)));
  }
  arcPts.push(new THREE.Vector3(...toThree(cx, cy, 0.35)));
  group.add(
    new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(arcPts),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.85 })
    )
  );

  // Real radar icon (sprite), not a sphere
  const icon = radarIconSprite(isUs);
  icon.position.set(cx, 8, cy);
  group.add(icon);

  const tag = makeLabelSprite(label, color, 220, 48);
  tag.scale.set(16, 3.5, 1);
  tag.position.set(cx, 16.5, cy);
  group.add(tag);

  return group;
}

/** Simple quadrotor mesh for chase/FPV reference (hidden in pure FPV). */
function createDrone() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(1.4, 0.35, 2.2),
    new THREE.MeshStandardMaterial({ color: 0x2a3340, metalness: 0.4, roughness: 0.4 })
  );
  g.add(body);
  const armMat = new THREE.MeshStandardMaterial({ color: 0x3d9cf0, metalness: 0.5, roughness: 0.35 });
  [[-1.1, 1.1], [1.1, 1.1], [-1.1, -1.1], [1.1, -1.1]].forEach(([x, z]) => {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.12, 1.6), armMat);
    arm.position.set(x * 0.55, 0.05, z * 0.55);
    arm.rotation.y = Math.atan2(z, x);
    g.add(arm);
    const motor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.35, 0.35, 0.2, 16),
      new THREE.MeshStandardMaterial({ color: 0x111820 })
    );
    motor.position.set(x, 0.2, z);
    g.add(motor);
  });
  // forward cue
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.25, 0.6, 10),
    new THREE.MeshStandardMaterial({ color: 0x3dd68c })
  );
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, 0, 1.3);
  g.add(nose);
  g.visible = false;
  return g;
}

function clearGroup(g) {
  if (!g) return;
  while (g.children.length) {
    const c = g.children.pop();
    if (c.isGroup) clearGroup(c);
    if (c.geometry) c.geometry.dispose();
    if (c.material) {
      if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose());
      else {
        if (c.material.map) c.material.map.dispose();
        c.material.dispose();
      }
    }
  }
}

function rebuildMapMeshes(map, keepCamera = false) {
  clearGroup(mapGroup);
  clearGroup(markersGroup);
  for (const b of map.buildings || []) mapGroup.add(aabbMesh(b, 0x8a93a3, 1));
  for (const z of map.risk_zones || []) mapGroup.add(aabbMesh(z, 0xe07040, 0.28));
  for (const r of map.radars || []) mapGroup.add(radarWedge(r));
  markersGroup.add(markerPad(map.start, 0x3dd68c, 'START'));
  markersGroup.add(markerPad(map.goal, 0xe040a0, 'GOAL'));

  if (!keepCamera && !flight.active) {
    const b = map.bounds;
    const cx = (b.min.x + b.max.x) / 2;
    const cy = (b.min.y + b.max.y) / 2;
    const cz = (b.min.z + b.max.z) / 2;
    controls.target.set(cx, cz * 0.45, cy);
    camera.position.set(cx + 90, 70, cy + 90);
    controls.update();
  }
}

function drawPath(points, algo) {
  if (pathObject) {
    scene.remove(pathObject);
    pathObject.geometry.dispose();
    pathObject.material.dispose();
    pathObject = null;
  }
  if (!points || !points.length) return;
  const verts = [];
  for (const p of points) verts.push(p.x, p.z, p.y);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  const color = algo === 'rrtstar' ? 0xf0d040 : 0x40d0e0;
  pathObject = new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
  scene.add(pathObject);
}

function buildFlightTrack(points) {
  const pts = points.map(mapToThree);
  const dist = [0];
  let total = 0;
  for (let i = 1; i < pts.length; i++) {
    total += pts[i].distanceTo(pts[i - 1]);
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
  const pos = pts[i0].clone().lerp(pts[i1], u);
  let dir;
  if (i1 > i0) dir = pts[i1].clone().sub(pts[i0]).normalize();
  else if (i0 > 0) dir = pts[i0].clone().sub(pts[i0 - 1]).normalize();
  else dir = new THREE.Vector3(0, 0, 1);
  return { pos, dir };
}

function setFpvHud(on) {
  const hud = $('fpvHud');
  if (!hud) return;
  hud.classList.toggle('show', on);
}

function setFlyButtons(flying) {
  const label = flying ? '■ 停止飞行' : '▶ 模拟飞行（第一视角）';
  const fabLabel = flying ? '■ 停止' : '▶ 模拟飞行';
  for (const id of ['flyBtn', 'flyFab']) {
    const el = $(id);
    if (!el) continue;
    el.textContent = id === 'flyFab' ? fabLabel : label;
    el.classList.toggle('flying', flying);
  }
}

function stopFlight(restoreOrbit = true) {
  flight.active = false;
  flight.t = 0;
  flight._hold = 0;
  if (droneGroup) droneGroup.visible = false;
  setFpvHud(false);
  controls.enabled = true;
  if (restoreOrbit && savedOrbit) {
    camera.position.copy(savedOrbit.pos);
    controls.target.copy(savedOrbit.target);
    camera.fov = 50;
    camera.updateProjectionMatrix();
    controls.update();
  }
  setFlyButtons(false);
}

function startFlight(points) {
  if (!points || points.length < 2) {
    setStats('没有可飞路径，请先点「规划」\nNo path — Plan first', 'err');
    return false;
  }
  const track = buildFlightTrack(points);
  // ~10s flight, minimum noticeable speed
  const speed = Math.max(12, track.total / 10);
  flight = { active: true, t: 0, ...track, speed, _hold: 0 };
  savedOrbit = {
    pos: camera.position.clone(),
    target: controls.target.clone(),
  };
  controls.enabled = false;
  camera.fov = 75;
  camera.near = 0.05;
  camera.updateProjectionMatrix();
  if (droneGroup) droneGroup.visible = false;
  setFpvHud(true);
  setFlyButtons(true);
  // snap camera to path start immediately so motion is obvious
  const here = sampleTrack(flight, 0);
  const ahead = sampleTrack(flight, Math.min(6, flight.total));
  camera.position.copy(here.pos);
  camera.up.set(0, 1, 0);
  camera.lookAt(ahead.pos);
  setStats('模拟飞行中（第一视角）… 再点按钮可停止', 'ok');
  return true;
}

function updateFlight(dt) {
  if (!flight.active) return;
  flight.t += dt * flight.speed;
  if (flight.t >= flight.total) {
    // linger at goal then restore
    const end = sampleTrack(flight, flight.total);
    camera.position.copy(end.pos);
    const look = end.pos.clone().add(end.dir);
    camera.lookAt(look);
    flight.t = flight.total;
    // auto-exit after brief hold handled by accumulating overshoot
    flight._hold = (flight._hold || 0) + dt;
    if (flight._hold > 1.2) stopFlight(true);
    return;
  }
  flight._hold = 0;
  const lookAhead = Math.min(flight.t + 4, flight.total);
  const here = sampleTrack(flight, flight.t);
  const ahead = sampleTrack(flight, lookAhead);
  camera.position.copy(here.pos);
  camera.up.set(0, 1, 0);
  camera.lookAt(ahead.pos);

  const pct = Math.floor((100 * flight.t) / flight.total);
  const hudMeta = $('fpvMeta');
  if (hudMeta) hudMeta.textContent = `FPV ${pct}% · ${flight.t.toFixed(0)} / ${flight.total.toFixed(0)} m`;
}

function initScene() {
  const host = $('canvas-host');
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x080b10);
  scene.fog = new THREE.Fog(0x080b10, 180, 320);

  camera = new THREE.PerspectiveCamera(50, host.clientWidth / host.clientHeight, 0.1, 1000);
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(host.clientWidth, host.clientHeight);
  host.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xb8c8e0, 0x1a2030, 0.85));
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(40, 80, 20);
  scene.add(dir);

  const grid = new THREE.GridHelper(100, 20, 0x334155, 0x1e293b);
  grid.position.set(50, 0, 50);
  scene.add(grid);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(100, 100),
    new THREE.MeshStandardMaterial({ color: 0x121820, roughness: 0.95 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.set(50, -0.02, 50);
  scene.add(ground);

  mapGroup = new THREE.Group();
  markersGroup = new THREE.Group();
  droneGroup = createDrone();
  scene.add(mapGroup);
  scene.add(markersGroup);
  scene.add(droneGroup);

  window.addEventListener('resize', () => {
    const w = host.clientWidth;
    const h = host.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });

  let last = performance.now();
  (function animate(now) {
    requestAnimationFrame(animate);
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    updateFlight(dt);
    if (!flight.active) controls.update();
    renderer.render(scene, camera);
  })(last);
}

async function ensurePlanner() {
  if (plannerModule) return plannerModule;
  if (typeof createPlanner !== 'function') {
    throw new Error('createPlanner not found — rebuild WASM (./web/build.sh)');
  }
  plannerModule = await createPlanner();
  return plannerModule;
}

async function onPlan() {
  planBtn.disabled = true;
  spinner.classList.add('show');
  setStats('Planning…', '');
  stopFlight(false);
  try {
    const map = currentMapJson();
    rebuildMapMeshes(map, true);
    const mod = await ensurePlanner();
    const algo = $('algo').value;
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
        `length: ${result.path_length.toFixed(2)} m\n` +
        `cost: ${result.path_cost.toFixed(2)}\n` +
        `mean_pd: ${meanPd.toFixed(3)}  max_pd: ${maxPd.toFixed(3)}\n` +
        `waypoints: ${result.waypoints}\n` +
        `nodes: ${result.nodes}\n` +
        `time: ${result.time_ms.toFixed(2)} ms\n` +
        `→ 点击「模拟飞行」或等待自动起飞`,
      'ok'
    );
    startFlight(lastPath);
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
  stopFlight(true);
  fillStartGoal(baseMap);
  $('algo').value = 'astar';
  $('informed').checked = true;
  lastPath = [];
  rebuildMapMeshes(baseMap);
  drawPath([], 'astar');
  setStats('Reset to sample map. Click Plan.', '');
}

async function onFlyToggle() {
  if (flight.active) {
    stopFlight(true);
    setStats('已停止模拟飞行', '');
    return;
  }
  if (!lastPath.length) {
    setStats('尚无路径，先自动规划再飞…', '');
    await onPlan(); // onPlan already starts flight on success
    if (!flight.active && lastPath.length >= 2) startFlight(lastPath);
    return;
  }
  startFlight(lastPath);
}

async function main() {
  initScene();
  setStats('Loading sample map…', '');
  const res = await fetch('./sample_map.json');
  if (!res.ok) throw new Error('Cannot load sample_map.json');
  baseMap = await res.json();
  fillStartGoal(baseMap);
  rebuildMapMeshes(baseMap);
  const nRad = (baseMap.radars || []).length;
  setStats(`Ready (${nRad} radars). Click Plan / 点击规划.`, '');
  planBtn.addEventListener('click', onPlan);
  $('resetBtn').addEventListener('click', onReset);
  const flyHandler = () => { onFlyToggle().catch((e) => setStats(String(e), 'err')); };
  if ($('flyBtn')) $('flyBtn').addEventListener('click', flyHandler);
  if ($('flyFab')) $('flyFab').addEventListener('click', flyHandler);
  setFlyButtons(false);
  ensurePlanner().catch((e) => console.warn('WASM preload:', e));
}

main().catch((e) => {
  console.error(e);
  setStats(`Init error: ${e.message || e}`, 'err');
});
