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

function aabbMesh(box, color, opacity) {
  const sx = Math.max(0.01, box.max.x - box.min.x);
  const sy = Math.max(0.01, box.max.y - box.min.y);
  const sz = Math.max(0.01, box.max.z - box.min.z);
  const geo = new THREE.BoxGeometry(sx, sz, sy); // y-up: world Y <- map Z, world Z <- map Y
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

function marker(pos, color) {
  const geo = new THREE.SphereGeometry(1.6, 20, 16);
  const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.25 });
  const m = new THREE.Mesh(geo, mat);
  m.position.set(pos.x, pos.z, pos.y);
  return m;
}

function clearGroup(g) {
  if (!g) return;
  while (g.children.length) {
    const c = g.children.pop();
    if (c.geometry) c.geometry.dispose();
    if (c.material) c.material.dispose();
  }
}

function rebuildMapMeshes(map) {
  clearGroup(mapGroup);
  clearGroup(markersGroup);
  for (const b of map.buildings || []) {
    mapGroup.add(aabbMesh(b, 0x8a93a3, 1));
  }
  for (const z of map.risk_zones || []) {
    mapGroup.add(aabbMesh(z, 0xe07040, 0.28));
  }
  markersGroup.add(marker(map.start, 0x3dd68c));
  markersGroup.add(marker(map.goal, 0xe040a0));

  const b = map.bounds;
  const cx = (b.min.x + b.max.x) / 2;
  const cy = (b.min.y + b.max.y) / 2;
  const cz = (b.min.z + b.max.z) / 2;
  controls.target.set(cx, cz * 0.45, cy);
  camera.position.set(cx + 90, 70, cy + 90);
  controls.update();
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
  for (const p of points) {
    verts.push(p.x, p.z, p.y);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  const color = algo === 'rrtstar' ? 0xf0d040 : 0x40d0e0;
  const mat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
  pathObject = new THREE.Line(geo, mat);
  scene.add(pathObject);
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

  const hemi = new THREE.HemisphereLight(0xb8c8e0, 0x1a2030, 0.85);
  scene.add(hemi);
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
  scene.add(mapGroup);
  scene.add(markersGroup);

  window.addEventListener('resize', () => {
    const w = host.clientWidth;
    const h = host.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });

  (function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  })();
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
  try {
    const map = currentMapJson();
    rebuildMapMeshes(map);
    const mod = await ensurePlanner();
    const algo = $('algo').value;
    const informed = $('informed').checked;
    const raw = mod.plan(JSON.stringify(map), algo, informed);
    const result = JSON.parse(raw);
    if (!result.success) {
      drawPath([], algo);
      setStats(
        `Failed / 失败\n` +
          (result.error ? `error: ${result.error}\n` : '') +
          `time: ${result.time_ms?.toFixed?.(2) ?? result.time_ms} ms`,
        'err'
      );
      return;
    }
    drawPath(result.path, algo);
    setStats(
      `OK · ${algo}${algo === 'rrtstar' && informed ? ' (informed)' : ''}\n` +
        `length: ${result.path_length.toFixed(2)} m\n` +
        `cost: ${result.path_cost.toFixed(2)}\n` +
        `waypoints: ${result.waypoints}\n` +
        `nodes: ${result.nodes}\n` +
        `time: ${result.time_ms.toFixed(2)} ms`,
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
  fillStartGoal(baseMap);
  $('algo').value = 'astar';
  $('informed').checked = true;
  rebuildMapMeshes(baseMap);
  drawPath([], 'astar');
  setStats('Reset to sample map. Click Plan.', '');
}

async function main() {
  initScene();
  setStats('Loading sample map…', '');
  const res = await fetch('./sample_map.json');
  if (!res.ok) throw new Error('Cannot load sample_map.json');
  baseMap = await res.json();
  fillStartGoal(baseMap);
  rebuildMapMeshes(baseMap);
  setStats('Ready. Click Plan / 点击规划.', '');
  planBtn.addEventListener('click', onPlan);
  $('resetBtn').addEventListener('click', onReset);
  // Warm-load WASM in background
  ensurePlanner().catch((e) => console.warn('WASM preload:', e));
}

main().catch((e) => {
  console.error(e);
  setStats(`Init error: ${e.message || e}`, 'err');
});
