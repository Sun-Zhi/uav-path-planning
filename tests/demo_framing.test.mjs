import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const F = require('../docs/framing.js');

const guomao = {
  bounds: {
    min: { lat: 39.902, lon: 116.45 },
    max: { lat: 39.916, lon: 116.468 },
  },
  start: { lat: 39.908, lon: 116.46, alt: 90 },
  goal: { lat: 39.912, lon: 116.455, alt: 110 },
};

// The previous camera sat 0.006° south of the bbox center at 2200 m, pitch -45,
// and did not look at the corridor. Vertical FOV then shrinks on short windows.
function legacyCamera(map) {
  const min = map.bounds.min;
  const max = map.bounds.max;
  const midLat = (min.lat + max.lat) / 2;
  const midLon = (min.lon + max.lon) / 2;
  return { lat: midLat - 0.006, lon: midLon, alt: 2200, pitch: -45 };
}

test('vertical FOV matches Cesium when the viewport is short and wide', () => {
  const fovy = F.verticalFovRad(Math.PI / 3, 2.5) * 180 / Math.PI;
  assert.ok(Math.abs(fovy - 26.0078) < 0.02, `fovy ${fovy}`);
  const wide = F.verticalFovRad(Math.PI / 3, 1.222) * 180 / Math.PI;
  assert.ok(Math.abs(wide - 50.578) < 0.02, `wide ${wide}`);
});

test('legacy fixed pitch drops the corridor out of a short viewport', () => {
  const cam = legacyCamera(guomao);
  const focus = F.corridorFocus(guomao);
  const targetPitch = F.pitchDownToTargetDeg(cam, focus);
  const fovy = F.verticalFovRad(Math.PI / 3, 2.5) * 180 / Math.PI;
  assert.ok(targetPitch < -65, `target pitch ${targetPitch}`);
  assert.equal(F.targetInFrustum(cam.pitch, targetPitch, fovy), false);
});

test('corridor focus is oblique and looks at the start/goal midpoint', () => {
  const focus = F.corridorFocus(guomao);
  assert.equal(focus.pitchDeg, -32);
  assert.ok(focus.pitchDeg > -50 && focus.pitchDeg < -20);
  assert.equal(focus.lat, (guomao.start.lat + guomao.goal.lat) / 2);
  assert.ok(focus.radius >= 280);
  const cam = { lat: focus.lat, lon: focus.lon, alt: focus.alt + 400, pitch: focus.pitchDeg };
  // A camera aimed at the focus has the target at the same pitch as the camera.
  const aimed = focus.pitchDeg;
  const fovy = F.verticalFovRad(Math.PI / 3, 2.5) * 180 / Math.PI;
  assert.equal(F.targetInFrustum(aimed, aimed, fovy), true);
  assert.ok(cam.alt > focus.alt);
});

test('enu yaw is east-zero, not a compass bearing', () => {
  const from = { lat: 39.91, lon: 116.46 };
  const north = { lat: 39.92, lon: 116.46 };
  const east = { lat: 39.91, lon: 116.47 };
  assert.ok(Math.abs(F.enuYawDeg(from, north) - 90) < 0.2, F.enuYawDeg(from, north));
  assert.ok(Math.abs(F.enuYawDeg(from, east)) < 0.2 || Math.abs(F.enuYawDeg(from, east) - 360) < 0.2);
  assert.ok(Math.abs(F.compassBearingDeg(from, north)) < 0.2);
  assert.ok(Math.abs(F.compassBearingDeg(from, east) - 90) < 0.2);
});

test('teaching radars aim with ENU yaw across the Guomao corridor', () => {
  const sites = F.teachingRadarPositions(guomao.start, guomao.goal);
  assert.equal(sites.length, 2);
  const mid = {
    lat: (guomao.start.lat + guomao.goal.lat) / 2,
    lon: (guomao.start.lon + guomao.goal.lon) / 2,
  };
  for (const s of sites) {
    const enu = F.enuYawDeg(s.position, mid);
    const compass = F.compassBearingDeg(s.position, mid);
    const yawErr = Math.abs(((s.yawDeg - enu + 540) % 360) - 180);
    const compassErr = Math.abs(((s.yawDeg - compass + 540) % 360) - 180);
    assert.ok(yawErr < 0.05, `${s.role} yaw ${s.yawDeg} enu ${enu}`);
    assert.ok(compassErr > 5, `${s.role} yaw ${s.yawDeg} compass ${compass}`);
    assert.equal(s.position.alt, 25);
  }
  const ring = F.sectorRing(mid.lat, mid.lon, 90, 500, 20, 4);
  assert.equal(ring[0].lat, mid.lat);
  assert.equal(ring[0].lon, mid.lon);
  // ENU yaw 90 faces north. The fan opens from 70° (east of north) to 110° (west of north).
  assert.ok(ring[1].lon > mid.lon);
  assert.ok(ring[ring.length - 1].lon < mid.lon);
  assert.ok(ring[3].lat > mid.lat);
});

test('footprints drop a repeated closing vertex', () => {
  const fp = F.footprintFromLatLonList([
    { lat: 1, lon: 2 },
    { lat: 1, lon: 3 },
    { lat: 2, lon: 3 },
    { lat: 1, lon: 2 },
  ]);
  assert.equal(fp.length, 3);
  assert.deepEqual(fp[0], { lat: 1, lon: 2 });
});

test('demo page shows the sample globe before Overpass and resizes the viewer', () => {
  const app = fs.readFileSync(new URL('../docs/app.js', import.meta.url), 'utf8');
  const html = fs.readFileSync(new URL('../docs/index.html', import.meta.url), 'utf8');
  assert.match(app, /onLoadMap\(\{\s*background:\s*true\s*\}\)/);
  assert.match(app, /corridorFocus/);
  assert.match(app, /viewer\.resize\(/);
  assert.match(app, /viewBoundingSphere/);
  assert.match(app, /new Worker\(/);
  assert.match(app, /worker\.terminate\(/);
  assert.match(app, /withTimeout\(/);
  assert.equal((app.match(/spinner\.classList\.add\(/g) || []).length, 1);
  assert.equal(app.includes('midLat - 0.006'), false);
  assert.match(html, /app\.js\?v=ux3/);
  assert.match(html, /framing\.js\?v=ux3/);
  assert.match(html, /planner\.js\?v=ux3/);
  assert.match(app, /overpass\.openstreetmap\.fr/);
  const fr = app.indexOf('overpass.openstreetmap.fr');
  const de = app.indexOf('overpass-api.de');
  assert.ok(fr > 0 && fr < de, 'healthy mirror is tried before the overloaded one');
  assert.equal(app.includes('Promise.any'), false);
  assert.match(app, /手绘样例/);
  assert.match(app, /sectorRing/);
  assert.match(app, /teachingRadarPositions/);
  assert.equal(app.includes('保留上次建筑'), false);
  assert.match(html, /overpass\.openstreetmap\.fr/);
  assert.match(html, /name="referrer" content="origin"/);
  assert.match(app, /referrerPolicy: 'origin'/);
  assert.match(html, /min-width:\s*0/);
  assert.match(html, /#cesiumContainer\s*\{[^}]*position:\s*absolute/s);
});
