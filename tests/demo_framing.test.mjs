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
  assert.match(html, /app\.js\?v=/);
  assert.match(html, /framing\.js\?v=/);
  assert.match(html, /min-width:\s*0/);
  assert.match(html, /#cesiumContainer\s*\{[^}]*position:\s*absolute/s);
});
