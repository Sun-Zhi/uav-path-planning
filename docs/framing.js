/* Pure camera-framing helpers for the Cesium demo. No Cesium dependency. */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.DemoFraming = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const M_PER_DEG = 111320;

  // Cesium PerspectiveFrustum: fov is horizontal when width >= height, else vertical.
  function verticalFovRad(fovRad, aspect) {
    if (!(aspect > 0)) return fovRad;
    if (aspect <= 1) return fovRad;
    return 2 * Math.atan(Math.tan(fovRad * 0.5) / aspect);
  }

  function enuDelta(from, to) {
    const midLat = ((from.lat + to.lat) / 2) * Math.PI / 180;
    return {
      east: (to.lon - from.lon) * M_PER_DEG * Math.cos(midLat),
      north: (to.lat - from.lat) * M_PER_DEG,
    };
  }

  // Pitch of the sightline from camera to target, degrees, negative looking down.
  function pitchDownToTargetDeg(camera, target) {
    const d = enuDelta(camera, target);
    const horiz = Math.hypot(d.east, d.north);
    const drop = (camera.alt || 0) - (target.alt || 0);
    return -Math.atan2(Math.max(drop, 0), Math.max(horiz, 1e-6)) * 180 / Math.PI;
  }

  function targetInFrustum(cameraPitchDeg, targetPitchDeg, fovyDeg) {
    const half = fovyDeg / 2;
    return targetPitchDeg >= cameraPitchDeg - half - 1e-6 &&
      targetPitchDeg <= cameraPitchDeg + half + 1e-6;
  }

  // Planner yaw: 0 = east (+X), 90 = north (+Y). Compass bearing is the other way around.
  function enuYawDeg(from, to) {
    const d = enuDelta(from, to);
    return (Math.atan2(d.north, d.east) * 180 / Math.PI + 360) % 360;
  }

  function compassBearingDeg(from, to) {
    const d = enuDelta(from, to);
    return (Math.atan2(d.east, d.north) * 180 / Math.PI + 360) % 360;
  }

  function corridorFrame(start, goal) {
    const midLat = (start.lat + goal.lat) / 2;
    const midLon = (start.lon + goal.lon) / 2;
    const d = enuDelta(start, goal);
    const len = Math.hypot(d.east, d.north) || 1;
    const ux = d.east / len;
    const uy = d.north / len;
    return {
      midLat: midLat,
      midLon: midLon,
      ux: ux,
      uy: uy,
      px: -uy,
      py: ux,
      mlat: M_PER_DEG,
      mlon: M_PER_DEG * Math.cos(midLat * Math.PI / 180),
    };
  }

  // Side-looking gates across the start/goal corridor. sideSign picks which flank.
  const TEACHING_LAYOUT = [
    { role: 'us', sideSign: -1, sideM: 360, alongM: -40, alt: 25, pitchDeg: 12 },
    { role: 'cn', sideSign: 1, sideM: 400, alongM: 30, alt: 25, pitchDeg: 11 },
  ];

  function teachingRadarPositions(start, goal) {
    const f = corridorFrame(start, goal);
    return TEACHING_LAYOUT.map((spec) => {
      const east = spec.sideSign * f.px * spec.sideM + f.ux * spec.alongM;
      const north = spec.sideSign * f.py * spec.sideM + f.uy * spec.alongM;
      const position = {
        lat: f.midLat + north / f.mlat,
        lon: f.midLon + east / f.mlon,
        alt: spec.alt,
      };
      return {
        role: spec.role,
        position: position,
        yawDeg: enuYawDeg(position, { lat: f.midLat, lon: f.midLon }),
        pitchDeg: spec.pitchDeg,
      };
    });
  }

  // Closed ring: site, then the arc. yawDeg is ENU (0 = east). halfAngleDeg is half the drawn fan.
  function sectorRing(lat, lon, yawDeg, radiusM, halfAngleDeg, steps) {
    const n = steps || 18;
    const mlat = M_PER_DEG;
    const mlon = M_PER_DEG * Math.cos((lat * Math.PI) / 180);
    const ring = [{ lat: lat, lon: lon }];
    const half = Math.max(1, halfAngleDeg);
    for (let i = 0; i <= n; i++) {
      const a = ((yawDeg - half + (2 * half * i) / n) * Math.PI) / 180;
      const east = radiusM * Math.cos(a);
      const north = radiusM * Math.sin(a);
      ring.push({ lat: lat + north / mlat, lon: lon + east / mlon });
    }
    return ring;
  }

  function footprintFromLatLonList(list) {
    const fp = [];
    for (const n of list || []) {
      if (n == null || n.lat == null || n.lon == null) continue;
      const lat = Number(n.lat);
      const lon = Number(n.lon);
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) continue;
      const last = fp[fp.length - 1];
      if (last && last.lat === lat && last.lon === lon) continue;
      fp.push({ lat: lat, lon: lon });
    }
    if (fp.length >= 2) {
      const a = fp[0];
      const b = fp[fp.length - 1];
      if (a.lat === b.lat && a.lon === b.lon) fp.pop();
    }
    return fp;
  }

  // Oblique look-at for the start/goal corridor. Pitch -32° (not nadir).
  // Range is filled in by Cesium from the current aspect so the sphere stays on screen.
  function corridorFocus(map) {
    const s = map.start;
    const g = map.goal;
    const midLat = (s.lat + g.lat) / 2;
    const midLon = (s.lon + g.lon) / 2;
    const d = enuDelta(s, g);
    const half = Math.hypot(d.east, d.north) / 2;
    const low = Math.min(s.alt || 0, g.alt || 0);
    return {
      lat: midLat,
      lon: midLon,
      alt: Math.max(40, low * 0.45),
      radius: Math.max(280, half + 200),
      headingDeg: 215,
      pitchDeg: -32,
    };
  }

  // viewBoundingSphere/lookAt write the local offset and leave positionWC stale.
  // lookAtTransform copies that cached world pose, so an unlock on the first
  // frame restores Cesium's default Americas camera. Reading positionWC runs
  // the camera update first and keeps the pose we just set.
  function unlockCamera(camera, identityTransform) {
    if (!camera || typeof camera.lookAtTransform !== 'function') return;
    void camera.positionWC;
    camera.lookAtTransform(identityTransform);
  }

  return {
    verticalFovRad: verticalFovRad,
    enuDelta: enuDelta,
    pitchDownToTargetDeg: pitchDownToTargetDeg,
    targetInFrustum: targetInFrustum,
    footprintFromLatLonList: footprintFromLatLonList,
    corridorFocus: corridorFocus,
    enuYawDeg: enuYawDeg,
    compassBearingDeg: compassBearingDeg,
    teachingRadarPositions: teachingRadarPositions,
    sectorRing: sectorRing,
    unlockCamera: unlockCamera,
  };
});
