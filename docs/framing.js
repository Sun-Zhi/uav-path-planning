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

  return {
    verticalFovRad: verticalFovRad,
    enuDelta: enuDelta,
    pitchDownToTargetDeg: pitchDownToTargetDeg,
    targetInFrustum: targetInFrustum,
    footprintFromLatLonList: footprintFromLatLonList,
    corridorFocus: corridorFocus,
  };
});
