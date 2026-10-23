/* Plans off the UI thread so a long WASM call cannot freeze the page. */
importScripts('planner.js?v=rrt-scale1');

let modulePromise = null;

self.onmessage = async (ev) => {
  const data = ev.data || {};
  const id = data.id;
  try {
    if (!modulePromise) {
      if (typeof createPlanner !== 'function') {
        throw new Error('createPlanner not found');
      }
      modulePromise = createPlanner();
    }
    const mod = await modulePromise;
    const raw = mod.plan(String(data.mapJson || ''), String(data.algo || 'astar'), !!data.informed);
    self.postMessage({ id: id, raw: raw });
  } catch (e) {
    modulePromise = null;
    self.postMessage({ id: id, error: (e && e.message) ? e.message : String(e) });
  }
};
