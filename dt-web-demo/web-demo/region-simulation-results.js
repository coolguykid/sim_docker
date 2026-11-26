/* REGION_DT_SIMULATION_RESULT
 * Frame-sized adapter for regional DT CSV artifacts exposed by each Docker.
 * It does not load a whole CSV into the browser and does not start simulation.
 */
window.RegionSimulationResults = (() => {
  'use strict';
  const endpoint = regionId => RegionDbTopology.endpoints[regionId];
  async function json(url) {
    const response = await fetch(url, {headers: {'Accept': 'application/json'}});
    if (!response.ok) throw Error(`HTTP ${response.status}: ${await response.text()}`);
    return response.json();
  }
  async function latest(regionId) {
    const base = endpoint(regionId);if (!base) return null;
    return json(`${base}/api/v1/simulation-results/latest`);
  }
  async function snapshot(regionId, runId, attemptId, time) {
    const base = endpoint(regionId);if (!base) return null;
    const data = await json(`${base}/api/v1/simulation-results/${encodeURIComponent(runId)}/attempts/${attemptId}/snapshot?time=${encodeURIComponent(time)}`);
    data.statisticsMap = new Map(data.statistics.map(row => [`${row.edgeType}:${row.edgeId}`, row]));
    return data;
  }
  function adaptSnapshot(data) {
    data.statisticsMap = new Map(data.statistics.map(row => [`${row.edgeType}:${row.edgeId}`, row]));
    return data;
  }
  function stream(regionId, {runId=null, attemptId=null, after=null, onRun, onSnapshot, onError}={}) {
    const base=endpoint(regionId);if(!base)return null;
    const query=new URLSearchParams();
    if(runId!=null&&attemptId!=null){query.set('run_id',runId);query.set('attempt_id',attemptId);}
    if(after!=null)query.set('after',after);
    const source=new EventSource(`${base}/api/v1/simulation-results/stream?${query}`);
    source.addEventListener('run',event=>onRun?.(JSON.parse(event.data)));
    source.addEventListener('snapshot',event=>onSnapshot?.(adaptSnapshot(JSON.parse(event.data))));
    source.onerror=event=>onError?.(event);
    return source;
  }
  async function replays(regionId) {
    const base = endpoint(regionId);
    return base ? json(base + '/api/v1/simulation-results/replays') : [];
  }

  async function topology(regionId, {runId, attemptId}) {
    const data = await json(endpoint(regionId) + '/api/v1/simulation-results/' +
      encodeURIComponent(runId) + '/attempts/' + attemptId + '/topology');
    return RegionDbTopology.adapt(RegionDbTopology.validate(data, regionId));
  }

  return {latest, snapshot, stream, adaptSnapshot, replays, topology};
})();
