/* REGION_DB_TOPOLOGY
 * Region-1 database topology loader used by the first Docker integration test.
 * It only reads GET /api/v1/topology. It does not send guidance or start DT.
 */
window.RegionDbTopology = (() => {
  'use strict';

  // REGION_DB_TOPOLOGY: the page keeps 36 selectable regions, but only the
  // 35 Docker regional services currently exist; R36 stays on the offline demo.
  const endpoints = Object.freeze(Object.fromEntries(
    Array.from({length: 35}, (_, index) => {
      const regionId = index + 1;
      return [regionId, `http://127.0.0.1:${18100 + regionId}`];
    })
  ));
  const cache = new Map();

  function validate(data, expectedRegionId) {
    if (!data || data.schemaVersion !== 'regional-topology-1.0') {
      throw Error('区域路网接口版本不受支持');
    }
    if (String(data.regionId) !== String(expectedRegionId)) {
      throw Error(`接口返回区域${data.regionId}，期望区域${expectedRegionId}`);
    }
    for (const key of ['layers', 'nodes', 'pipes', 'ramps', 'lanes']) {
      if (!Array.isArray(data[key])) throw Error(`路网字段${key}不是数组`);
    }
    const nodeIds = new Set(data.nodes.map(node => String(node.nodeId)));
    for (const pipe of data.pipes) {
      if (!nodeIds.has(String(pipe.startNodeId)) || !nodeIds.has(String(pipe.endNodeId))) {
        throw Error(`管道${pipe.pipeId}引用了不存在的节点`);
      }
    }
    return data;
  }

  function adapt(data) {
    const nodes = new Map(data.nodes.map(node => [String(node.nodeId), {
      ...node,
      nodeId: String(node.nodeId),
      position: node.position.map(Number)
    }]));
    const pipes = data.pipes.map(pipe => ({
      ...pipe,
      key: `pipe:${pipe.pipeId}`,
      points: [nodes.get(String(pipe.startNodeId)).position, nodes.get(String(pipe.endNodeId)).position]
    }));
    const ramps = data.ramps.map(ramp => ({
      ...ramp,
      key: `ramp:${ramp.rampId}`,
      points: ramp.path.map(point => point.map(Number))
    }));
    const lanes = data.lanes.map(lane => ({
      ...lane,
      key: `lane:${lane.laneId}`,
      points: [lane.startPoint.map(Number), lane.endPoint.map(Number)]
    }));
    // REGION_DB_TOPOLOGY: classify each type=2 connected component from the
    // database's directed node semantics. nodeType 1 starts a takeoff/inflow
    // guide component; nodeType 2 terminates a landing/outflow component.
    const funnelRamps = ramps.filter(ramp => ramp.type === 'funnel');
    const adjacency = new Map();
    for (const ramp of funnelRamps) {
      for (const [a, b] of [[String(ramp.startNodeId), String(ramp.endNodeId)], [String(ramp.endNodeId), String(ramp.startNodeId)]]) {
        if (!adjacency.has(a)) adjacency.set(a, []);
        adjacency.get(a).push(b);
      }
    }
    const componentKind = new Map(), visited = new Set();
    for (const seed of adjacency.keys()) {
      if (visited.has(seed)) continue;
      const stack = [seed], component = [];let kind = null;
      while (stack.length) {
        const id = stack.pop();if (visited.has(id)) continue;visited.add(id);component.push(id);
        const type = nodes.get(id)?.nodeType;if (type === 1) kind = 'takeoff';if (type === 2) kind = 'landing';
        stack.push(...(adjacency.get(id) || []));
      }
      for (const id of component) componentKind.set(id, kind);
    }
    for (const ramp of funnelRamps) ramp.funnelFlow = componentKind.get(String(ramp.startNodeId)) || componentKind.get(String(ramp.endNodeId));
    const funnels = [...nodes.values()].filter(node => node.nodeType === 1 || node.nodeType === 2).map(node => {
      const ramp = funnelRamps.find(value => String(value.startNodeId) === node.nodeId || String(value.endNodeId) === node.nodeId);
      const otherId = ramp ? (String(ramp.startNodeId) === node.nodeId ? String(ramp.endNodeId) : String(ramp.startNodeId)) : null;
      return {key: `funnel:${node.nodeId}`, nodeId: node.nodeId, type: node.nodeType === 1 ? 'takeoff' : 'landing', bottom: node.position, top: nodes.get(otherId)?.position || node.position, rampId: ramp?.rampId || null};
    });
    const objects = new Map([
      ...pipes.map(pipe => [pipe.key, pipe]),
      ...ramps.map(ramp => [ramp.key, ramp]),
      ...funnels.map(funnel => [funnel.key, funnel])
    ]);
    return {
      ...data,
      nodes,
      pipes,
      ramps,
      lanes,
      funnels,
      objects,
      portals: [...nodes.values()].filter(node => node.isPortal)
    };
  }

  async function load(regionId, {reload = false} = {}) {
    const endpoint = endpoints[regionId];
    if (!endpoint) return null; // Region 36 retains the offline demo for now.
    if (!reload && cache.has(regionId)) return cache.get(regionId);
    const promise = fetch(`${endpoint}/api/v1/topology`, {
      headers: {'Accept': 'application/json'}
    }).then(async response => {
      if (!response.ok) throw Error(`HTTP ${response.status}: ${await response.text()}`);
      return adapt(validate(await response.json(), regionId));
    });
    cache.set(regionId, promise);
    try {
      return await promise;
    } catch (error) {
      cache.delete(regionId);
      throw error;
    }
  }

  return {load, validate, adapt, endpoints};
})();
