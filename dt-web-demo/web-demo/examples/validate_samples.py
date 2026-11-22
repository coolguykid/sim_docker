"""Validate the supplied reduced mock examples, not a production JSON Schema."""
import json
import math
from datetime import datetime, timedelta
from pathlib import Path

HERE = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def timestamp(value):
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(result.tzinfo is not None, 'Timestamp must include timezone')
    return result


def index(rows, field):
    result = {row[field]: row for row in rows}
    require(len(result) == len(rows), 'Duplicate ' + field)
    require(all(isinstance(key, str) and key for key in result), 'IDs must be nonempty strings')
    return result


def validate(topology_message, frames):
    require(topology_message['schemaVersion'] == 'regional-simulation-draft-0.1', 'Unexpected draft version')
    require(topology_message['type'] == 'topology', 'Expected topology message')
    topo = topology_message['data']['topology']
    meta = topology_message['data']['metadata']
    require(topo['versionId'] == topology_message['topologyVersion'], 'Topology version mismatch')
    require(topo['regionId'] == topology_message['scope']['regionId'], 'Topology region mismatch')
    require(meta['coordinate']['axisOrder'] == ['east', 'north', 'up'], 'Coordinate convention mismatch')
    require(meta['coordinate']['horizontalUnit'] == meta['coordinate']['verticalUnit'] == 'm', 'Expected meter coordinates')
    require(meta['units']['averageSpeed'] == 'm/s', 'Expected speed in m/s')
    layers = index(topo['layers'], 'layerId')
    nodes = index(topo['nodes'], 'nodeId')
    pipes = index(topo['pipes'], 'pipeId')
    ramps = index(topo['ramps'], 'rampId')
    lanes = index(topo['lanes'], 'laneId')
    for node in nodes.values():
        require(node['layerId'] in layers, 'Unknown node layer')
        require(len(node['position']) == 3 and all(map(number, node['position'])), 'Invalid node position')
    for pipe in pipes.values():
        require(pipe['layerId'] in layers, 'Unknown pipe layer')
        require(pipe['startNodeId'] in nodes and pipe['endNodeId'] in nodes, 'Unknown pipe endpoint')
        require(number(pipe['length']) and pipe['length'] > 0, 'Invalid pipe length')
    for ramp in ramps.values():
        require(ramp['startNodeId'] in nodes and ramp['endNodeId'] in nodes, 'Unknown ramp endpoint')
        path = ramp['path']
        require(len(path) >= 2 and all(len(point) == 3 and all(map(number, point)) for point in path), 'Invalid ramp path')
        require(path[0] == nodes[ramp['startNodeId']]['position'] and path[-1] == nodes[ramp['endNodeId']]['position'], 'Ramp endpoints do not match geometry')
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        require(math.isclose(length, ramp['length'], abs_tol=1e-6), 'Ramp length mismatch')
    for lane in lanes.values():
        require(lane['pipeId'] in pipes, 'Unknown lane pipe')
    require(len(frames) >= 3, 'Need at least 3 output frames')
    prior_seq = prior_step = prior_time = -1
    seen_snapshots = set()
    for frame in frames:
        for key in ('schemaVersion', 'runId', 'streamId', 'scope', 'topologyVersion'):
            require(frame[key] == topology_message[key], 'Frame scope/version/run mismatch: ' + key)
        require(frame['type'] == 'snapshot', 'Expected full snapshot')
        require(type(frame['seq']) is int and frame['seq'] > prior_seq, 'Sequence must increase')
        require(type(frame['step']) is int and frame['step'] > prior_step, 'Step must increase')
        seconds = frame['simulationTimeSeconds']
        require(number(seconds) and seconds > prior_time, 'Simulation time must increase')
        require(math.isclose(seconds, frame['step'] * meta['simulationClock']['stepDurationSeconds']), 'Step and simulation time mismatch')
        expected = timestamp(meta['simulationClock']['startTime']) + timedelta(seconds=seconds)
        require(timestamp(frame['simulationTime']) == expected, 'ISO simulation time mismatch')
        require(timestamp(frame['sampleTime']) == expected, 'Sample clock mismatch')
        timestamp(frame['serverTime'])
        require(frame['snapshotId'] not in seen_snapshots, 'Duplicate snapshot ID')
        seen_snapshots.add(frame['snapshotId'])
        require(frame['quality']['source'] == 'mock', 'These example files must remain labeled mock')
        state = frame['data']['state']
        pstate = index(state['pipes'], 'pipeId')
        rstate = index(state['ramps'], 'rampId')
        require(pstate.keys() == pipes.keys() and rstate.keys() == ramps.keys(), 'Full state coverage mismatch')
        uavs = index(state['uavs'], 'uavId')
        coverage = state['uavCoverage']
        require(coverage['mode'] == 'full' and coverage['scope'] == 'region', 'Example coverage must be full region')
        require(coverage['total'] == coverage['returned'] == len(uavs), 'UAV coverage count mismatch')
        groups = {('pipe', key): [] for key in pipes} | {('ramp', key): [] for key in ramps}
        for uav in uavs.values():
            require(uav['regionId'] == topo['regionId'], 'Unknown UAV region')
            require(len(uav['position']) == 3 and all(map(number, uav['position'])), 'Invalid UAV position')
            require(number(uav['speed']) and uav['speed'] >= 0, 'Invalid UAV speed')
            key = (uav['location']['type'], uav['location']['objectId'])
            require(key in groups and uav['phase'] == key[0], 'Unknown or mismatched UAV location')
            if uav['laneId'] is not None:
                require(key[0] == 'pipe' and uav['laneId'] in lanes and lanes[uav['laneId']]['pipeId'] == key[1], 'Invalid UAV lane')
            groups[key].append(uav)
        for kind, rows in (('pipe', pstate), ('ramp', rstate)):
            for key, row in rows.items():
                values = groups[(kind, key)]
                count = row['uavNumber']
                require(type(count) is int and count >= 0 and count == len(values), 'Object count mismatch')
                require(type(row['isOpen']) is bool, 'Invalid open state')
                mean = row['averageSpeed']
                require((mean is None) if not values else number(mean) and math.isclose(mean, sum(u['speed'] for u in values) / len(values)), 'Object speed mean mismatch')
                if kind == 'pipe':
                    require(all(row[field] is None for field in ('flow', 'density', 'maxCapacity', 'speedLimit', 'safeDistance')), 'Unsupported example metric must be null')
        prior_seq, prior_step, prior_time = frame['seq'], frame['step'], seconds
    return {'frames': len(frames), 'layers': len(layers), 'pipes': len(pipes), 'ramps': len(ramps), 'uavsPerFrame': len(frames[0]['data']['state']['uavs'])}


if __name__ == '__main__':
    topology = json.loads((HERE / 'regional-topology.example.json').read_text(encoding='utf-8'))
    snapshots = json.loads((HERE / 'regional-frames.example.json').read_text(encoding='utf-8'))
    print('PASS:', json.dumps(validate(topology, snapshots), ensure_ascii=False))