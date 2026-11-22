# Metropolis 36 First-Version Enhancement Implementation Plan

> For agentic workers: use subagent-driven-development for isolated tasks. Run spec review before quality review; keep all mutations in this delivery directory. The user explicitly approved the updated design on 2026-11-01; no further design approval is needed.

Goal: extend the first-version application, preserve its configuration panel, and verify six genuine 10,000-UAV runs.

Architecture: original Airspace/Pipeline/Ramp geometry gains regional segment views; original UAV/route-server/actor/coordinator/control classes gain versioned regional routing and fixed-second prepare/commit motion. NumPy batches and spatial indexes accelerate the existing control path. The original app, preview, output and optional pickup interfaces stay in place.

Tech Stack: Python 3.9, NumPy, PyQt5, Matplotlib, Ray, pytest, standard-library JSON.

## Task 1: isolate and define shared interfaces

- [x] Copy D:\无人机_仅算法1 without caches, histories, or Git; hash the baseline.
- [x] Add `airspace/partition_contracts.py` with immutable directed `RoadNode`, `RoadEdge`, `RegionMap`, `PortalSummary`, `Destination`, and `RegionPlan` records. Preserve original pipe/ramp identifiers and lane endpoint offsets.
- [x] Use seconds throughout: `context={step_index,sim_time_seconds,sim_dt_seconds,topology_version}`; workers reject stale commits and mismatched dt.

## Task 2: original geometry + region compiler and routing

Owner files: airspace/config.py, environment.py, ramp_topology.py; new partition_graph.py and partition_router.py; tests/enhanced/test_partition*.py.

- [x] Write failing tests for 300 original pipes, 20 lanes, 1000 unique bottom slots, 36 ownership tiles, conserved geometry/direction and multi-region ramps.
- [x] Extend metropolis generation in the original Airspace class; generate one world, then compile region views, never 36 separately generated worlds.
- [x] Build generic versioned portal reachability costs and current-region bundles. Route using complete portal Dijkstra plus local Dijkstra; no three-portal truncation or spatial-rounding cache.
- [x] Implement `PartitionMaps.region(id)`, `.summary()`, `.destination(node_id)` and `RegionalDijkstra(summary,region_map).plan(start_node_id,destination)`; only local detailed edges in local results.
- [x] Verify shortest costs against a test-only full-graph oracle; retain geometry regressions.

## Task 3: exact motion and continuous safety primitives

Owner files: new algorithms/continuous_motion.py, algorithms/continuous_safety.py; tests/enhanced/test_continuous_*.py.

- [x] First add RED analytic cases: acceleration, cap event, stopping event, short-edge remainder, fractional spawn, swept overtaking and crossing.
- [x] Implement `integrate_motion(v,a,dt,cap)` and `time_to_distance(v,a,d,cap)` with exact phase boundaries. Accept float seconds, reject non-finite inputs and physically inconsistent initial speed.
- [x] Implement `TrajectoryPiece(uid,t0,t1,p0,v0,a)` and continuous 3D closest distance, including repeated roots, plus swept spatial broad phase.
- [x] Implement batch endpoint/piece audit supporting per-vehicle safety distances, separate body-contact metrics, and no all-pairs global root solves.
- [x] Run focused tests and seeded numeric oracle comparisons.

## Task 4: preserve and extend original parameter panel

Owner: ui/gui_config.py only; tests/enhanced/test_gui_metropolis.py.

- [x] Retain ConfigGUI(QMainWindow), AirspaceCanvas, preview, capacity/pickup, old modes, trace/log/export/database and Ray controls.
- [x] Add routing mode, dt, replenishment period/limit, fixed/per-batch destinations, seed, pipe/lane and ramp quotas. Default metropolis, pickup=0, target=10000, batch<=1000, period=100 seconds, destination_count=10, destination_policy=fixed.
- [x] Display physical seconds and derived step count. Preview original 300 pipes and random internal targets; preserve hand-picked destinations.
- [x] Test config extraction, mode switches, default values, validation and render screenshots.

## Task 5: original runtime integration

Owner: entities/uav.py, app.py, simulation/runner.py, distributed/{messages,route_planning_server,local_env_proxy,uav_actor,orchestrator,authoritative_control_server}.py; small scheduler/map transport helpers where useful.

- [x] Add failing tests for exact destination, no-route status, all three routing data boundaries and prepare/commit rollback.
- [x] Centralized-full plans all regional prefixes at dispatch; centralized-regional computes only the current prefix; onboard-regional computes in real Ray worker from summary/current map only.
- [x] Extend UAV with exact local-edge progress, continuous ramp XYZ, target lane/position, region event handling and safe lower-speed entry. Use batch arithmetic in groups where it preserves the same equations.
- [x] Extend AuthoritativeControlCore/safety_shield for lane/ramp reservations, near-neighbor proposals and continuous geometry. Resolve candidate conflicts before committing, retain bounded acceleration, record failed/rejected intervals honestly.
- [x] Wire integer-decisecond clock, initial t=0 frame, per-step dt timestamps and output lifecycle through the original orchestrator and runner.

## Task 6: replenishment and shared destinations

- [x] Write RED tests for t=0 batch, period ticks at both dt values, 1000 per batch, active+pending<=10000, monotonic IDs, fixed pool and per-batch reproducibility.
- [x] Default fixed pool: sample 10 distinct reachable sixth-layer interior lane coordinates once; share across all batches. Store seed and actual sampled coordinates.
- [x] Issue only safe launch slots, preserve pending reservations, release completed actor state, stream completed metrics and selected trajectories.
- [x] Stop replenishment at configured horizon, drain all released tasks, report unfinished tasks independently from successful arrivals.

## Task 7: verification and delivery

- [x] Run retained baseline regressions plus enhanced tests using D:\Anaconda3\envs\hxy2\python.exe -B -m pytest -p no:cacheprovider.
- [x] Spec review, then quality review; fix every concrete important finding and rerun corresponding tests.
- [x] Per the user's updated scope on 2026-11-02, run all three actual routing modes with dt=1 and .1 using 120 UAVs per case, four 30-UAV batches at 100-second intervals, the full default metropolis topology, exact goal checks and independent sampled safety. All six cases passed; retain three completed 10000-task runs separately.
- [x] Record actual airborne peak without counting ground queues. The three completed 10000-task runs peaked at 2122 airborne; no claim of 10000 genuinely simultaneous UAVs. Remaining full-load and shorter-interval stress runs are no longer required under the latest user instruction.
- [x] Include actual Ray workers (four per small onboard case), 3208 private-map scope checks, 500 independent whole-graph routing comparisons, source-path review, first-version diff manifest, 124 baseline SHA checks, 74 frozen-source checks and the plain-Chinese report.

All tests execute in the new directory. The original package and database fixtures remain, and no Git branch/merge step is needed for this standalone folder delivery.
