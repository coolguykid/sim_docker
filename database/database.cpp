#include <pqxx/pqxx>

#include "database.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {

std::vector<std::vector<double>> parse2dPgArray(const std::string& s) {
    std::vector<std::vector<double>> result;
    if (s.empty() || s == "{}") return result;
    std::size_t i = 0;
    if (s[i] == '{') ++i;
    while (i < s.size()) {
        if (s[i] != '{') { ++i; continue; }
        const std::size_t begin = ++i;
        while (i < s.size() && s[i] != '}') ++i;
        if (i >= s.size()) break;
        std::string inner = s.substr(begin, i - begin);
        std::replace(inner.begin(), inner.end(), ',', ' ');
        std::istringstream ss(inner);
        std::vector<double> row;
        double v{};
        while (ss >> v) row.push_back(v);
        result.push_back(std::move(row));
        ++i;
    }
    return result;
}


void validateVec3(const Vec& v, const char* name) {
    if (v.size() != 3) {
        throw std::invalid_argument(std::string(name) + " must contain exactly 3 values");
    }
}

void validateUavState(const UavState& d) {
    validateVec3(d.position, "UavState.position");

    if (!std::isfinite(d.speed) || d.speed < 0.0) {
        throw std::invalid_argument("UavState.speed must be finite and >= 0");
    }
    if (d.has_acceleration && !std::isfinite(d.acceleration)) {
        throw std::invalid_argument("UavState.acceleration must be finite when present");
    }
    if (d.distance_on_edge != INVALID_DISTANCE &&
        (!std::isfinite(d.distance_on_edge) || d.distance_on_edge < 0.0)) {
        throw std::invalid_argument(
            "UavState.distance_on_edge must be -1 or a finite value >= 0");
    }

    const bool has_pipe = d.pipe_id != INVALID_PIPE_ID;
    const bool has_lane = d.lane_id != INVALID_LANE_ID;
    const bool has_ramp = d.ramp_id != INVALID_RAMP_ID;
    const bool has_node = d.node_id != INVALID_NODE_ID;

    switch (d.edge_type) {
        case EdgeType::Pipe:
            if (!has_pipe || !has_lane || has_ramp || has_node) {
                throw std::invalid_argument(
                    "Pipe UavState requires pipe_id/lane_id and forbids ramp_id/node_id");
            }
            if (d.distance_on_edge == INVALID_DISTANCE) {
                throw std::invalid_argument(
                    "Pipe UavState requires distance_on_edge >= 0");
            }
            break;

        case EdgeType::Ramp:
            if (has_pipe || has_lane || !has_ramp || has_node) {
                throw std::invalid_argument(
                    "Ramp UavState requires ramp_id and forbids pipe_id/lane_id/node_id");
            }
            if (d.distance_on_edge == INVALID_DISTANCE) {
                throw std::invalid_argument(
                    "Ramp UavState requires distance_on_edge >= 0");
            }
            break;

        case EdgeType::Node:
            if (has_pipe || has_lane || has_ramp || !has_node) {
                throw std::invalid_argument(
                    "Node UavState requires node_id and forbids pipe_id/lane_id/ramp_id");
            }
            if (d.distance_on_edge != INVALID_DISTANCE) {
                throw std::invalid_argument(
                    "Node UavState requires distance_on_edge = -1");
            }
            break;

        default:
            throw std::invalid_argument("invalid UavState.edge_type");
    }
}

std::string build2dPgArray(const Matrix& m) {
    if (m.empty()) return "{}";
    std::ostringstream ss;
    ss << '{';
    for (std::size_t i = 0; i < m.size(); ++i) {
        if (i) ss << ',';
        ss << '{';
        for (std::size_t j = 0; j < m[i].size(); ++j) {
            if (j) ss << ',';
            ss << std::setprecision(17) << m[i][j];
        }
        ss << '}';
    }
    ss << '}';
    return ss.str();
}

std::string databaseNow(pqxx::transaction_base& txn) {
    auto r = txn.exec("SELECT CURRENT_TIMESTAMP::text AS now");
    return r[0]["now"].as<std::string>();
}

std::string resolveTime(pqxx::transaction_base& txn, const std::string& t) {
    return t.empty() ? databaseNow(txn) : t;
}

RegionId localRegionId(pqxx::transaction_base& txn) {
    auto r = txn.exec("SELECT region_id FROM region_info ORDER BY region_id LIMIT 1");
    if (r.empty()) throw std::runtime_error("region_info is empty");
    return r[0]["region_id"].as<RegionId>();
}

struct PooledConn {
    std::unique_ptr<pqxx::connection> conn;
    bool in_use{false};
};

class ConnectionPool {
public:
    explicit ConnectionPool(std::size_t max_per_conn_str = 8, int wait_ms = 5000)
        : max_per_conn_str_(max_per_conn_str), wait_ms_(wait_ms) {}

    std::shared_ptr<PooledConn> acquire(const std::string& conn_str) {
        std::unique_lock<std::mutex> lock(mutex_);
        auto& queue = pools_[conn_str];
        for (auto& p : queue) {
            if (!p->in_use) {
                p->in_use = true;
                if (!p->conn->is_open()) p->conn = std::make_unique<pqxx::connection>(conn_str);
                return p;
            }
        }
        if (queue.size() < max_per_conn_str_) {
            auto p = std::make_shared<PooledConn>();
            p->conn = std::make_unique<pqxx::connection>(conn_str);
            p->in_use = true;
            queue.push_back(p);
            return p;
        }
        const bool ready = cv_.wait_for(lock, std::chrono::milliseconds(wait_ms_), [&] {
            for (const auto& p : queue) if (!p->in_use) return true;
            return false;
        });
        if (!ready) throw std::runtime_error("timeout acquiring PostgreSQL connection");
        for (auto& p : queue) {
            if (!p->in_use) {
                p->in_use = true;
                return p;
            }
        }
        throw std::runtime_error("connection pool internal error");
    }

    void release(const std::string&, const std::shared_ptr<PooledConn>& p) {
        std::lock_guard<std::mutex> lock(mutex_);
        p->in_use = false;
        cv_.notify_one();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::unordered_map<std::string, std::deque<std::shared_ptr<PooledConn>>> pools_;
    std::size_t max_per_conn_str_;
    int wait_ms_;
};

ConnectionPool& globalPool() {
    static ConnectionPool pool;
    return pool;
}

TopologyVersion topologyVersionAtTxn(
    pqxx::transaction_base& txn, RegionId region_id, const std::string& observation_time) {
    const std::string t = resolveTime(txn, observation_time);
    auto r = txn.exec_params(R"SQL(
        SELECT region_id, version_id, valid_from::text, valid_to::text, reason
        FROM topology_version
        WHERE region_id=$1
          AND valid_from <= $2::timestamptz
          AND (valid_to IS NULL OR $2::timestamptz < valid_to)
        ORDER BY valid_from DESC
        LIMIT 1
    )SQL", region_id, t);
    if (r.empty()) throw std::runtime_error("no topology version active at observation time");
    TopologyVersion v;
    v.region_id = r[0]["region_id"].as<RegionId>();
    v.version_id = r[0]["version_id"].as<VersionId>();
    v.valid_from = r[0]["valid_from"].as<std::string>();
    if (r[0]["valid_to"].is_null()) {
        v.valid_to = "";
    } else {
        v.valid_to = r[0]["valid_to"].as<std::string>();
    }
    if (r[0]["reason"].is_null()) {
        v.reason = "";
    } else {
        v.reason = r[0]["reason"].as<std::string>();
    }
    return v;
}

std::vector<std::string> recentSampleTimes(pqxx::transaction_base& txn, RegionId region_id, int steps) {
    if (steps <= 0) return {};
    // Use both traffic samples and topology-state changes as observation timestamps.
    // This preserves replay steps even when a pipe/ramp switch changes between two flow samples.
    auto r = txn.exec_params(R"SQL(
        SELECT time::text
        FROM (
            SELECT time FROM pipe_flow   WHERE region_id=$1
            UNION
            SELECT time FROM ramp_flow   WHERE region_id=$1
            UNION
            SELECT time FROM lane_flow   WHERE region_id=$1
            UNION
            SELECT time FROM pipe_status WHERE region_id=$1
            UNION
            SELECT time FROM ramp_status WHERE region_id=$1
        ) t
        ORDER BY time DESC
        LIMIT $2
    )SQL", region_id, steps);
    std::vector<std::string> times;
    times.reserve(r.size());
    for (const auto& row : r) times.push_back(row[std::size_t{0}].as<std::string>());
    std::reverse(times.begin(), times.end());
    return times;
}

std::vector<std::string> fixedGridTimes(
    pqxx::transaction_base& txn, const TimeSeriesQuery& query) {
    if (query.steps <= 0) return {};
    if (query.step_seconds <= 0) {
        throw std::invalid_argument("TimeSeriesQuery.step_seconds must be > 0");
    }
    const std::string end_time = resolveTime(txn, query.end_time);
    auto r = txn.exec_params(R"SQL(
        SELECT gs::text AS time
        FROM generate_series(
            $1::timestamptz - (($2::integer - 1) * make_interval(secs => $3::integer)),
            $1::timestamptz,
            make_interval(secs => $3::integer)
        ) AS gs
        ORDER BY gs
    )SQL", end_time, query.steps, query.step_seconds);
    std::vector<std::string> times;
    times.reserve(r.size());
    for (const auto& row : r) times.push_back(row["time"].as<std::string>());
    return times;
}

struct MatrixEdgeRecord {
    EdgeKey key;
    NodeId start{};
    NodeId end{};
    bool is_open{true};
    double length{};
    std::int16_t type{};
};

std::vector<MatrixEdgeRecord> matrixEdgesAt(
    pqxx::transaction_base& txn,
    RegionId region_id,
    VersionId version_id,
    const std::string& t) {

    auto pipe_rows = txn.exec_params(R"SQL(
        SELECT p.pipe_id, p.start_node_id, p.end_node_id, p.length, p.type,
               COALESCE(s.is_open, FALSE) AS is_open
        FROM pipe_info p
        LEFT JOIN LATERAL (
            SELECT is_open
            FROM pipe_status ps
            WHERE ps.region_id=p.region_id AND ps.version_id=p.version_id
              AND ps.pipe_id=p.pipe_id AND ps.time <= $3::timestamptz
            ORDER BY ps.time DESC LIMIT 1
        ) s ON TRUE
        WHERE p.region_id=$1 AND p.version_id=$2
        ORDER BY p.pipe_id
    )SQL", region_id, version_id, t);

    auto ramp_rows = txn.exec_params(R"SQL(
        SELECT r.ramp_id, r.start_node_id, r.end_node_id, r.length, r.type,
               COALESCE(s.is_open, FALSE) AS is_open
        FROM ramp_info r
        LEFT JOIN LATERAL (
            SELECT is_open
            FROM ramp_status rs
            WHERE rs.region_id=r.region_id AND rs.version_id=r.version_id
              AND rs.ramp_id=r.ramp_id AND rs.time <= $3::timestamptz
            ORDER BY rs.time DESC LIMIT 1
        ) s ON TRUE
        WHERE r.region_id=$1 AND r.version_id=$2
        ORDER BY r.ramp_id
    )SQL", region_id, version_id, t);

    std::vector<MatrixEdgeRecord> edges;
    edges.reserve(pipe_rows.size() + ramp_rows.size());
    for (const auto& row : pipe_rows) {
        MatrixEdgeRecord e;
        e.key = {GraphEdgeKind::Pipe, region_id, version_id, row["pipe_id"].as<PipeId>()};
        e.start = row["start_node_id"].as<NodeId>();
        e.end = row["end_node_id"].as<NodeId>();
        e.length = row["length"].as<double>();
        e.type = row["type"].as<std::int16_t>();
        e.is_open = row["is_open"].as<bool>();
        edges.push_back(e);
    }
    for (const auto& row : ramp_rows) {
        MatrixEdgeRecord e;
        e.key = {GraphEdgeKind::Ramp, region_id, version_id, row["ramp_id"].as<RampId>()};
        e.start = row["start_node_id"].as<NodeId>();
        e.end = row["end_node_id"].as<NodeId>();
        e.length = row["length"].as<double>();
        e.type = row["type"].as<std::int16_t>();
        e.is_open = row["is_open"].as<bool>();
        edges.push_back(e);
    }
    return edges;
}

} // namespace

Postgresql::Postgresql(const std::string& conn_str) {
    auto pc = globalPool().acquire(conn_str);
    struct Releaser {
        std::string key;
        std::shared_ptr<PooledConn> pc;
        Releaser(std::string k, std::shared_ptr<PooledConn> p) : key(std::move(k)), pc(std::move(p)) {}
        ~Releaser() { globalPool().release(key, pc); }
    };
    pool_token_ = std::make_shared<Releaser>(conn_str, pc);
    conn_getter_ = [pc] { return pc->conn.get(); };
}

pqxx::connection* Postgresql::conn() const {
    return conn_getter_ ? conn_getter_() : nullptr;
}

bool Postgresql::upsertRegionInfo(const RegionInfo& d) {
    try {
        pqxx::work tx(*conn());
        tx.exec_params(R"SQL(
            INSERT INTO region_info(region_id,bandwidth,computing_resource,load,neighbourhood,region_range)
            VALUES($1,$2,$3,$4,$5::integer[],$6::real[])
            ON CONFLICT(region_id) DO UPDATE SET
              bandwidth=EXCLUDED.bandwidth,
              computing_resource=EXCLUDED.computing_resource,
              load=EXCLUDED.load,
              neighbourhood=EXCLUDED.neighbourhood,
              region_range=EXCLUDED.region_range
        )SQL", d.region_id, d.bandwidth, d.computing_resource, d.load, d.neighbourhood, d.region_range);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "upsertRegionInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertTopologyVersions(const std::vector<TopologyVersion>& data) {
    try {
        pqxx::work tx(*conn());
        for (const auto& d : data) {
            tx.exec_params(R"SQL(
                INSERT INTO topology_version(region_id,version_id,valid_from,valid_to,reason)
                VALUES($1,$2,$3::timestamptz,NULLIF($4,'')::timestamptz,NULLIF($5,''))
                ON CONFLICT(region_id,version_id) DO UPDATE SET
                  valid_from=EXCLUDED.valid_from, valid_to=EXCLUDED.valid_to, reason=EXCLUDED.reason
            )SQL", d.region_id, d.version_id, d.valid_from, d.valid_to, d.reason);
        }
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertTopologyVersions: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertLayerInfo(const std::vector<LayerInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) tx.exec_params(R"SQL(
            INSERT INTO layer_info(region_id,version_id,layer_id,angle,height,pipe_number)
            VALUES($1,$2,$3,$4,$5,$6)
            ON CONFLICT(region_id,version_id,layer_id) DO UPDATE SET
              angle=EXCLUDED.angle,height=EXCLUDED.height,pipe_number=EXCLUDED.pipe_number
        )SQL", d.region_id,d.version_id,d.layer_id,d.angle,d.height,d.pipe_number);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertLayerInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertNodeInfo(const std::vector<NodeInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) tx.exec_params(R"SQL(
            INSERT INTO node_info(region_id,version_id,node_id,position,region_ids,layer_id,node_type,boundary_type)
            VALUES($1,$2,$3,$4::real[],$5::integer[],NULLIF($6,-1),$7,$8)
            ON CONFLICT(region_id,version_id,node_id) DO UPDATE SET
              position=EXCLUDED.position,region_ids=EXCLUDED.region_ids,
              layer_id=EXCLUDED.layer_id,node_type=EXCLUDED.node_type,
              boundary_type=EXCLUDED.boundary_type
        )SQL", d.region_id,d.version_id,d.node_id,d.position,d.region_ids,d.layer_id,d.node_type,d.boundary_role);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertNodeInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertPipeInfo(const std::vector<PipeInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) tx.exec_params(R"SQL(
            INSERT INTO pipe_info(region_id,version_id,pipe_id,main_pipe_id,layer_id,type,angle,length,
                                  lane_number,pipe_width,lane_width,start_node_id,end_node_id)
            VALUES($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12,$13)
            ON CONFLICT(region_id,version_id,pipe_id) DO UPDATE SET
              main_pipe_id=EXCLUDED.main_pipe_id,layer_id=EXCLUDED.layer_id,type=EXCLUDED.type,
              angle=EXCLUDED.angle,length=EXCLUDED.length,lane_number=EXCLUDED.lane_number,
              pipe_width=EXCLUDED.pipe_width,lane_width=EXCLUDED.lane_width,
              start_node_id=EXCLUDED.start_node_id,end_node_id=EXCLUDED.end_node_id
        )SQL", d.region_id,d.version_id,d.pipe_id,d.main_pipe_id,d.layer_id,d.type,d.angle,d.length,
                d.lane_number,d.pipe_width,d.lane_width,d.start_node_id,d.end_node_id);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertPipeInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertRampInfo(const std::vector<RampInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) {
            const std::string path = build2dPgArray(d.ramp_path);
            tx.exec_params(R"SQL(
                INSERT INTO ramp_info(region_id,version_id,ramp_id,type,length,ramp_width,lane_number,lane_width,
                                      start_node_id,end_node_id,ramp_path)
                VALUES($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11::real[])
                ON CONFLICT(region_id,version_id,ramp_id) DO UPDATE SET
                  type=EXCLUDED.type,length=EXCLUDED.length,ramp_width=EXCLUDED.ramp_width,
                  lane_number=EXCLUDED.lane_number,lane_width=EXCLUDED.lane_width,
                  start_node_id=EXCLUDED.start_node_id,end_node_id=EXCLUDED.end_node_id,ramp_path=EXCLUDED.ramp_path
            )SQL", d.region_id,d.version_id,d.ramp_id,d.type,d.length,d.ramp_width,d.lane_number,d.lane_width,
                    d.start_node_id,d.end_node_id,path);
        }
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertRampInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertLaneInfo(const std::vector<LaneInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) tx.exec_params(R"SQL(
            INSERT INTO lane_info(region_id,version_id,lane_id,layer_id,pipe_id,length,start_pt,end_pt)
            VALUES($1,$2,$3,$4,$5,$6,$7::real[],$8::real[])
            ON CONFLICT(region_id,version_id,lane_id) DO UPDATE SET
              layer_id=EXCLUDED.layer_id,pipe_id=EXCLUDED.pipe_id,length=EXCLUDED.length,
              start_pt=EXCLUDED.start_pt,end_pt=EXCLUDED.end_pt
        )SQL", d.region_id,d.version_id,d.lane_id,d.layer_id,d.pipe_id,d.length,d.start_pt,d.end_pt);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertLaneInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertPipeStatus(const std::vector<PipeStatus>& data) {
    try { pqxx::work tx(*conn());
        auto st = pqxx::stream_to::table(tx,{"pipe_status"},{"time","region_id","version_id","pipe_id","is_open","d_safe","speed_limit","max_capacity"});
        for (const auto& d : data) st.write_values(d.time,d.region_id,d.version_id,d.pipe_id,d.is_open,d.d_safe,d.speed_limit,d.max_capacity);
        st.complete(); tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertPipeStatus: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertRampStatus(const std::vector<RampStatus>& data) {
    try { pqxx::work tx(*conn());
        auto st = pqxx::stream_to::table(tx,{"ramp_status"},{"time","region_id","version_id","ramp_id","is_open"});
        for (const auto& d : data) st.write_values(d.time,d.region_id,d.version_id,d.ramp_id,d.is_open);
        st.complete(); tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertRampStatus: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertPipeFlow(const std::vector<PipeFlow>& data) {
    try { pqxx::work tx(*conn());
        auto st = pqxx::stream_to::table(tx,{"pipe_flow"},{"time","region_id","version_id","pipe_id","uav_number","flow","density","average_speed"});
        for (const auto& d : data) st.write_values(d.time,d.region_id,d.version_id,d.pipe_id,d.uav_number,d.flow,d.density,d.average_speed);
        st.complete(); tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertPipeFlow: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertRampFlow(const std::vector<RampFlow>& data) {
    try { pqxx::work tx(*conn());
        auto st = pqxx::stream_to::table(tx,{"ramp_flow"},{"time","region_id","version_id","ramp_id","uav_number","flow","density","average_speed"});
        for (const auto& d : data) st.write_values(d.time,d.region_id,d.version_id,d.ramp_id,d.uav_number,d.flow,d.density,d.average_speed);
        st.complete(); tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertRampFlow: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertLaneFlow(const std::vector<LaneFlow>& data) {
    try { pqxx::work tx(*conn());
        auto st = pqxx::stream_to::table(tx,{"lane_flow"},{"time","region_id","version_id","lane_id","pipe_id","uav_number","flow","density","average_speed"});
        for (const auto& d : data) st.write_values(d.time,d.region_id,d.version_id,d.lane_id,d.pipe_id,d.uav_number,d.flow,d.density,d.average_speed);
        st.complete(); tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertLaneFlow: " << e.what() << '\n'; return false; }
}

bool Postgresql::upsertUavInfo(const std::vector<UavInfo>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) tx.exec_params(R"SQL(
            INSERT INTO uav_info(uav_id,source,destination) VALUES($1,$2::real[],$3::real[])
            ON CONFLICT(uav_id) DO UPDATE SET source=EXCLUDED.source,destination=EXCLUDED.destination
        )SQL", d.uav_id,d.source,d.destination);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "upsertUavInfo: " << e.what() << '\n'; return false; }
}

bool Postgresql::insertUavStates(const std::vector<UavState>& data) {
    try { pqxx::work tx(*conn());
        for (const auto& d : data) {
            validateUavState(d);
            tx.exec_params(R"SQL(
                INSERT INTO uav_data(time,uav_id,region_id,version_id,layer_id,edge_type,pipe_id,lane_id,ramp_id,node_id,
                                     distance_on_edge,speed,acceleration,position,status)
                VALUES(
                    $1::timestamptz,$2,$3,$4,NULLIF($5::integer,-1),$6,
                    NULLIF($7::integer,-1),NULLIF($8::integer,-1),NULLIF($9::integer,-1),NULLIF($10::bigint,-1),
                    NULLIF($11::real,-1.0),$12::real,CASE WHEN $13 THEN $14::real ELSE NULL END,$15::real[],$16
                )
            )SQL", d.time,d.uav_id,d.region_id,d.version_id,d.layer_id,static_cast<std::int16_t>(d.edge_type),
                    d.pipe_id,d.lane_id,d.ramp_id,d.node_id,d.distance_on_edge,d.speed,
                    d.has_acceleration,d.acceleration,d.position,d.status);
        }
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "insertUavStates: " << e.what() << '\n'; return false; }
}

bool Postgresql::upsertFlowPrediction(const FlowPrediction& d) {
    try { pqxx::work tx(*conn()); const auto m=build2dPgArray(d.result);
        tx.exec_params("INSERT INTO flow_prediction(region_id,result) VALUES($1,$2::real[]) ON CONFLICT(region_id) DO UPDATE SET result=EXCLUDED.result",d.region_id,m);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "upsertFlowPrediction: " << e.what() << '\n'; return false; }
}

bool Postgresql::upsertFlowGuidance(const FlowGuidance& d) {
    try { pqxx::work tx(*conn()); const auto m=build2dPgArray(d.result);
        tx.exec_params("INSERT INTO flow_guidance(region_id,result) VALUES($1,$2::real[]) ON CONFLICT(region_id) DO UPDATE SET result=EXCLUDED.result",d.region_id,m);
        tx.commit(); return true;
    } catch (const std::exception& e) { std::cerr << "upsertFlowGuidance: " << e.what() << '\n'; return false; }
}

RegionInfo Postgresql::getRegionInfo() {
    pqxx::read_transaction tx(*conn());
    auto r=tx.exec("SELECT region_id,bandwidth,computing_resource,load,neighbourhood,region_range FROM region_info ORDER BY region_id LIMIT 1");
    if(r.empty()) throw std::runtime_error("region_info is empty");
    RegionInfo d; d.region_id=r[0]["region_id"].as<RegionId>(); d.bandwidth=r[0]["bandwidth"].as<double>();
    d.computing_resource=r[0]["computing_resource"].as<double>(); d.load=r[0]["load"].as<double>();
    d.neighbourhood=r[0]["neighbourhood"].as<std::vector<int>>(); d.region_range=r[0]["region_range"].as<Vec>(); return d;
}

RegionResources Postgresql::getRegionResources() {
    const auto r=getRegionInfo(); return {r.region_id,r.bandwidth,r.computing_resource,r.load};
}

bool Postgresql::updateRegionResources(const RegionResources& d) {
    try { pqxx::work tx(*conn()); auto r=tx.exec_params("UPDATE region_info SET bandwidth=$1,computing_resource=$2,load=$3 WHERE region_id=$4 RETURNING region_id",d.bandwidth,d.computing_resource,d.load,d.region_id);
        if (r.empty()) return false;
        tx.commit();
        return true;
    } catch(const std::exception& e){ std::cerr<<"updateRegionResources: "<<e.what()<<'\n'; return false; }
}

TopologyVersion Postgresql::getCurrentTopologyVersion(RegionId region_id) {
    pqxx::read_transaction tx(*conn());
    auto r=tx.exec_params("SELECT region_id,version_id,valid_from::text,valid_to::text,reason FROM topology_version WHERE region_id=$1 AND valid_to IS NULL ORDER BY valid_from DESC LIMIT 1",region_id);
    if(r.empty()) throw std::runtime_error("no current topology version");
    TopologyVersion v;
    v.region_id = r[0]["region_id"].as<RegionId>();
    v.version_id = r[0]["version_id"].as<VersionId>();
    v.valid_from = r[0]["valid_from"].as<std::string>();
    if (r[0]["valid_to"].is_null()) {
        v.valid_to = "";
    } else {
        v.valid_to = r[0]["valid_to"].as<std::string>();
    }
    if (r[0]["reason"].is_null()) {
        v.reason = "";
    } else {
        v.reason = r[0]["reason"].as<std::string>();
    }
    return v;
}

TopologyVersion Postgresql::getTopologyVersionAt(RegionId region_id,const std::string& observation_time) {
    pqxx::read_transaction tx(*conn());
    return topologyVersionAtTxn(tx,region_id,observation_time);
}

std::vector<LayerInfo> Postgresql::getLayerInfo(RegionId region_id,VersionId version_id) {
    pqxx::read_transaction tx(*conn()); auto r=tx.exec_params("SELECT region_id,version_id,layer_id,angle,height,pipe_number FROM layer_info WHERE region_id=$1 AND version_id=$2 ORDER BY layer_id",region_id,version_id);
    std::vector<LayerInfo> out; out.reserve(r.size()); for(const auto& x:r) out.push_back({x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["layer_id"].as<int>(),x["angle"].as<double>(),x["height"].as<double>(),x["pipe_number"].as<int>()}); return out;
}

std::vector<NodeInfo> Postgresql::getNodeInfo(RegionId region_id,VersionId version_id) {
    pqxx::read_transaction tx(*conn()); auto r=tx.exec_params("SELECT region_id,version_id,node_id,position,region_ids,layer_id,node_type,boundary_type FROM node_info WHERE region_id=$1 AND version_id=$2 ORDER BY node_id",region_id,version_id);
    std::vector<NodeInfo> out;
    out.reserve(r.size());
    for (const auto& x : r) {
        NodeInfo d;
        d.region_id = x["region_id"].as<RegionId>();
        d.version_id = x["version_id"].as<VersionId>();
        d.node_id = x["node_id"].as<NodeId>();
        d.position = x["position"].as<Vec>();
        d.region_ids = x["region_ids"].as<std::vector<int>>();
        if (x["layer_id"].is_null()) {
            d.layer_id = INVALID_LAYER_ID;
        } else {
            d.layer_id = x["layer_id"].as<int>();
        }
        d.node_type = x["node_type"].as<std::int16_t>();
        d.boundary_role = x["boundary_type"].as<std::int16_t>();
        out.push_back(std::move(d));
    }
    return out;
}

std::vector<PipeInfo> Postgresql::getPipeInfo(RegionId region_id,VersionId version_id) {
    pqxx::read_transaction tx(*conn()); auto r=tx.exec_params(R"SQL(SELECT region_id,version_id,pipe_id,main_pipe_id,layer_id,type,angle,length,lane_number,pipe_width,lane_width,start_node_id,end_node_id FROM pipe_info WHERE region_id=$1 AND version_id=$2 ORDER BY pipe_id)SQL",region_id,version_id);
    std::vector<PipeInfo> out; out.reserve(r.size()); for(const auto& x:r) out.push_back({x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["pipe_id"].as<PipeId>(),x["main_pipe_id"].as<std::int64_t>(),x["layer_id"].as<int>(),x["type"].as<std::int16_t>(),x["angle"].as<double>(),x["length"].as<double>(),x["lane_number"].as<int>(),x["pipe_width"].as<double>(),x["lane_width"].as<double>(),x["start_node_id"].as<NodeId>(),x["end_node_id"].as<NodeId>()}); return out;
}

std::vector<RampInfo> Postgresql::getRampInfo(RegionId region_id,VersionId version_id) {
    pqxx::read_transaction tx(*conn()); auto r=tx.exec_params("SELECT region_id,version_id,ramp_id,type,length,ramp_width,lane_number,lane_width,start_node_id,end_node_id,ramp_path::text FROM ramp_info WHERE region_id=$1 AND version_id=$2 ORDER BY ramp_id",region_id,version_id);
    std::vector<RampInfo> out; out.reserve(r.size()); for(const auto& x:r){ RampInfo d;d.region_id=x["region_id"].as<RegionId>();d.version_id=x["version_id"].as<VersionId>();d.ramp_id=x["ramp_id"].as<RampId>();d.type=x["type"].as<std::int16_t>();d.length=x["length"].as<double>();d.ramp_width=x["ramp_width"].as<double>();d.lane_number=x["lane_number"].as<int>();d.lane_width=x["lane_width"].as<double>();d.start_node_id=x["start_node_id"].as<NodeId>();d.end_node_id=x["end_node_id"].as<NodeId>();d.ramp_path=parse2dPgArray(x["ramp_path"].as<std::string>());out.push_back(std::move(d)); } return out;
}

std::vector<LaneInfo> Postgresql::getLaneInfo(RegionId region_id,VersionId version_id) {
    pqxx::read_transaction tx(*conn()); auto r=tx.exec_params("SELECT region_id,version_id,lane_id,layer_id,pipe_id,length,start_pt,end_pt FROM lane_info WHERE region_id=$1 AND version_id=$2 ORDER BY pipe_id,lane_id",region_id,version_id);
    std::vector<LaneInfo> out; out.reserve(r.size()); for(const auto& x:r) out.push_back({x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["lane_id"].as<LaneId>(),x["layer_id"].as<int>(),x["pipe_id"].as<PipeId>(),x["length"].as<double>(),x["start_pt"].as<Vec>(),x["end_pt"].as<Vec>()}); return out;
}

std::unordered_map<PipeId,PipeStatus> Postgresql::getPipeStatusAt(RegionId region_id,VersionId version_id,const std::string& observation_time) {
    pqxx::read_transaction tx(*conn()); 
    const auto t=resolveTime(tx,observation_time); 
    auto r=tx.exec_params(R"SQL(
        SELECT DISTINCT ON(pipe_id) time::text,region_id,version_id,pipe_id,is_open,d_safe,speed_limit,max_capacity
        FROM pipe_status WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
        ORDER BY pipe_id,time DESC)SQL",region_id,version_id,t);
    std::unordered_map<PipeId,PipeStatus> out; 
    for(const auto& x:r)
    { PipeStatus d{x["time"].as<std::string>(),x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["pipe_id"].as<PipeId>(),x["is_open"].as<bool>(),x["d_safe"].as<double>(),x["speed_limit"].as<double>(),x["max_capacity"].as<int>()};
    out[d.pipe_id]=d;
    } 
    return out;
}

std::unordered_map<RampId,RampStatus> Postgresql::getRampStatusAt(RegionId region_id,VersionId version_id,const std::string& observation_time) {
    pqxx::read_transaction tx(*conn()); const auto t=resolveTime(tx,observation_time); auto r=tx.exec_params(R"SQL(
        SELECT DISTINCT ON(ramp_id) time::text,region_id,version_id,ramp_id,is_open
        FROM ramp_status WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
        ORDER BY ramp_id,time DESC)SQL",region_id,version_id,t);
    std::unordered_map<RampId,RampStatus> out; for(const auto& x:r){ RampStatus d{x["time"].as<std::string>(),x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["ramp_id"].as<RampId>(),x["is_open"].as<bool>()};out[d.ramp_id]=d;} return out;
}

std::unordered_map<PipeId,std::vector<PipeFlow>> Postgresql::getRecentPipeFlow(int top_n) {
    std::unordered_map<PipeId,std::vector<PipeFlow>> out; if(top_n<=0) return out; pqxx::read_transaction tx(*conn()); const auto rid=localRegionId(tx); const auto v=topologyVersionAtTxn(tx,rid,""); auto r=tx.exec_params(R"SQL(
        WITH ranked AS(SELECT *,ROW_NUMBER() OVER(PARTITION BY region_id,version_id,pipe_id ORDER BY time DESC) rn FROM pipe_flow WHERE region_id=$1 AND version_id=$2)
        SELECT time::text,region_id,version_id,pipe_id,uav_number,flow,density,average_speed FROM ranked WHERE rn <= $3 ORDER BY pipe_id,time DESC)SQL",rid,v.version_id,top_n);
    for(const auto& x:r){ PipeFlow d{x["time"].as<std::string>(),x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["pipe_id"].as<PipeId>(),x["uav_number"].as<int>(),x["flow"].as<double>(),x["density"].as<double>(),x["average_speed"].as<double>()};out[d.pipe_id].push_back(d);} return out;
}

std::unordered_map<RampId,std::vector<RampFlow>> Postgresql::getRecentRampFlow(int top_n) {
    std::unordered_map<RampId,std::vector<RampFlow>> out; if(top_n<=0) return out; pqxx::read_transaction tx(*conn()); const auto rid=localRegionId(tx); const auto v=topologyVersionAtTxn(tx,rid,""); auto r=tx.exec_params(R"SQL(
        WITH ranked AS(SELECT *,ROW_NUMBER() OVER(PARTITION BY region_id,version_id,ramp_id ORDER BY time DESC) rn FROM ramp_flow WHERE region_id=$1 AND version_id=$2)
        SELECT time::text,region_id,version_id,ramp_id,uav_number,flow,density,average_speed FROM ranked WHERE rn <= $3 ORDER BY ramp_id,time DESC)SQL",rid,v.version_id,top_n);
    for(const auto& x:r){ RampFlow d{x["time"].as<std::string>(),x["region_id"].as<RegionId>(),x["version_id"].as<VersionId>(),x["ramp_id"].as<RampId>(),x["uav_number"].as<int>(),x["flow"].as<double>(),x["density"].as<double>(),x["average_speed"].as<double>()};out[d.ramp_id].push_back(d);} return out;
}


std::unordered_map<LaneId,std::vector<LaneFlow>> Postgresql::getRecentLaneFlow(int top_n) {
    std::unordered_map<LaneId,std::vector<LaneFlow>> out;
    if (top_n <= 0) return out;
    pqxx::read_transaction tx(*conn());
    const auto rid = localRegionId(tx);
    const auto v = topologyVersionAtTxn(tx, rid, "");
    auto r = tx.exec_params(R"SQL(
        WITH ranked AS (
            SELECT *, ROW_NUMBER() OVER(
                PARTITION BY region_id,version_id,lane_id ORDER BY time DESC) rn
            FROM lane_flow WHERE region_id=$1 AND version_id=$2
        )
        SELECT time::text,region_id,version_id,lane_id,pipe_id,uav_number,flow,density,average_speed
        FROM ranked WHERE rn <= $3 ORDER BY lane_id,time DESC
    )SQL", rid, v.version_id, top_n);
    for (const auto& x : r) {
        LaneFlow d{x["time"].as<std::string>(), x["region_id"].as<RegionId>(),
                   x["version_id"].as<VersionId>(), x["lane_id"].as<LaneId>(),
                   x["pipe_id"].as<PipeId>(), x["uav_number"].as<int>(),
                   x["flow"].as<double>(), x["density"].as<double>(),
                   x["average_speed"].as<double>()};
        out[d.lane_id].push_back(std::move(d));
    }
    return out;
}

std::vector<UavInfo> Postgresql::getUavInfo() {
    pqxx::read_transaction tx(*conn());
    auto r = tx.exec("SELECT uav_id,source,destination FROM uav_info ORDER BY uav_id");
    std::vector<UavInfo> out;
    out.reserve(r.size());
    for (const auto& x : r) {
        out.push_back({x["uav_id"].as<UavId>(), x["source"].as<Vec>(), x["destination"].as<Vec>()});
    }
    return out;
}

std::unordered_map<UavId,std::vector<UavState>> Postgresql::getRecentUavStates(int top_n) {
    std::unordered_map<UavId,std::vector<UavState>> out; 
    if(top_n<=0) 
    return out; 
    pqxx::read_transaction tx(*conn()); 
    const auto rid=localRegionId(tx); 
    auto r=tx.exec_params(R"SQL(
        WITH ranked AS(SELECT *,ROW_NUMBER() OVER(PARTITION BY uav_id ORDER BY time DESC) rn FROM uav_data WHERE region_id=$1)
        SELECT time::text,uav_id,region_id,version_id,layer_id,edge_type,pipe_id,lane_id,ramp_id,node_id,distance_on_edge,speed,acceleration,position,status
        FROM ranked WHERE rn <= $2 ORDER BY uav_id,time DESC)SQL",rid,top_n);
    for (const auto& x : r) {
        UavState d;
        d.time = x["time"].as<std::string>();
        d.uav_id = x["uav_id"].as<UavId>();
        d.region_id = x["region_id"].as<RegionId>();
        d.version_id = x["version_id"].as<VersionId>();
        if (x["layer_id"].is_null()) {
            d.layer_id = INVALID_LAYER_ID;
        } else {
            d.layer_id = x["layer_id"].as<int>();
        }
        d.edge_type = static_cast<EdgeType>(x["edge_type"].as<std::int16_t>());
        if (x["pipe_id"].is_null()) {
            d.pipe_id = INVALID_PIPE_ID;
        } else {
            d.pipe_id = x["pipe_id"].as<PipeId>();
        }
        if (x["lane_id"].is_null()) {
            d.lane_id = INVALID_LANE_ID;
        } else {
            d.lane_id = x["lane_id"].as<LaneId>();
        }
        if (x["ramp_id"].is_null()) {
            d.ramp_id = INVALID_RAMP_ID;
        } else {
            d.ramp_id = x["ramp_id"].as<RampId>();
        }
        if (x["node_id"].is_null()) {
            d.node_id = INVALID_NODE_ID;
        } else {
            d.node_id = x["node_id"].as<NodeId>();
        }
        if (x["distance_on_edge"].is_null()) {
            d.distance_on_edge = INVALID_DISTANCE;
        } else {
            d.distance_on_edge = x["distance_on_edge"].as<double>();
        }
        d.speed = x["speed"].as<double>();
        d.has_acceleration = !x["acceleration"].is_null();
        if (x["acceleration"].is_null()) {
            d.acceleration = 0.0;
        } else {
            d.acceleration = x["acceleration"].as<double>();
        }
        d.position = x["position"].as<Vec>();
        d.status = x["status"].as<std::int16_t>();
        out[d.uav_id].push_back(std::move(d));
    }
    return out;
}

LocalGraphAdj Postgresql::getLocalTopologyAdj(const std::string& observation_time) {
    pqxx::read_transaction tx(*conn()); 
    const auto rid=localRegionId(tx); 
    const auto t=resolveTime(tx,observation_time); 
    const auto version=topologyVersionAtTxn(tx,rid,t);
    LocalGraphAdj out; 
    out.observation_time=t; 
    out.region_id=rid; 
    out.version_id=version.version_id;

    auto nodes = tx.exec_params(
    "SELECT region_id,version_id,node_id,position,region_ids,"
    "layer_id,node_type,boundary_type AS boundary_role "
    "FROM node_info "
    "WHERE region_id=$1 AND version_id=$2 "
    "ORDER BY node_id",
    rid,
    version.version_id);

    for (const auto& x : nodes) {
        NodeInfo n;
        n.region_id = x["region_id"].as<RegionId>();
        n.version_id = x["version_id"].as<VersionId>();
        n.node_id = x["node_id"].as<NodeId>();
        n.position = x["position"].as<Vec>();
        n.region_ids = x["region_ids"].as<std::vector<int>>();
        if (x["layer_id"].is_null()) {
            n.layer_id = INVALID_LAYER_ID;
        } else {
            n.layer_id = x["layer_id"].as<int>();
        }
        n.node_type = x["node_type"].as<std::int16_t>();
        n.boundary_role = x["boundary_role"].as<std::int16_t>();
        out.nodes.emplace(n.node_id, std::move(n));
    }

    auto pipes=tx.exec_params(R"SQL(
        SELECT p.region_id,p.version_id,p.pipe_id,p.main_pipe_id,p.layer_id,p.type,p.angle,p.length,p.start_node_id,p.end_node_id,
               COALESCE(ps.is_open,FALSE) is_open,COALESCE(ps.d_safe,20) d_safe,COALESCE(ps.speed_limit,15) speed_limit,COALESCE(ps.max_capacity,50) max_capacity
        FROM pipe_info p
        LEFT JOIN LATERAL(SELECT is_open,d_safe,speed_limit,max_capacity FROM pipe_status s WHERE s.region_id=p.region_id AND s.version_id=p.version_id AND s.pipe_id=p.pipe_id AND s.time <= $3::timestamptz ORDER BY s.time DESC LIMIT 1) ps ON TRUE
        WHERE p.region_id=$1 AND p.version_id=$2 ORDER BY p.pipe_id)SQL",rid,version.version_id,t);

    auto lanes=tx.exec_params(R"SQL(
        SELECT l.lane_id,l.pipe_id,l.length,l.start_pt,l.end_pt,
               lf.time::text AS flow_time,lf.uav_number,lf.flow,lf.density,lf.average_speed
        FROM lane_info l
        LEFT JOIN LATERAL(SELECT time,uav_number,flow,density,average_speed FROM lane_flow f WHERE f.region_id=l.region_id AND f.version_id=l.version_id AND f.lane_id=l.lane_id AND f.time <= $3::timestamptz ORDER BY f.time DESC LIMIT 1) lf ON TRUE
        WHERE l.region_id=$1 AND l.version_id=$2 ORDER BY l.pipe_id,l.lane_id)SQL",rid,version.version_id,t);
    std::unordered_map<PipeId,std::vector<LaneState>> lane_map; std::unordered_map<PipeId,int> lane_index;
    for(const auto& x:lanes){ LaneState l;l.lane_id=x["lane_id"].as<LaneId>();l.lane_index=lane_index[x["pipe_id"].as<PipeId>()]++;l.length=x["length"].as<double>();l.start_pt=x["start_pt"].as<Vec>();l.end_pt=x["end_pt"].as<Vec>(); if(!x["flow_time"].is_null()){LaneFlow f;f.time=x["flow_time"].as<std::string>();f.region_id=rid;f.version_id=version.version_id;f.lane_id=l.lane_id;f.pipe_id=x["pipe_id"].as<PipeId>();f.uav_number=x["uav_number"].as<int>();f.flow=x["flow"].as<double>();f.density=x["density"].as<double>();f.average_speed=x["average_speed"].as<double>();l.latest_flow=f;l.has_latest_flow=true;} lane_map[x["pipe_id"].as<PipeId>()].push_back(std::move(l)); }

    for(const auto& x:pipes){ PipeEdge e;e.end_node_id=x["end_node_id"].as<NodeId>();e.length=x["length"].as<double>();e.region_id=rid;e.version_id=version.version_id;e.is_open=x["is_open"].as<bool>();e.pipe_id=x["pipe_id"].as<PipeId>();e.main_pipe_id=x["main_pipe_id"].as<std::int64_t>();e.layer_id=x["layer_id"].as<int>();e.type=x["type"].as<std::int16_t>();e.angle=x["angle"].as<double>();e.d_safe=x["d_safe"].as<double>();e.speed_limit=x["speed_limit"].as<double>();e.max_capacity=x["max_capacity"].as<int>();auto it=lane_map.find(e.pipe_id);if(it!=lane_map.end())e.lanes=std::move(it->second);out.graph[x["start_node_id"].as<NodeId>()].push_back(std::move(e)); }

    auto ramps=tx.exec_params(R"SQL(
        SELECT r.ramp_id,r.type,r.length,r.ramp_width,r.lane_number,r.start_node_id,r.end_node_id,r.ramp_path::text,
               COALESCE(rs.is_open,FALSE) is_open
        FROM ramp_info r
        LEFT JOIN LATERAL(SELECT is_open FROM ramp_status s WHERE s.region_id=r.region_id AND s.version_id=r.version_id AND s.ramp_id=r.ramp_id AND s.time <= $3::timestamptz ORDER BY s.time DESC LIMIT 1) rs ON TRUE
        WHERE r.region_id=$1 AND r.version_id=$2 ORDER BY r.ramp_id)SQL",rid,version.version_id,t);
    for(const auto& x:ramps){ RampEdge e;e.end_node_id=x["end_node_id"].as<NodeId>();e.length=x["length"].as<double>();e.region_id=rid;e.version_id=version.version_id;e.is_open=x["is_open"].as<bool>();e.ramp_id=x["ramp_id"].as<RampId>();e.type=x["type"].as<std::int16_t>();e.ramp_width=x["ramp_width"].as<double>();e.lane_number=x["lane_number"].as<int>();e.ramp_path=parse2dPgArray(x["ramp_path"].as<std::string>());out.graph[x["start_node_id"].as<NodeId>()].push_back(std::move(e)); }
    return out;
}

std::vector<TopologyMatrixSnapshot> Postgresql::getLocalTopologyMatrixSeries(int steps) {
    pqxx::read_transaction tx(*conn()); 
    const auto rid=localRegionId(tx); 
    const auto times=recentSampleTimes(tx,rid,steps); 
    std::vector<TopologyMatrixSnapshot> out; 
    out.reserve(times.size());
    for(const auto& t:times)
    { const auto v=topologyVersionAtTxn(tx,rid,t); 
        const auto edges=matrixEdgesAt(tx,rid,v.version_id,t); 
        TopologyMatrixSnapshot s;s.observation_time=t;
        s.region_id=rid;
        s.version_id=v.version_id;
        s.edge_order.reserve(edges.size());
        s.adjacency.assign(edges.size(),std::vector<int>(edges.size(),0));
        std::unordered_map<NodeId,std::vector<std::size_t>> starts; 
        for(std::size_t i=0;i<edges.size();++i)
        {s.edge_order.push_back(edges[i].key);
        starts[edges[i].start].push_back(i);} 
            for(std::size_t i=0;i<edges.size();++i){if(!edges[i].is_open)
                continue;
                auto it=starts.find(edges[i].end);
                if(it==starts.end())continue;
                for(auto j:it->second)if(edges[j].is_open)
                s.adjacency[i][j]=1;} out.push_back(std::move(s)); }
    return out;
}

std::vector<FlowFeatureSnapshot> Postgresql::getFlowFeatureSeries(int steps) {
    pqxx::read_transaction tx(*conn()); const auto rid=localRegionId(tx); const auto times=recentSampleTimes(tx,rid,steps); std::vector<FlowFeatureSnapshot> out; out.reserve(times.size());
    for(const auto& t:times){ const auto v=topologyVersionAtTxn(tx,rid,t); const auto edges=matrixEdgesAt(tx,rid,v.version_id,t); FlowFeatureSnapshot s;s.observation_time=t;s.region_id=rid;s.version_id=v.version_id;s.edge_order.reserve(edges.size());s.features.reserve(edges.size());
        auto pf=tx.exec_params(R"SQL(SELECT DISTINCT ON(pipe_id) pipe_id,uav_number,flow,density,average_speed FROM pipe_flow WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz ORDER BY pipe_id,time DESC)SQL",rid,v.version_id,t); std::unordered_map<PipeId,std::array<double,4>> pmap; for(const auto& x:pf)pmap[x["pipe_id"].as<PipeId>()]={x["uav_number"].as<double>(),x["flow"].as<double>(),x["density"].as<double>(),x["average_speed"].as<double>()};
        auto rf=tx.exec_params(R"SQL(SELECT DISTINCT ON(ramp_id) ramp_id,uav_number,flow,density,average_speed FROM ramp_flow WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz ORDER BY ramp_id,time DESC)SQL",rid,v.version_id,t); std::unordered_map<RampId,std::array<double,4>> rmap; for(const auto& x:rf)rmap[x["ramp_id"].as<RampId>()]={x["uav_number"].as<double>(),x["flow"].as<double>(),x["density"].as<double>(),x["average_speed"].as<double>()};
        for(const auto& e:edges){s.edge_order.push_back(e.key);std::array<double,4> f{0,0,0,0}; if(e.key.kind==GraphEdgeKind::Pipe){auto it=pmap.find(e.key.edge_id);if(it!=pmap.end())f=it->second;s.features.push_back({f[0],f[1],f[2],f[3],e.is_open?1.0:0.0,e.length,1.0,0.0,0.0,0.0});}else{auto it=rmap.find(e.key.edge_id);if(it!=rmap.end())f=it->second;const double same=e.type==0?1.0:0.0,cross=e.type==1?1.0:0.0,funnel=e.type==2?1.0:0.0;s.features.push_back({f[0],f[1],f[2],f[3],e.is_open?1.0:0.0,e.length,0.0,same,cross,funnel});}} out.push_back(std::move(s)); }
    return out;
}


std::vector<TrainingSnapshot> Postgresql::getTrainingSeries(int steps) {
    pqxx::read_transaction tx(*conn());
    const auto rid = localRegionId(tx);
    const auto times = recentSampleTimes(tx, rid, steps);
    std::vector<TrainingSnapshot> out;
    out.reserve(times.size());

    for (const auto& t : times) {
        const auto v = topologyVersionAtTxn(tx, rid, t);
        const auto edges = matrixEdgesAt(tx, rid, v.version_id, t);

        TrainingSnapshot sample;
        sample.topology.observation_time = t;
        sample.topology.region_id = rid;
        sample.topology.version_id = v.version_id;
        sample.topology.edge_order.reserve(edges.size());
        sample.topology.adjacency.assign(edges.size(), std::vector<int>(edges.size(), 0));

        sample.flow.observation_time = t;
        sample.flow.region_id = rid;
        sample.flow.version_id = v.version_id;
        sample.flow.edge_order.reserve(edges.size());
        sample.flow.features.reserve(edges.size());

        std::unordered_map<NodeId, std::vector<std::size_t>> starts;
        for (std::size_t i = 0; i < edges.size(); ++i) {
            sample.topology.edge_order.push_back(edges[i].key);
            sample.flow.edge_order.push_back(edges[i].key);
            starts[edges[i].start].push_back(i);
        }
        for (std::size_t i = 0; i < edges.size(); ++i) {
            if (!edges[i].is_open) continue;
            const auto it = starts.find(edges[i].end);
            if (it == starts.end()) continue;
            for (const auto j : it->second) {
                if (edges[j].is_open) sample.topology.adjacency[i][j] = 1;
            }
        }

        auto pf = tx.exec_params(R"SQL(
            SELECT DISTINCT ON(pipe_id) pipe_id,uav_number,flow,density,average_speed
            FROM pipe_flow
            WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
            ORDER BY pipe_id,time DESC
        )SQL", rid, v.version_id, t);
        std::unordered_map<PipeId, std::array<double,4>> pmap;
        for (const auto& x : pf) {
            pmap[x["pipe_id"].as<PipeId>()] = {
                x["uav_number"].as<double>(), x["flow"].as<double>(),
                x["density"].as<double>(), x["average_speed"].as<double>()};
        }

        auto rf = tx.exec_params(R"SQL(
            SELECT DISTINCT ON(ramp_id) ramp_id,uav_number,flow,density,average_speed
            FROM ramp_flow
            WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
            ORDER BY ramp_id,time DESC
        )SQL", rid, v.version_id, t);
        std::unordered_map<RampId, std::array<double,4>> rmap;
        for (const auto& x : rf) {
            rmap[x["ramp_id"].as<RampId>()] = {
                x["uav_number"].as<double>(), x["flow"].as<double>(),
                x["density"].as<double>(), x["average_speed"].as<double>()};
        }

        for (const auto& e : edges) {
            std::array<double,4> f{0,0,0,0};
            if (e.key.kind == GraphEdgeKind::Pipe) {
                const auto it = pmap.find(static_cast<PipeId>(e.key.edge_id));
                if (it != pmap.end()) f = it->second;
                sample.flow.features.push_back({
                    f[0], f[1], f[2], f[3], e.is_open ? 1.0 : 0.0,
                    e.length, 1.0, 0.0, 0.0, 0.0});
            } else {
                const auto it = rmap.find(static_cast<RampId>(e.key.edge_id));
                if (it != rmap.end()) f = it->second;
                sample.flow.features.push_back({
                    f[0], f[1], f[2], f[3], e.is_open ? 1.0 : 0.0,
                    e.length, 0.0,
                    e.type == 0 ? 1.0 : 0.0,
                    e.type == 1 ? 1.0 : 0.0,
                    e.type == 2 ? 1.0 : 0.0});
            }
        }

        if (sample.topology.edge_order.size() != sample.flow.edge_order.size() ||
            sample.topology.edge_order.size() != sample.flow.features.size()) {
            throw std::runtime_error("training snapshot topology/feature alignment failure");
        }
        out.push_back(std::move(sample));
    }
    return out;
}


std::vector<TrainingSnapshot> Postgresql::getTrainingSeries(const TimeSeriesQuery& query) {
    pqxx::read_transaction tx(*conn());
    const auto rid = localRegionId(tx);
    const auto times = fixedGridTimes(tx, query);
    std::vector<TrainingSnapshot> out;
    out.reserve(times.size());

    for (const auto& t : times) {
        TopologyVersion v;
        try {
            v = topologyVersionAtTxn(tx, rid, t);
        } catch (const std::exception&) {
            // A fixed-grid replay may start before the first topology version existed.
            // Skip such timestamps instead of fabricating topology.
            continue;
        }
        const auto edges = matrixEdgesAt(tx, rid, v.version_id, t);

        TrainingSnapshot sample;
        sample.topology.observation_time = t;
        sample.topology.region_id = rid;
        sample.topology.version_id = v.version_id;
        sample.topology.edge_order.reserve(edges.size());
        sample.topology.adjacency.assign(edges.size(), std::vector<int>(edges.size(), 0));

        sample.flow.observation_time = t;
        sample.flow.region_id = rid;
        sample.flow.version_id = v.version_id;
        sample.flow.edge_order.reserve(edges.size());
        sample.flow.features.reserve(edges.size());

        std::unordered_map<NodeId, std::vector<std::size_t>> starts;
        for (std::size_t i = 0; i < edges.size(); ++i) {
            sample.topology.edge_order.push_back(edges[i].key);
            sample.flow.edge_order.push_back(edges[i].key);
            starts[edges[i].start].push_back(i);
        }
        for (std::size_t i = 0; i < edges.size(); ++i) {
            if (!edges[i].is_open) continue;
            const auto it = starts.find(edges[i].end);
            if (it == starts.end()) continue;
            for (const auto j : it->second) {
                if (edges[j].is_open) sample.topology.adjacency[i][j] = 1;
            }
        }

        auto pf = tx.exec_params(R"SQL(
            SELECT DISTINCT ON(pipe_id) pipe_id,uav_number,flow,density,average_speed
            FROM pipe_flow
            WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
            ORDER BY pipe_id,time DESC
        )SQL", rid, v.version_id, t);
        std::unordered_map<PipeId, std::array<double,4>> pmap;
        for (const auto& x : pf) {
            pmap[x["pipe_id"].as<PipeId>()] = {
                x["uav_number"].as<double>(), x["flow"].as<double>(),
                x["density"].as<double>(), x["average_speed"].as<double>()};
        }

        auto rf = tx.exec_params(R"SQL(
            SELECT DISTINCT ON(ramp_id) ramp_id,uav_number,flow,density,average_speed
            FROM ramp_flow
            WHERE region_id=$1 AND version_id=$2 AND time <= $3::timestamptz
            ORDER BY ramp_id,time DESC
        )SQL", rid, v.version_id, t);
        std::unordered_map<RampId, std::array<double,4>> rmap;
        for (const auto& x : rf) {
            rmap[x["ramp_id"].as<RampId>()] = {
                x["uav_number"].as<double>(), x["flow"].as<double>(),
                x["density"].as<double>(), x["average_speed"].as<double>()};
        }

        for (const auto& e : edges) {
            std::array<double,4> f{0,0,0,0};
            if (e.key.kind == GraphEdgeKind::Pipe) {
                const auto it = pmap.find(static_cast<PipeId>(e.key.edge_id));
                if (it != pmap.end()) f = it->second;
                sample.flow.features.push_back({
                    f[0], f[1], f[2], f[3], e.is_open ? 1.0 : 0.0,
                    e.length, 1.0, 0.0, 0.0, 0.0});
            } else {
                const auto it = rmap.find(static_cast<RampId>(e.key.edge_id));
                if (it != rmap.end()) f = it->second;
                sample.flow.features.push_back({
                    f[0], f[1], f[2], f[3], e.is_open ? 1.0 : 0.0,
                    e.length, 0.0,
                    e.type == 0 ? 1.0 : 0.0,
                    e.type == 1 ? 1.0 : 0.0,
                    e.type == 2 ? 1.0 : 0.0});
            }
        }

        if (sample.topology.edge_order != sample.flow.edge_order ||
            sample.topology.edge_order.size() != sample.flow.features.size()) {
            throw std::runtime_error("training snapshot topology/feature alignment failure");
        }
        out.push_back(std::move(sample));
    }
    return out;
}

std::vector<TopologyMatrixSnapshot>
Postgresql::getLocalTopologyMatrixSeries(const TimeSeriesQuery& query) {
    auto samples = getTrainingSeries(query);
    std::vector<TopologyMatrixSnapshot> out;
    out.reserve(samples.size());
    for (auto& s : samples) out.push_back(std::move(s.topology));
    return out;
}

std::vector<FlowFeatureSnapshot>
Postgresql::getFlowFeatureSeries(const TimeSeriesQuery& query) {
    auto samples = getTrainingSeries(query);
    std::vector<FlowFeatureSnapshot> out;
    out.reserve(samples.size());
    for (auto& s : samples) out.push_back(std::move(s.flow));
    return out;
}

bool Postgresql::appendPipeSafetyDistance(const std::vector<std::pair<PipeId,double>>& data,const std::string& time) {
    try { pqxx::work tx(*conn()); 
        const auto rid=localRegionId(tx); 
        const auto t=resolveTime(tx,time); 
        const auto v=topologyVersionAtTxn(tx,rid,t);
        for(const auto& [pid,dsafe]:data){ 
            auto r=tx.exec_params(R"SQL(
            INSERT INTO pipe_status(time,region_id,version_id,pipe_id,is_open,d_safe,speed_limit,max_capacity)
            SELECT $1::timestamptz,$2,$3,$4,COALESCE(s.is_open,TRUE),$5,COALESCE(s.speed_limit,15),COALESCE(s.max_capacity,50)
            FROM (SELECT 1) dummy LEFT JOIN LATERAL(SELECT is_open,speed_limit,max_capacity FROM pipe_status 
            WHERE region_id=$2 AND version_id=$3 AND pipe_id=$4 AND time <= $1::timestamptz ORDER BY time DESC LIMIT 1) s ON TRUE
            ON CONFLICT(time,region_id,version_id,pipe_id) DO UPDATE SET d_safe=EXCLUDED.d_safe
            RETURNING pipe_id)SQL",t,rid,v.version_id,pid,dsafe); 
            if(r.empty())
            throw std::runtime_error("failed to append pipe safety distance"); 
        }
        tx.commit(); 
        return true;
    } catch(const std::exception& e){
        std::cerr<<"appendPipeSafetyDistance: "<<e.what()<<'\n';return false;}
}

std::vector<std::tuple<PipeId,double,double>> Postgresql::getLatestPipeSafetyAndSpeed() {
    pqxx::read_transaction tx(*conn()); 
    const auto rid=localRegionId(tx); 
    const auto v=topologyVersionAtTxn(tx,rid,""); 
    auto r=tx.exec_params(R"SQL(SELECT DISTINCT ON(pipe_id) pipe_id,d_safe,speed_limit 
        FROM pipe_status 
        WHERE region_id=$1 AND version_id=$2 ORDER BY pipe_id,time DESC)SQL",rid,v.version_id); 
    std::vector<std::tuple<PipeId,double,double>> out; 
    for(const auto& x:r)
    out.emplace_back(x["pipe_id"].as<PipeId>(),x["d_safe"].as<double>(),x["speed_limit"].as<double>());
    return out;
}

FlowGuidance Postgresql::getFlowGuidance() { 
    pqxx::read_transaction tx(*conn()); 
    const auto rid=localRegionId(tx); 
    auto r=tx.exec_params("SELECT region_id,result::text FROM flow_guidance WHERE region_id=$1",rid); 
    if(r.empty())return {}; 
    return {r[0]["region_id"].as<RegionId>(),parse2dPgArray(r[0]["result"].as<std::string>())}; 
}
FlowPrediction Postgresql::getFlowPrediction() { 
    pqxx::read_transaction tx(*conn());
    const auto rid=localRegionId(tx); 
    auto r=tx.exec_params("SELECT region_id,result::text FROM flow_prediction WHERE region_id=$1",rid); 
    if(r.empty())return {}; 
    return {r[0]["region_id"].as<RegionId>(),parse2dPgArray(r[0]["result"].as<std::string>())}; }

std::vector<FlightIntent> Postgresql::getFlightIntents() {
    pqxx::read_transaction tx(*conn()); const auto rid=localRegionId(tx); auto r=tx.exec_params(R"SQL(
        SELECT DISTINCT ON(d.uav_id) d.time::text,d.uav_id,d.position,i.source,i.destination
        FROM uav_data d JOIN uav_info i ON i.uav_id=d.uav_id
        WHERE d.region_id=$1 ORDER BY d.uav_id,d.time DESC)SQL",rid); std::vector<FlightIntent> out; out.reserve(r.size()); for(const auto& x:r)out.push_back({x["time"].as<std::string>(),x["uav_id"].as<UavId>(),x["position"].as<Vec>(),x["source"].as<Vec>(),x["destination"].as<Vec>()}); return out;
}

GlobalGraph Postgresql::getGlobalTopologyAdj(const std::string& observation_time) {
    pqxx::read_transaction tx(*conn()); 
    const auto t=resolveTime(tx,observation_time); 
    GlobalGraph out; 
    out.observation_time=t;
    auto versions=tx.exec_params(R"SQL(SELECT region_id,version_id
         FROM topology_version 
         WHERE valid_from <= $1::timestamptz AND (valid_to IS NULL OR $1::timestamptz < valid_to) ORDER BY region_id)SQL",t);
    for(const auto& vr:versions) {
        const RegionId rid=vr["region_id"].as<RegionId>();
        const VersionId vid=vr["version_id"].as<VersionId>();
        auto nodes=tx.exec_params("SELECT region_id,version_id,node_id,position,region_ids,layer_id,node_type FROM node_info WHERE region_id=$1 AND version_id=$2",rid,vid);
        for(const auto& x:nodes) {
            NodeInfo n;
            n.region_id=rid;
            n.version_id=vid;
            n.node_id=x["node_id"].as<NodeId>();
            n.position=x["position"].as<Vec>();
            n.region_ids=x["region_ids"].as<std::vector<int>>();
            if (x["layer_id"].is_null()) { 
                n.layer_id = INVALID_LAYER_ID; 
            } 
            else {
                n.layer_id = x["layer_id"].as<int>(); 
            }
            n.node_type=x["node_type"].as<std::int16_t>();
            auto it=out.nodes.find(n.node_id);
            if(it==out.nodes.end())out.nodes.emplace(n.node_id,n);
            else{
                for(int r:n.region_ids)
                if(std::find(it->second.region_ids.begin(),it->second.region_ids.end(),r)==it->second.region_ids.end())
                it->second.region_ids.push_back(r);}
            }
        auto edges=matrixEdgesAt(tx,rid,vid,t);
        for(const auto& e:edges)
            out.graph[e.start].push_back({e.end,e.length,rid,vid,e.key.kind,e.key.edge_id,e.is_open}); }
    return out;
}
