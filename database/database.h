#ifndef AIRWAY_DATABASE_H
#define AIRWAY_DATABASE_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

namespace pqxx { class connection; }

using RegionId = std::int32_t;
using VersionId = std::int64_t;
using NodeId = std::int64_t;
using PipeId = std::int32_t;
using RampId = std::int32_t;
using LaneId = std::int32_t;
using UavId = std::int32_t;
using Vec = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

constexpr int INVALID_LAYER_ID = -1;
constexpr PipeId INVALID_PIPE_ID = -1;
constexpr RampId INVALID_RAMP_ID = -1;
constexpr LaneId INVALID_LANE_ID = -1;
constexpr NodeId INVALID_NODE_ID = -1;
constexpr double INVALID_DISTANCE = -1.0;

struct RegionResources {
    RegionId region_id{};
    double bandwidth{};
    double computing_resource{};
    double load{};
};

struct RegionInfo : RegionResources {
    std::vector<int> neighbourhood;
    Vec region_range; // [xmin,xmax,ymin,ymax]
};

struct TopologyVersion {
    RegionId region_id{};
    VersionId version_id{};
    std::string valid_from;
    std::string valid_to; // empty = SQL NULL/current version
    std::string reason;   // empty = SQL NULL
};

struct LayerInfo {
    RegionId region_id{};
    VersionId version_id{};
    int layer_id{};
    double angle{};
    double height{};
    int pipe_number{};
};

struct NodeInfo {
    RegionId region_id{};
    VersionId version_id{};
    NodeId node_id{};
    Vec position;
    std::vector<int> region_ids;
    int layer_id{INVALID_LAYER_ID};
    std::int16_t node_type{}; // 0 general/boundary, 1 F_OUT, 2 F_IN
    std::int16_t boundary_role{}; // 0 none, 1 entry, 2 exit
};

struct PipeInfo {
    RegionId region_id{};
    VersionId version_id{};
    PipeId pipe_id{};
    std::int64_t main_pipe_id{};
    int layer_id{};
    std::int16_t type{};      // 0 main, 1 backup
    double angle{};
    double length{};
    int lane_number{};
    double pipe_width{};
    double lane_width{};
    NodeId start_node_id{};
    NodeId end_node_id{};
};

struct RampInfo {
    RegionId region_id{};
    VersionId version_id{};
    RampId ramp_id{};
    std::int16_t type{};      // 0 same-layer reverse, 1 cross-layer, 2 funnel
    double length{};
    double ramp_width{10.0};
    int lane_number{1};
    double lane_width{5.0};
    NodeId start_node_id{};
    NodeId end_node_id{};
    Matrix ramp_path;
};

struct LaneInfo {
    RegionId region_id{};
    VersionId version_id{};
    LaneId lane_id{};
    int layer_id{};
    PipeId pipe_id{};
    double length{};
    Vec start_pt;
    Vec end_pt;
};

struct PipeStatus {
    std::string time;
    RegionId region_id{};
    VersionId version_id{};
    PipeId pipe_id{};
    bool is_open{true};
    double d_safe{20.0};
    double speed_limit{15.0};
    int max_capacity{50};
};

struct RampStatus {
    std::string time;
    RegionId region_id{};
    VersionId version_id{};
    RampId ramp_id{};
    bool is_open{true};
};

struct LaneFlow {
    std::string time;
    RegionId region_id{};
    VersionId version_id{};
    LaneId lane_id{};
    PipeId pipe_id{};
    int uav_number{};
    double flow{};
    double density{};
    double average_speed{};
};

struct PipeFlow {
    std::string time;
    RegionId region_id{};
    VersionId version_id{};
    PipeId pipe_id{};
    int uav_number{};
    double flow{};
    double density{};
    double average_speed{};
};

struct RampFlow {
    std::string time;
    RegionId region_id{};
    VersionId version_id{};
    RampId ramp_id{};
    int uav_number{};
    double flow{};
    double density{};
    double average_speed{};
};

struct UavInfo {
    UavId uav_id{};
    Vec source;
    Vec destination;
};

enum class EdgeType : std::int16_t { Pipe = 0, Ramp = 1, Node = 2 };

struct UavState {
    std::string time;
    UavId uav_id{};
    RegionId region_id{};
    VersionId version_id{};
    int layer_id{INVALID_LAYER_ID};
    EdgeType edge_type{EdgeType::Node};
    PipeId pipe_id{INVALID_PIPE_ID};
    LaneId lane_id{INVALID_LANE_ID};
    RampId ramp_id{INVALID_RAMP_ID};
    NodeId node_id{INVALID_NODE_ID};
    double distance_on_edge{INVALID_DISTANCE};
    double speed{0.0};                // scalar m/s, NOT NULL
    double acceleration{0.0};         // valid only when has_acceleration=true
    bool has_acceleration{false};     // preserves SQL NULL without std::optional
    Vec position;
    std::int16_t status{};
};

struct FlowGuidance {
    RegionId region_id{};
    Matrix result;
};

struct FlowPrediction {
    RegionId region_id{};
    Matrix result;
};

// In-memory algorithm structures ------------------------------------------------
struct LaneState {
    LaneId lane_id{};
    int lane_index{}; // algorithm-side physical order in one pipe, 0..lane_number-1; derived from lane_id order
    double length{};
    Vec start_pt;
    Vec end_pt;
    LaneFlow latest_flow;
    bool has_latest_flow{false};
};

struct EdgeBase {
    NodeId end_node_id{};
    double length{};
    RegionId region_id{};
    VersionId version_id{};
    bool is_open{true};
};

struct PipeEdge : EdgeBase {
    PipeId pipe_id{};
    std::int64_t main_pipe_id{};
    int layer_id{};
    std::int16_t type{};
    double angle{};
    double d_safe{};
    double speed_limit{};
    int max_capacity{};
    std::vector<LaneState> lanes;
};

struct RampEdge : EdgeBase {
    RampId ramp_id{};
    std::int16_t type{};
    double ramp_width{};
    int lane_number{1};
    Matrix ramp_path;
};

using LocalEdge = std::variant<PipeEdge, RampEdge>;

struct LocalGraphAdj {
    std::string observation_time;
    RegionId region_id{};
    VersionId version_id{};
    std::unordered_map<NodeId, std::vector<LocalEdge>> graph;
    std::unordered_map<NodeId, NodeInfo> nodes;
};

enum class GraphEdgeKind : std::int16_t { Pipe = 0, Ramp = 1 };

struct EdgeKey {
    GraphEdgeKind kind{GraphEdgeKind::Pipe};
    RegionId region_id{};
    VersionId version_id{};
    std::int32_t edge_id{}; // pipe_id or ramp_id

    bool operator==(const EdgeKey& other) const noexcept {
        return kind == other.kind && region_id == other.region_id &&
               version_id == other.version_id && edge_id == other.edge_id;
    }
    bool operator!=(const EdgeKey& other) const noexcept { return !(*this == other); }
};

struct TopologyMatrixSnapshot {
    std::string observation_time;
    RegionId region_id{};
    VersionId version_id{};
    std::vector<EdgeKey> edge_order;
    std::vector<std::vector<int>> adjacency; // edge-to-edge adjacency, open edges only
};

struct FlowFeatureSnapshot {
    std::string observation_time;
    RegionId region_id{};
    VersionId version_id{};
    std::vector<EdgeKey> edge_order; // identical ordering rule to topology snapshot
    // [uav_number, flow, density, average_speed, is_open, length,
    //  type_main, type_same_layer_ramp, type_cross_layer_ramp, type_funnel_ramp]
    std::vector<std::vector<double>> features;
};


struct TimeSeriesQuery {
    std::string end_time;  // empty = current database time
    int steps{1000};
    int step_seconds{60};  // fixed sampling interval for replay/training
};

struct TrainingSnapshot {
    TopologyMatrixSnapshot topology;
    FlowFeatureSnapshot flow;
};

struct GlobalEdge {
    NodeId end_node_id{};
    double length{};
    RegionId region_id{};
    VersionId version_id{};
    GraphEdgeKind kind{GraphEdgeKind::Pipe};
    std::int32_t edge_id{};
    bool is_open{true};
};

struct GlobalGraph {
    std::string observation_time;
    std::unordered_map<NodeId, std::vector<GlobalEdge>> graph;
    std::unordered_map<NodeId, NodeInfo> nodes;
};

struct FlightIntent {
    std::string time;
    UavId uav_id{};
    Vec position;
    Vec source;
    Vec destination;
};

class IDatabaseBase {
public:
    virtual ~IDatabaseBase() = default;

    virtual bool upsertRegionInfo(const RegionInfo& data) = 0;
    virtual bool insertTopologyVersions(const std::vector<TopologyVersion>& data) = 0;
    virtual bool insertLayerInfo(const std::vector<LayerInfo>& data) = 0;
    virtual bool insertNodeInfo(const std::vector<NodeInfo>& data) = 0;
    virtual bool insertPipeInfo(const std::vector<PipeInfo>& data) = 0;
    virtual bool insertRampInfo(const std::vector<RampInfo>& data) = 0;
    virtual bool insertLaneInfo(const std::vector<LaneInfo>& data) = 0;

    virtual bool insertPipeStatus(const std::vector<PipeStatus>& data) = 0;
    virtual bool insertRampStatus(const std::vector<RampStatus>& data) = 0;
    virtual bool insertPipeFlow(const std::vector<PipeFlow>& data) = 0;
    virtual bool insertRampFlow(const std::vector<RampFlow>& data) = 0;
    virtual bool insertLaneFlow(const std::vector<LaneFlow>& data) = 0;

    virtual bool upsertUavInfo(const std::vector<UavInfo>& data) = 0;
    virtual bool insertUavStates(const std::vector<UavState>& data) = 0;

    virtual bool upsertFlowPrediction(const FlowPrediction& data) = 0;
    virtual bool upsertFlowGuidance(const FlowGuidance& data) = 0;
};

class IRegionDatabase : public virtual IDatabaseBase {
public:
    virtual ~IRegionDatabase() = default;

    virtual RegionInfo getRegionInfo() = 0;
    virtual RegionResources getRegionResources() = 0;
    virtual bool updateRegionResources(const RegionResources& data) = 0;

    virtual TopologyVersion getCurrentTopologyVersion(RegionId region_id) = 0;
    virtual TopologyVersion getTopologyVersionAt(
        RegionId region_id, const std::string& observation_time) = 0;

    virtual std::vector<LayerInfo> getLayerInfo(RegionId region_id, VersionId version_id) = 0;
    virtual std::vector<NodeInfo> getNodeInfo(RegionId region_id, VersionId version_id) = 0;
    virtual std::vector<PipeInfo> getPipeInfo(RegionId region_id, VersionId version_id) = 0;
    virtual std::vector<RampInfo> getRampInfo(RegionId region_id, VersionId version_id) = 0;
    virtual std::vector<LaneInfo> getLaneInfo(RegionId region_id, VersionId version_id) = 0;

    virtual std::unordered_map<PipeId, PipeStatus> getPipeStatusAt(
        RegionId region_id, VersionId version_id, const std::string& observation_time) = 0;
    virtual std::unordered_map<RampId, RampStatus> getRampStatusAt(
        RegionId region_id, VersionId version_id, const std::string& observation_time) = 0;

    virtual std::unordered_map<PipeId, std::vector<PipeFlow>> getRecentPipeFlow(int top_n = 12) = 0;
    virtual std::unordered_map<RampId, std::vector<RampFlow>> getRecentRampFlow(int top_n = 12) = 0;
    virtual std::unordered_map<LaneId, std::vector<LaneFlow>> getRecentLaneFlow(int top_n = 12) = 0;
    virtual std::vector<UavInfo> getUavInfo() = 0;
    virtual std::unordered_map<UavId, std::vector<UavState>> getRecentUavStates(int top_n = 12) = 0;

    // Algorithm-ready views. Empty observation_time means current database time.
    virtual LocalGraphAdj getLocalTopologyAdj(const std::string& observation_time = "") = 0;
    // Recent observed timestamps (status/flow driven).
    virtual std::vector<TopologyMatrixSnapshot> getLocalTopologyMatrixSeries(int steps = 1000) = 0;
    virtual std::vector<FlowFeatureSnapshot> getFlowFeatureSeries(int steps = 1000) = 0;
    virtual std::vector<TrainingSnapshot> getTrainingSeries(int steps = 1000) = 0;

    // Fixed-grid timestamps for replay/model training.
    virtual std::vector<TopologyMatrixSnapshot> getLocalTopologyMatrixSeries(const TimeSeriesQuery& query) = 0;
    virtual std::vector<FlowFeatureSnapshot> getFlowFeatureSeries(const TimeSeriesQuery& query) = 0;
    virtual std::vector<TrainingSnapshot> getTrainingSeries(const TimeSeriesQuery& query) = 0;

    virtual bool appendPipeSafetyDistance(
        const std::vector<std::pair<PipeId, double>>& data,
        const std::string& time = "") = 0;
    virtual std::vector<std::tuple<PipeId, double, double>> getLatestPipeSafetyAndSpeed() = 0;

    virtual FlowGuidance getFlowGuidance() = 0;
    virtual FlowPrediction getFlowPrediction() = 0;
    virtual std::vector<FlightIntent> getFlightIntents() = 0;
};

class ICentralDatabase : public virtual IDatabaseBase {
public:
    virtual ~ICentralDatabase() = default;
    virtual GlobalGraph getGlobalTopologyAdj(const std::string& observation_time = "") = 0;
};

class Postgresql final : public IRegionDatabase, public ICentralDatabase {
private:
    std::shared_ptr<void> pool_token_;
    std::function<pqxx::connection*()> conn_getter_;
    pqxx::connection* conn() const;

public:
    explicit Postgresql(const std::string& conn_str);
    ~Postgresql() override = default;

    bool upsertRegionInfo(const RegionInfo& data) override;
    bool insertTopologyVersions(const std::vector<TopologyVersion>& data) override;
    bool insertLayerInfo(const std::vector<LayerInfo>& data) override;
    bool insertNodeInfo(const std::vector<NodeInfo>& data) override;
    bool insertPipeInfo(const std::vector<PipeInfo>& data) override;
    bool insertRampInfo(const std::vector<RampInfo>& data) override;
    bool insertLaneInfo(const std::vector<LaneInfo>& data) override;
    bool insertPipeStatus(const std::vector<PipeStatus>& data) override;
    bool insertRampStatus(const std::vector<RampStatus>& data) override;
    bool insertPipeFlow(const std::vector<PipeFlow>& data) override;
    bool insertRampFlow(const std::vector<RampFlow>& data) override;
    bool insertLaneFlow(const std::vector<LaneFlow>& data) override;
    bool upsertUavInfo(const std::vector<UavInfo>& data) override;
    bool insertUavStates(const std::vector<UavState>& data) override;
    bool upsertFlowPrediction(const FlowPrediction& data) override;
    bool upsertFlowGuidance(const FlowGuidance& data) override;

    RegionInfo getRegionInfo() override;
    RegionResources getRegionResources() override;
    bool updateRegionResources(const RegionResources& data) override;
    TopologyVersion getCurrentTopologyVersion(RegionId region_id) override;
    TopologyVersion getTopologyVersionAt(
        RegionId region_id, const std::string& observation_time) override;

    std::vector<LayerInfo> getLayerInfo(RegionId region_id, VersionId version_id) override;
    std::vector<NodeInfo> getNodeInfo(RegionId region_id, VersionId version_id) override;
    std::vector<PipeInfo> getPipeInfo(RegionId region_id, VersionId version_id) override;
    std::vector<RampInfo> getRampInfo(RegionId region_id, VersionId version_id) override;
    std::vector<LaneInfo> getLaneInfo(RegionId region_id, VersionId version_id) override;
    std::unordered_map<PipeId, PipeStatus> getPipeStatusAt(
        RegionId region_id, VersionId version_id, const std::string& observation_time) override;
    std::unordered_map<RampId, RampStatus> getRampStatusAt(
        RegionId region_id, VersionId version_id, const std::string& observation_time) override;

    std::unordered_map<PipeId, std::vector<PipeFlow>> getRecentPipeFlow(int top_n = 12) override;
    std::unordered_map<RampId, std::vector<RampFlow>> getRecentRampFlow(int top_n = 12) override;
    std::unordered_map<LaneId, std::vector<LaneFlow>> getRecentLaneFlow(int top_n = 12) override;
    std::vector<UavInfo> getUavInfo() override;
    std::unordered_map<UavId, std::vector<UavState>> getRecentUavStates(int top_n = 12) override;

    LocalGraphAdj getLocalTopologyAdj(const std::string& observation_time = "") override;
    std::vector<TopologyMatrixSnapshot> getLocalTopologyMatrixSeries(int steps = 1000) override;
    std::vector<FlowFeatureSnapshot> getFlowFeatureSeries(int steps = 1000) override;
    std::vector<TrainingSnapshot> getTrainingSeries(int steps = 1000) override;
    std::vector<TopologyMatrixSnapshot> getLocalTopologyMatrixSeries(const TimeSeriesQuery& query) override;
    std::vector<FlowFeatureSnapshot> getFlowFeatureSeries(const TimeSeriesQuery& query) override;
    std::vector<TrainingSnapshot> getTrainingSeries(const TimeSeriesQuery& query) override;
    bool appendPipeSafetyDistance(
        const std::vector<std::pair<PipeId, double>>& data,
        const std::string& time = "") override;
    std::vector<std::tuple<PipeId, double, double>> getLatestPipeSafetyAndSpeed() override;

    FlowGuidance getFlowGuidance() override;
    FlowPrediction getFlowPrediction() override;
    std::vector<FlightIntent> getFlightIntents() override;
    GlobalGraph getGlobalTopologyAdj(const std::string& observation_time = "") override;
};

#endif
