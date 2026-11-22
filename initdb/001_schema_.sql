BEGIN;

CREATE EXTENSION IF NOT EXISTS timescaledb;
CREATE SCHEMA IF NOT EXISTS public;

CREATE TABLE IF NOT EXISTS public.region_info (
    region_id              INTEGER PRIMARY KEY,
    bandwidth              REAL NOT NULL DEFAULT 0,
    computing_resource     REAL NOT NULL DEFAULT 0,
    load                   REAL NOT NULL DEFAULT 0,
    neighbourhood          INTEGER[] NOT NULL DEFAULT ARRAY[0]::INTEGER[],
    region_range           REAL[] NOT NULL,
    CONSTRAINT ck_region_range_len CHECK (cardinality(region_range) = 4)
);

COMMENT ON TABLE public.region_info IS '区域基本信息表';
COMMENT ON COLUMN public.region_info.region_range IS '[xmin,xmax,ymin,ymax]';
COMMENT ON COLUMN public.region_info.neighbourhood IS '邻域区域ID列表';

CREATE TABLE IF NOT EXISTS public.topology_version (
    region_id      INTEGER NOT NULL,
    version_id     BIGINT NOT NULL,
    valid_from     TIMESTAMPTZ NOT NULL,
    valid_to       TIMESTAMPTZ NULL,
    reason         TEXT NULL,
    PRIMARY KEY (region_id, version_id),
    CONSTRAINT fk_topology_version_region
        FOREIGN KEY (region_id) REFERENCES public.region_info(region_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT ck_topology_version_time
        CHECK (valid_to IS NULL OR valid_to > valid_from)
);

COMMENT ON TABLE public.topology_version IS '区域拓扑版本生命周期';
COMMENT ON COLUMN public.topology_version.valid_to IS 'NULL表示当前仍生效';

CREATE UNIQUE INDEX IF NOT EXISTS ux_topology_version_current
ON public.topology_version(region_id)
WHERE valid_to IS NULL;

CREATE INDEX IF NOT EXISTS idx_topology_version_time
ON public.topology_version(region_id, valid_from DESC);

CREATE TABLE IF NOT EXISTS public.layer_info (
    region_id       INTEGER NOT NULL,
    version_id      BIGINT NOT NULL,
    layer_id        INTEGER NOT NULL,
    angle           REAL NOT NULL,
    height          REAL NOT NULL,
    pipe_number     INTEGER NOT NULL,
    PRIMARY KEY (region_id, version_id, layer_id),
    CONSTRAINT fk_layer_topology_version
        FOREIGN KEY (region_id, version_id)
        REFERENCES public.topology_version(region_id, version_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_layer_pipe_number CHECK (pipe_number >= 0)
);

COMMENT ON TABLE public.layer_info IS '某拓扑版本下的层信息';
COMMENT ON COLUMN public.layer_info.pipe_number IS '本层完整主干数量';

CREATE TABLE IF NOT EXISTS public.node_info (
    region_id       INTEGER NOT NULL,
    version_id      BIGINT NOT NULL,
    node_id         BIGINT NOT NULL,
    position        REAL[] NOT NULL,
    region_ids      INTEGER[] NOT NULL,
    layer_id        INTEGER NULL,
    node_type       SMALLINT NOT NULL DEFAULT 0,
    boundary_type   SMALLINT NOT NULL DEFAULT 0,
    PRIMARY KEY (region_id, version_id, node_id),
    CONSTRAINT fk_node_topology_version
        FOREIGN KEY (region_id, version_id)
        REFERENCES public.topology_version(region_id, version_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_node_position_len CHECK (cardinality(position) = 3),
    CONSTRAINT ck_node_region_ids_nonempty CHECK (cardinality(region_ids) >= 1),
    CONSTRAINT ck_node_type CHECK (node_type IN (0,1,2,3))
);

COMMENT ON TABLE public.node_info IS '拓扑节点完整快照';
COMMENT ON COLUMN public.node_info.node_id IS '36区域统一的全局节点ID';
COMMENT ON COLUMN public.node_info.position IS '[x,y,z]';
COMMENT ON COLUMN public.node_info.region_ids IS '该节点实际关联区域ID集合';
COMMENT ON COLUMN public.node_info.node_type IS '0主干/边界节点,1 Funnel OUT,2 Funnel IN,3 引路交汇节点';
COMMENT ON COLUMN public.node_info.boundary_type IS '0非边界,1区域边界入口,2区域边界出口';

CREATE INDEX IF NOT EXISTS idx_node_global_id
ON public.node_info(node_id);

CREATE INDEX IF NOT EXISTS idx_node_region_version_layer
ON public.node_info(region_id, version_id, layer_id);

CREATE TABLE IF NOT EXISTS public.pipe_info (
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    pipe_id             INTEGER NOT NULL,
    main_pipe_id        BIGINT NOT NULL,
    main_pipe_pair_id   BIGINT NOT NULL,
    layer_id            INTEGER NOT NULL,
    type                SMALLINT NOT NULL DEFAULT 0,
    angle               REAL NOT NULL,
    length              REAL NOT NULL,
    lane_number         INTEGER NOT NULL DEFAULT 5,
    pipe_width          REAL NOT NULL DEFAULT 30,
    lane_width          REAL NOT NULL DEFAULT 6,
    start_node_id       BIGINT NOT NULL,
    end_node_id         BIGINT NOT NULL,
    PRIMARY KEY (region_id, version_id, pipe_id),
    CONSTRAINT fk_pipe_layer
        FOREIGN KEY (region_id, version_id, layer_id)
        REFERENCES public.layer_info(region_id, version_id, layer_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT fk_pipe_start_node
        FOREIGN KEY (region_id, version_id, start_node_id)
        REFERENCES public.node_info(region_id, version_id, node_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_pipe_end_node
        FOREIGN KEY (region_id, version_id, end_node_id)
        REFERENCES public.node_info(region_id, version_id, node_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT ck_pipe_type CHECK (type IN (0,1)),
    CONSTRAINT ck_pipe_length CHECK (length > 0),
    CONSTRAINT ck_pipe_lane_number CHECK (lane_number > 0),
    CONSTRAINT ck_pipe_width CHECK (pipe_width > 0),
    CONSTRAINT ck_pipe_lane_width CHECK (lane_width > 0),
    CONSTRAINT ck_pipe_distinct_nodes CHECK (start_node_id <> end_node_id)
);

COMMENT ON TABLE public.pipe_info IS '主干管道segment静态拓扑表';
COMMENT ON COLUMN public.pipe_info.main_pipe_id IS '完整主干全局ID，同一完整主干的多个segment共享该ID';
COMMENT ON COLUMN public.pipe_info.main_pipe_pair_id IS '主干管道对ID，用于标识主干管道的配对关系';
COMMENT ON COLUMN public.pipe_info.type IS '0主用,1备用';
COMMENT ON COLUMN public.pipe_info.pipe_width IS '管道直径/宽度，当前默认30m';
COMMENT ON COLUMN public.pipe_info.lane_width IS 'pipe内部lane宽度，当前默认6m';

CREATE INDEX IF NOT EXISTS idx_pipe_main_pipe
ON public.pipe_info(main_pipe_id);
CREATE INDEX IF NOT EXISTS idx_pipe_region_version_layer
ON public.pipe_info(region_id, version_id, layer_id);
CREATE INDEX IF NOT EXISTS idx_pipe_start_node
ON public.pipe_info(region_id, version_id, start_node_id);
CREATE INDEX IF NOT EXISTS idx_pipe_end_node
ON public.pipe_info(region_id, version_id, end_node_id);

CREATE TABLE IF NOT EXISTS public.ramp_info (
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    ramp_id             INTEGER NOT NULL,
    type                SMALLINT NOT NULL,
    length              REAL NOT NULL,
    ramp_width          REAL NULL,
    lane_number         INTEGER NOT NULL DEFAULT 1,
    lane_width          REAL NOT NULL DEFAULT 5,
    start_node_id       BIGINT NOT NULL,
    end_node_id         BIGINT NOT NULL,
    ramp_path           REAL[] NOT NULL,
    PRIMARY KEY (region_id, version_id, ramp_id),
    CONSTRAINT fk_ramp_topology_version
        FOREIGN KEY (region_id, version_id)
        REFERENCES public.topology_version(region_id, version_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT fk_ramp_start_node
        FOREIGN KEY (region_id, version_id, start_node_id)
        REFERENCES public.node_info(region_id, version_id, node_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_ramp_end_node
        FOREIGN KEY (region_id, version_id, end_node_id)
        REFERENCES public.node_info(region_id, version_id, node_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT ck_ramp_type CHECK (type IN (0,1,2)),
    CONSTRAINT ck_ramp_length CHECK (length > 0),
    CONSTRAINT ck_ramp_width CHECK (ramp_width IS NULL OR ramp_width > 0),
    CONSTRAINT ck_ramp_lane_number CHECK (lane_number > 0),
    CONSTRAINT ck_ramp_lane_width CHECK (lane_width > 0),
    CONSTRAINT ck_ramp_path_nonempty CHECK (cardinality(ramp_path) >= 6),
    CONSTRAINT ck_ramp_distinct_nodes CHECK (start_node_id <> end_node_id)
);

COMMENT ON TABLE public.ramp_info IS '最终保留的有向引路静态拓扑表';
COMMENT ON COLUMN public.ramp_info.type IS '0同层反向,1跨层,2漏斗';
COMMENT ON COLUMN public.ramp_info.ramp_width IS '引路直径/宽度，当前预留可空';
COMMENT ON COLUMN public.ramp_info.lane_number IS '当前默认1';
COMMENT ON COLUMN public.ramp_info.lane_width IS '当前默认5m';
COMMENT ON COLUMN public.ramp_info.ramp_path IS '三维路径采样点数组';

CREATE INDEX IF NOT EXISTS idx_ramp_start_node
ON public.ramp_info(region_id, version_id, start_node_id);
CREATE INDEX IF NOT EXISTS idx_ramp_end_node
ON public.ramp_info(region_id, version_id, end_node_id);

CREATE TABLE IF NOT EXISTS public.lane_info (
    region_id       INTEGER NOT NULL,
    version_id      BIGINT NOT NULL,
    lane_id         INTEGER NOT NULL,
    layer_id        INTEGER NOT NULL,
    pipe_id         INTEGER NOT NULL,
    length          REAL NOT NULL,
    start_pt        REAL[] NOT NULL,
    end_pt          REAL[] NOT NULL,
    PRIMARY KEY (region_id, version_id, lane_id),
    CONSTRAINT fk_lane_layer
        FOREIGN KEY (region_id, version_id, layer_id)
        REFERENCES public.layer_info(region_id, version_id, layer_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT fk_lane_pipe
        FOREIGN KEY (region_id, version_id, pipe_id)
        REFERENCES public.pipe_info(region_id, version_id, pipe_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_lane_length CHECK (length > 0),
    CONSTRAINT ck_lane_start_pt_len CHECK (cardinality(start_pt) = 3),
    CONSTRAINT ck_lane_end_pt_len CHECK (cardinality(end_pt) = 3)
);

COMMENT ON TABLE public.lane_info IS 'pipe segment内部lane几何信息';
COMMENT ON COLUMN public.lane_info.start_pt IS '[x,y,z]';
COMMENT ON COLUMN public.lane_info.end_pt IS '[x,y,z]';

CREATE INDEX IF NOT EXISTS idx_lane_pipe
ON public.lane_info(region_id, version_id, pipe_id);

CREATE TABLE IF NOT EXISTS public.pipe_status (
    time                TIMESTAMPTZ NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    pipe_id             INTEGER NOT NULL,
    is_open             BOOLEAN NOT NULL DEFAULT TRUE,
    d_safe              REAL NOT NULL DEFAULT 20,
    speed_limit         REAL NOT NULL DEFAULT 15,
    max_capacity        INTEGER NOT NULL DEFAULT 50,
    PRIMARY KEY (time, region_id, version_id, pipe_id),
    CONSTRAINT fk_pipe_status_pipe
        FOREIGN KEY (region_id, version_id, pipe_id)
        REFERENCES public.pipe_info(region_id, version_id, pipe_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_pipe_status_dsafe CHECK (d_safe >= 0),
    CONSTRAINT ck_pipe_status_speed_limit CHECK (speed_limit >= 0),
    CONSTRAINT ck_pipe_status_capacity CHECK (max_capacity >= 0)
);

COMMENT ON TABLE public.pipe_status IS 'pipe运行状态时序表';

CREATE TABLE IF NOT EXISTS public.ramp_status (
    time                TIMESTAMPTZ NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    ramp_id             INTEGER NOT NULL,
    is_open             BOOLEAN NOT NULL DEFAULT TRUE,
    PRIMARY KEY (time, region_id, version_id, ramp_id),
    CONSTRAINT fk_ramp_status_ramp
        FOREIGN KEY (region_id, version_id, ramp_id)
        REFERENCES public.ramp_info(region_id, version_id, ramp_id)
        ON UPDATE CASCADE ON DELETE CASCADE
);

COMMENT ON TABLE public.ramp_status IS 'ramp运行状态时序表';

CREATE TABLE IF NOT EXISTS public.pipe_flow (
    time                TIMESTAMPTZ NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    pipe_id             INTEGER NOT NULL,
    uav_number          INTEGER NOT NULL DEFAULT 0,
    flow                REAL NOT NULL DEFAULT 0,
    density             REAL NOT NULL DEFAULT 0,
    average_speed       REAL NOT NULL DEFAULT 0,
    PRIMARY KEY (time, region_id, version_id, pipe_id),
    CONSTRAINT fk_pipe_flow_pipe
        FOREIGN KEY (region_id, version_id, pipe_id)
        REFERENCES public.pipe_info(region_id, version_id, pipe_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_pipe_flow_nonnegative
        CHECK (uav_number >= 0 AND flow >= 0 AND density >= 0 AND average_speed >= 0)
);

COMMENT ON TABLE public.pipe_flow IS 'pipe交通流统计时序表';

CREATE TABLE IF NOT EXISTS public.ramp_flow (
    time                TIMESTAMPTZ NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    ramp_id             INTEGER NOT NULL,
    uav_number          INTEGER NOT NULL DEFAULT 0,
    flow                REAL NOT NULL DEFAULT 0,
    density             REAL NOT NULL DEFAULT 0,
    average_speed       REAL NOT NULL DEFAULT 0,
    PRIMARY KEY (time, region_id, version_id, ramp_id),
    CONSTRAINT fk_ramp_flow_ramp
        FOREIGN KEY (region_id, version_id, ramp_id)
        REFERENCES public.ramp_info(region_id, version_id, ramp_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_ramp_flow_nonnegative
        CHECK (uav_number >= 0 AND flow >= 0 AND density >= 0 AND average_speed >= 0)
);

COMMENT ON TABLE public.ramp_flow IS 'ramp交通流统计时序表';

CREATE TABLE IF NOT EXISTS public.lane_flow (
    time                TIMESTAMPTZ NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    lane_id             INTEGER NOT NULL,
    pipe_id             INTEGER NOT NULL,
    uav_number          INTEGER NOT NULL DEFAULT 0,
    flow                REAL NOT NULL DEFAULT 0,
    density             REAL NOT NULL DEFAULT 0,
    average_speed       REAL NOT NULL DEFAULT 0,
    PRIMARY KEY (time, region_id, version_id, lane_id),
    CONSTRAINT fk_lane_flow_lane
        FOREIGN KEY (region_id, version_id, lane_id)
        REFERENCES public.lane_info(region_id, version_id, lane_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT fk_lane_flow_pipe
        FOREIGN KEY (region_id, version_id, pipe_id)
        REFERENCES public.pipe_info(region_id, version_id, pipe_id)
        ON UPDATE CASCADE ON DELETE CASCADE,
    CONSTRAINT ck_lane_flow_nonnegative
        CHECK (uav_number >= 0 AND flow >= 0 AND density >= 0 AND average_speed >= 0)
);

COMMENT ON TABLE public.lane_flow IS 'pipe内部lane交通流统计时序表';

CREATE TABLE IF NOT EXISTS public.uav_info (
    uav_id              INTEGER PRIMARY KEY,
    source              REAL[] NOT NULL,
    destination         REAL[] NOT NULL,
    CONSTRAINT ck_uav_info_source_len CHECK (cardinality(source) = 3),
    CONSTRAINT ck_uav_info_destination_len CHECK (cardinality(destination) = 3)
);

COMMENT ON TABLE public.uav_info IS 'UAV低频基本/任务信息，本区域可保存当前涉及UAV的本地副本';
COMMENT ON COLUMN public.uav_info.source IS '任务起始坐标[x,y,z]';
COMMENT ON COLUMN public.uav_info.destination IS '任务目标坐标[x,y,z]';

CREATE TABLE IF NOT EXISTS public.uav_data (
    time                TIMESTAMPTZ NOT NULL,
    uav_id              INTEGER NOT NULL,
    region_id           INTEGER NOT NULL,
    version_id          BIGINT NOT NULL,
    layer_id            INTEGER NULL,
    edge_type           SMALLINT NOT NULL,
    pipe_id             INTEGER NULL,
    lane_id             INTEGER NULL,
    ramp_id             INTEGER NULL,
    node_id             BIGINT NULL,
    distance_on_edge    REAL NULL,
    speed               REAL NULL,
    acceleration        REAL NULL,
    position            REAL[] NOT NULL,
    status              SMALLINT NOT NULL DEFAULT 0,
    PRIMARY KEY (time, uav_id),
    CONSTRAINT fk_uav_data_uav
        FOREIGN KEY (uav_id) REFERENCES public.uav_info(uav_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_uav_data_topology_version
        FOREIGN KEY (region_id, version_id)
        REFERENCES public.topology_version(region_id, version_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_uav_data_pipe
        FOREIGN KEY (region_id, version_id, pipe_id)
        REFERENCES public.pipe_info(region_id, version_id, pipe_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_uav_data_lane
        FOREIGN KEY (region_id, version_id, lane_id)
        REFERENCES public.lane_info(region_id, version_id, lane_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_uav_data_ramp
        FOREIGN KEY (region_id, version_id, ramp_id)
        REFERENCES public.ramp_info(region_id, version_id, ramp_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT fk_uav_data_node
        FOREIGN KEY (region_id, version_id, node_id)
        REFERENCES public.node_info(region_id, version_id, node_id)
        ON UPDATE CASCADE ON DELETE RESTRICT,
    CONSTRAINT ck_uav_data_edge_type CHECK (edge_type IN (0,1,2)),
    CONSTRAINT ck_uav_data_edge_reference CHECK (
        (edge_type = 0 AND pipe_id IS NOT NULL AND ramp_id IS NULL AND node_id IS NULL)
        OR
        (edge_type = 1 AND pipe_id IS NULL AND lane_id IS NULL AND ramp_id IS NOT NULL AND node_id IS NULL)
        OR
        (edge_type = 2 AND pipe_id IS NULL AND lane_id IS NULL AND ramp_id IS NULL AND node_id IS NOT NULL)
    ),
    CONSTRAINT ck_uav_data_distance CHECK (distance_on_edge IS NULL OR distance_on_edge >= 0),
    CONSTRAINT ck_uav_data_speed_len CHECK (speed IS NULL OR speed >= 0),
    CONSTRAINT ck_uav_data_position_len CHECK (cardinality(position) = 3)
);

COMMENT ON TABLE public.uav_data IS '区域DT使用的UAV微观实时状态时序表';
COMMENT ON COLUMN public.uav_data.edge_type IS '0 pipe,1 ramp,2 node';
COMMENT ON COLUMN public.uav_data.distance_on_edge IS '沿当前pipe/ramp飞行方向距edge起点距离(m)';
COMMENT ON COLUMN public.uav_data.status IS '区域DT运行状态码,未定保留';

CREATE TABLE IF NOT EXISTS public.flow_prediction (
    region_id           INTEGER PRIMARY KEY,
    result              REAL[] NOT NULL,
    CONSTRAINT fk_flow_prediction_region
        FOREIGN KEY (region_id) REFERENCES public.region_info(region_id)
        ON UPDATE CASCADE ON DELETE CASCADE
);

COMMENT ON TABLE public.flow_prediction IS '区域级流量预测结果矩阵';

CREATE TABLE IF NOT EXISTS public.flow_guidance (
    region_id           INTEGER PRIMARY KEY,
    result              REAL[] NOT NULL,
    CONSTRAINT fk_flow_guidance_region
        FOREIGN KEY (region_id) REFERENCES public.region_info(region_id)
        ON UPDATE CASCADE ON DELETE CASCADE
);

COMMENT ON TABLE public.flow_guidance IS '中心DT下发的区域级流量导引结果矩阵';

-- TimescaleDB hypertables
SELECT create_hypertable('public.pipe_status', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);
SELECT create_hypertable('public.ramp_status', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);
SELECT create_hypertable('public.pipe_flow', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);
SELECT create_hypertable('public.ramp_flow', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);
SELECT create_hypertable('public.lane_flow', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);
SELECT create_hypertable('public.uav_data', 'time', chunk_time_interval => INTERVAL '1 hour', if_not_exists => TRUE, migrate_data => TRUE);

-- 常用查询索引
CREATE INDEX IF NOT EXISTS idx_pipe_status_latest
ON public.pipe_status(region_id, version_id, pipe_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_ramp_status_latest
ON public.ramp_status(region_id, version_id, ramp_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_pipe_flow_latest
ON public.pipe_flow(region_id, version_id, pipe_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_ramp_flow_latest
ON public.ramp_flow(region_id, version_id, ramp_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_lane_flow_latest
ON public.lane_flow(region_id, version_id, lane_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_uav_data_latest
ON public.uav_data(uav_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_uav_data_region_time
ON public.uav_data(region_id, time DESC);

CREATE INDEX IF NOT EXISTS idx_uav_data_pipe_time
ON public.uav_data(region_id, version_id, pipe_id, time DESC)
WHERE pipe_id IS NOT NULL;

CREATE INDEX IF NOT EXISTS idx_uav_data_ramp_time
ON public.uav_data(region_id, version_id, ramp_id, time DESC)
WHERE ramp_id IS NOT NULL;

-- 当前版本 / 最新状态辅助视图
CREATE OR REPLACE VIEW public.current_topology_version AS
SELECT region_id, version_id, valid_from, reason
FROM public.topology_version
WHERE valid_to IS NULL;

CREATE OR REPLACE VIEW public.latest_pipe_status AS
SELECT DISTINCT ON (region_id, version_id, pipe_id)
       time, region_id, version_id, pipe_id,
       is_open, d_safe, speed_limit, max_capacity
FROM public.pipe_status
ORDER BY region_id, version_id, pipe_id, time DESC;

CREATE OR REPLACE VIEW public.latest_ramp_status AS
SELECT DISTINCT ON (region_id, version_id, ramp_id)
       time, region_id, version_id, ramp_id, is_open
FROM public.ramp_status
ORDER BY region_id, version_id, ramp_id, time DESC;

COMMIT;

-- SELECT add_retention_policy('public.pipe_status', INTERVAL '17 hours', if_not_exists => TRUE);
-- SELECT add_retention_policy('public.ramp_status', INTERVAL '17 hours', if_not_exists => TRUE);
-- SELECT add_retention_policy('public.pipe_flow',   INTERVAL '17 hours', if_not_exists => TRUE);
-- SELECT add_retention_policy('public.ramp_flow',   INTERVAL '17 hours', if_not_exists => TRUE);
-- SELECT add_retention_policy('public.lane_flow',   INTERVAL '17 hours', if_not_exists => TRUE);
-- SELECT add_retention_policy('public.uav_data',    INTERVAL '17 hours', if_not_exists => TRUE);
