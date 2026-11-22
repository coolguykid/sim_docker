#!/bin/sh
set -eu

data_dir="${REGION_DATA_DIR:-/region_data/current}"
if [ ! -f "$data_dir/region_info.csv" ]; then
    echo "Missing regional data: $data_dir/region_info.csv" >&2
    exit 1
fi
csv_region_id="$(awk -F, 'NR==2 {gsub(/\r/,"",$1); print $1}' "$data_dir/region_info.csv")"
if [ "$csv_region_id" != "$REGION_ID" ]; then
    echo "Regional data mismatch: container=$REGION_ID csv=$csv_region_id" >&2
    exit 1
fi
existing="$(psql -h 127.0.0.1 -U "$POSTGRES_USER" -d "$POSTGRES_DB" -Atc 'SELECT COUNT(*) FROM node_info')"
if [ "$existing" -gt 0 ]; then
    echo "Region $REGION_ID database already contains topology data; import skipped."
    exit 0
fi
echo "Importing region $REGION_ID data from $data_dir..."
psql -h 127.0.0.1 -U "$POSTGRES_USER" -d "$POSTGRES_DB" -v ON_ERROR_STOP=1 <<SQL
\copy region_info from '$data_dir/region_info.csv' WITH (FORMAT csv, HEADER true);
\copy topology_version from '$data_dir/topology_version.csv' WITH (FORMAT csv, HEADER true);
\copy layer_info from '$data_dir/layer_info.csv' WITH (FORMAT csv, HEADER true);
\copy node_info from '$data_dir/node_info.csv' WITH (FORMAT csv, HEADER true);
\copy pipe_info from '$data_dir/pipe_info.csv' WITH (FORMAT csv, HEADER true);
\copy pipe_status from '$data_dir/pipe_status.csv' WITH (FORMAT csv, HEADER true);
\copy ramp_info from '$data_dir/ramp_info.csv' WITH (FORMAT csv, HEADER true);
\copy ramp_status from '$data_dir/ramp_status.csv' WITH (FORMAT csv, HEADER true);
\copy lane_info from '$data_dir/lane_info.csv' WITH (FORMAT csv, HEADER true);
SQL
