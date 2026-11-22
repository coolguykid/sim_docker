#!/bin/sh
set -eu

postgres_pid=""
api_pid=""

shutdown() {
    [ -z "$api_pid" ] || kill -TERM "$api_pid" 2>/dev/null || true
    [ -z "$postgres_pid" ] || kill -TERM "$postgres_pid" 2>/dev/null || true
}
trap shutdown INT TERM EXIT

/usr/local/bin/docker-entrypoint.sh postgres &
postgres_pid=$!

echo "Waiting for the local PostgreSQL server..."
attempt=0
until pg_isready -h 127.0.0.1 -p 5432 -U "${POSTGRES_USER}" -d "${POSTGRES_DB}" >/dev/null 2>&1; do
    if ! kill -0 "$postgres_pid" 2>/dev/null; then
        wait "$postgres_pid"
        exit 1
    fi
    attempt=$((attempt + 1))
    if [ "$attempt" -ge 120 ]; then
        echo "PostgreSQL did not become ready" >&2
        exit 1
    fi
    sleep 1
done

region-dt-import-data

echo "Starting region DT service for region ${REGION_ID}..."
uvicorn app:app \
    --app-dir /opt/region_service \
    --host 0.0.0.0 \
    --port "${REGION_DT_PORT:-8080}" \
    --timeout-keep-alive "${REGION_DT_KEEPALIVE_SECONDS:-300}" &
api_pid=$!

while kill -0 "$postgres_pid" 2>/dev/null && kill -0 "$api_pid" 2>/dev/null; do
    sleep 1
done

if ! kill -0 "$postgres_pid" 2>/dev/null; then
    wait "$postgres_pid" || true
    echo "PostgreSQL stopped unexpectedly" >&2
else
    wait "$api_pid" || true
    echo "Region DT service stopped unexpectedly" >&2
fi
exit 1
