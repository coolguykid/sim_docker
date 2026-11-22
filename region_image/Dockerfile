FROM timescale/timescaledb:latest-pg17

RUN apk add --no-cache python3 py3-pip curl \
    && python3 -m venv /opt/region-venv \
    && /opt/region-venv/bin/pip install --no-cache-dir \
       "fastapi>=0.115,<1" \
       "uvicorn>=0.30,<1" \
       "httpx>=0.28,<1" \
       "numpy>=2,<3" \
       "psycopg[binary]>=3.2,<4"

ENV PATH="/opt/region-venv/bin:${PATH}" \
    PYTHONPATH="/opt/region_service:/opt/uav_sim_src:/opt/simulation_engine_parent:/opt/airway_capacity"

COPY region_service /opt/region_service
COPY simulation_engine /opt/simulation_engine_parent/simulation_engine
# Wildcards avoid depending on the host code page for non-ASCII delivery names.
COPY ["*36*/src/uav_sim", "/opt/uav_sim_src/uav_sim"]
COPY ["*/airway", "/opt/airway_capacity/airway"]
COPY region_image/entrypoint.sh /usr/local/bin/region-dt-entrypoint
COPY region_image/import_region_data.sh /usr/local/bin/region-dt-import-data
RUN chmod +x /usr/local/bin/region-dt-entrypoint /usr/local/bin/region-dt-import-data

EXPOSE 5432 8080
ENTRYPOINT ["/usr/local/bin/region-dt-entrypoint"]
