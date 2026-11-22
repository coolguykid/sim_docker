FROM airway-region-dt:dev

ENV PYTHONPATH="/opt/coordinator_service"

COPY coordinator_service /opt/coordinator_service

ENTRYPOINT []
EXPOSE 8080
CMD ["uvicorn", "app:app", "--app-dir", "/opt/coordinator_service", "--host", "0.0.0.0", "--port", "8080", "--timeout-keep-alive", "300"]
