"""Serve the standalone comparison and aggregate regional CSV replay frames."""
import argparse
import csv
from functools import lru_cache, partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
from urllib.parse import parse_qs, urlsplit

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parents[1] / "output"
REGIONS = range(1, 36)
RUN_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}")


@lru_cache(maxsize=256)
def read_frames(directory, timeline_size):
    with (directory / "frame_times.csv").open(encoding="utf-8-sig") as stream:
        raw = stream.buffer.read(timeline_size).decode("utf-8-sig")
    complete = raw[:raw.rfind("\n") + 1]
    times = {float(row["time"]): [0, 0.0, 0]
             for row in csv.DictReader(complete.splitlines())}
    with (directory / "uav_states.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.reader(stream)
        header = next(rows, None)
        if not header:
            return {}
        ti, active, speed = (header.index(name) for name in ("time", "active", "current_speed"))
        for row in rows:
            if len(row) != len(header):
                continue
            time = float(row[ti])
            if time not in times or row[active] != "1":
                continue
            values = times[time]
            values[0] += 1
            if row[speed]:
                velocity = float(row[speed])
                if math.isfinite(velocity):
                    values[1] += velocity
                    values[2] += 1
    return times


def series(run, attempt, regions):
    inputs = []
    for region in regions:
        directory = OUTPUT / f"region_{region:02d}" / run / f"attempt_{attempt:03d}"
        timeline = directory / "frame_times.csv"
        if not timeline.is_file() or not (directory / "uav_states.csv").is_file():
            return []
        inputs.append(read_frames(directory, timeline.stat().st_size))
    common_times = set(inputs[0]).intersection(*inputs[1:])
    points = []
    for time in sorted(common_times):
        counts, sums, samples = zip(*(frames[time] for frames in inputs))
        size = sum(samples)
        points.append([time, sum(counts), sum(sums) / size if size else None])
    return points


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/api/runs":
            root = OUTPUT / "region_01"
            names = sorted((p.name for p in root.iterdir() if p.is_dir()
                            and RUN_NAME.fullmatch(p.name)
                            and (p / "attempt_000/frame_times.csv").is_file()), reverse=True)
            self.send_json(names)
        elif url.path == "/api/comparison":
            query = parse_qs(url.query)
            run = query.get("run", [""])[0]
            region = query.get("region", ["all"])[0]
            if not RUN_NAME.fullmatch(run) or region not in {"all", *map(str, REGIONS)}:
                self.send_error(400)
                return
            regions = REGIONS if region == "all" else (int(region),)
            try:
                self.send_json([series(run, attempt, regions) for attempt in (0, 1)])
            except (OSError, ValueError) as error:
                self.send_error(503, str(error))
        else:
            super().do_GET()

    def send_json(self, value):
        data = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8768)
    args = parser.parse_args()
    ThreadingHTTPServer(("127.0.0.1", args.port), partial(Handler, directory=HERE)).serve_forever()
