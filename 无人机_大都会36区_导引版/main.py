"""Compatibility entry point for running the UAV simulation from the repo root."""

from pathlib import Path
import os
import sys


ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
os.environ["PYTHONPATH"] = str(SRC) + (
    os.pathsep + os.environ["PYTHONPATH"]
    if os.environ.get("PYTHONPATH")
    else ""
)

from uav_sim.cli import main  # noqa: E402


if __name__ == "__main__":
    main()
