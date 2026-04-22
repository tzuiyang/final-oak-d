#!/usr/bin/env python3
"""
Deployment entry point on the Pupper.

Brings up the full stack with one command:
  1. Sources the upstream Pupper v3 workspace (read-only — never writes to
     /home/pi/pupperv3-monorepo, so it's safe to share with other teams).
  2. Launches `neural_controller launch.py` in the background.
  3. Waits a few seconds for ros2_control + policy fade-in.
  4. Launches our `oakd.launch.py` in the foreground.
  5. On Ctrl+C (or if our stack exits), kills the upstream cleanly.

Flags:
  --ours       Only launch our stack. Use while iterating on our code when
               the upstream is already running in another terminal.
  --upstream   Only launch upstream. Useful for running the stock Pupper
               stack without any of our nodes.

Environment:
  PUPPER_WS    Path to the upstream ROS 2 workspace root.
               Default: /home/pi/pupperv3-monorepo/ros2_ws
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
BLOB_PATH = SCRIPT_DIR / "models" / "yolov8n_coco_640x352.blob"
LAUNCH_FILE = SCRIPT_DIR / "oakd.launch.py"

DEFAULT_PUPPER_WS = "/home/pi/pupperv3-monorepo/ros2_ws"
UPSTREAM_INIT_DELAY_S = 5.0


def ensure_model() -> bool:
    if BLOB_PATH.exists():
        return True
    print(f"Model blob missing at {BLOB_PATH}. Downloading...")
    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / "download_model.py")],
        cwd=str(SCRIPT_DIR),
    )
    return result.returncode == 0 and BLOB_PATH.exists()


def _sourced(setup_file: Path, inner: str) -> list[str]:
    """Wrap a command so it runs after sourcing the given setup.bash."""
    return ["bash", "-c", f"source {setup_file} && exec {inner}"]


def _resolve_setup(pupper_ws: Path) -> Path:
    setup = pupper_ws / "install" / "setup.bash"
    if not setup.exists():
        print(f"Upstream setup.bash not found at {setup}.")
        print("Either build the pupper workspace or set PUPPER_WS correctly.")
        sys.exit(1)
    return setup


def start_upstream(pupper_ws: Path) -> subprocess.Popen:
    setup = _resolve_setup(pupper_ws)
    cmd = _sourced(setup, "ros2 launch neural_controller launch.py")
    print(f"[deploy] Launching upstream neural_controller from {pupper_ws}")
    # Separate process group so Ctrl+C on us doesn't kill it implicitly —
    # we handle its shutdown explicitly in stop_upstream().
    return subprocess.Popen(cmd, start_new_session=True)


def run_our_stack(pupper_ws: Path) -> int:
    setup = pupper_ws / "install" / "setup.bash"
    inner = f"ros2 launch {LAUNCH_FILE}"
    cmd = _sourced(setup, inner) if setup.exists() else ["ros2", "launch", str(LAUNCH_FILE)]
    print(f"[deploy] Launching our stack ({LAUNCH_FILE.name})")
    return subprocess.run(cmd).returncode


def stop_upstream(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    print("[deploy] Stopping upstream neural_controller...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    print("[deploy] Upstream did not exit after SIGINT — sending SIGTERM")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--ours", action="store_true",
        help="Only launch our stack (upstream assumed already running).",
    )
    parser.add_argument(
        "--upstream", action="store_true",
        help="Only launch upstream (no OAK-D or UI).",
    )
    args = parser.parse_args()

    if args.ours and args.upstream:
        print("--ours and --upstream are mutually exclusive.")
        return 2

    pupper_ws = Path(os.environ.get("PUPPER_WS", DEFAULT_PUPPER_WS))

    if args.upstream:
        upstream = start_upstream(pupper_ws)
        try:
            return upstream.wait()
        except KeyboardInterrupt:
            return 0
        finally:
            stop_upstream(upstream)

    if not ensure_model():
        print("Model download failed. Aborting.")
        return 1

    if args.ours:
        return run_our_stack(pupper_ws)

    upstream = start_upstream(pupper_ws)
    try:
        print(f"[deploy] Waiting {UPSTREAM_INIT_DELAY_S:.0f}s for upstream fade-in...")
        time.sleep(UPSTREAM_INIT_DELAY_S)
        if upstream.poll() is not None:
            print(f"[deploy] Upstream exited early with code {upstream.returncode}.")
            return upstream.returncode or 1
        return run_our_stack(pupper_ws)
    finally:
        stop_upstream(upstream)


if __name__ == "__main__":
    sys.exit(main())
