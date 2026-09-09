#!/usr/bin/env python3
"""
stream_csv_trajectory.py
Stream an interpolated CSV of joint angles to myCobot smoothly (no waypoint stops).

Assumptions:
 - CSV file: rows = timesteps, columns = 6 (joint1..joint6) in degrees.
 - Running locally on robot; serial device is /dev/ttyAMA0.
 - pymycobot.mycobot280.MyCobot280 is available (install pymycobot in venv).

Usage:
    # default (uses smooth_angles.csv in current folder)
    python stream_csv_trajectory.py

    # or specify a file explicitly
    python stream_csv_trajectory.py path/to/your_angles.csv
"""
import sys
import time
from pathlib import Path
import numpy as np
from pymycobot.mycobot280 import MyCobot280

# ---------------- User-configurable parameters ----------------
# Default CSV file in same folder (as requested)
CSV_PATH = "smooth_angles.csv"         # default filename (overridable via CLI)
SERIAL_PORT = "/dev/ttyAMA0"
BAUD = 1000000

# Playback timing
T_DESIRED = 10.0                   # desired total motion time (seconds)
MIN_DT = 0.01                      # minimum send interval (seconds) to avoid flooding
DEFAULT_SPEED = 60                 # fallback speed if motion magnitude is zero (1..100)

# Joint limits (degrees) — replace by your robot's exact limits if known
JOINT_LIMITS = np.array([
    [-180.0, 180.0],
    [ -90.0,  90.0],
    [-150.0, 150.0],
    [-150.0, 150.0],
    [-120.0, 120.0],
    [-360.0, 360.0],
])

# Safety: maximum allowed API speed range
API_SPEED_MIN = 1
API_SPEED_MAX = 100

# Debug / logging
VERBOSE = True
# -------------------------------------------------------------


def load_csv_angles(path: Path) -> np.ndarray:
    # Accept CSV with commas, optional header, whitespace tolerant.
    try:
        data = np.genfromtxt(str(path), delimiter=",")
    except Exception:
        data = np.loadtxt(str(path), delimiter=",")
    if data.ndim == 1:
        data = data[np.newaxis, :]
    if data.shape[1] != 6:
        raise ValueError(f"CSV must have 6 columns (j1..j6). Found shape: {data.shape}")
    return data.astype(float)  # shape: (N_steps, 6)


def check_and_clip_joint_limits(traj: np.ndarray):
    # traj: (N,6)
    clipped = False
    for j in range(6):
        low, high = JOINT_LIMITS[j]
        below = traj[:, j] < low
        above = traj[:, j] > high
        if np.any(below) or np.any(above):
            clipped = True
            traj[below, j] = low
            traj[above, j] = high
            if VERBOSE:
                print(f"[WARN] Joint {j+1}: {np.sum(below)+np.sum(above)} points clipped to limits [{low},{high}]")
    return traj, clipped


def compute_api_speeds_from_trajectory(traj: np.ndarray, dt: float) -> np.ndarray:
    """
    Map required deg/sec -> API speed (1..100).
    Strategy:
      - compute max joint delta per step (deg)
      - required_deg_per_sec = max_delta / dt
      - scale required_deg_per_sec linearly so that the maximum observed maps to 100
      - clip to [1,100]
    """
    deltas = np.abs(np.diff(traj, axis=0))  # shape (N-1,6)
    deltas = np.vstack([np.zeros((1, 6)), deltas])  # shape (N,6)
    max_deltas = np.max(deltas, axis=1)  # deg per step (N,)
    required_deg_per_sec = np.divide(max_deltas, dt, out=np.zeros_like(max_deltas), where=dt > 0)

    max_req = required_deg_per_sec.max()
    if max_req <= 0:
        return np.ones(len(required_deg_per_sec), dtype=int) * DEFAULT_SPEED

    speeds = 1.0 + (required_deg_per_sec / max_req) * (API_SPEED_MAX - 1.0)
    speeds = np.round(speeds).astype(int)
    speeds = np.clip(speeds, API_SPEED_MIN, API_SPEED_MAX)
    return speeds


def stream_trajectory(traj: np.ndarray, speeds: np.ndarray, dt: float):
    """
    Streams trajectory rows to robot using mc.send_angles(row, speed) paced by dt.
    """
    n_steps = traj.shape[0]
    mc = MyCobot280(SERIAL_PORT, BAUD)
    time.sleep(0.05)  # let serial warm up

    try:
        cur = mc.get_angles()
        if VERBOSE:
            print("Robot current angles (deg):", cur)
    except Exception:
        if VERBOSE:
            print("Warning: could not read current robot angles.")

    t_start = time.perf_counter()
    try:
        for i in range(n_steps):
            q = traj[i].tolist()
            s = int(speeds[i])
            mc.send_angles(q, s)
            if (i % 50) == 0 and VERBOSE:
                elapsed = time.perf_counter() - t_start
                print(f"[{i}/{n_steps}] sent speed={s}, elapsed={elapsed:.3f}s")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[ABORT] KeyboardInterrupt received — stopping stream safely.")
    finally:
        elapsed_total = time.perf_counter() - t_start
        if VERBOSE:
            print(f"Stream finished (send-loop elapsed {elapsed_total:.3f}s).")
        time.sleep(0.2)


def main():
    global CSV_PATH
    if len(sys.argv) >= 2:
        CSV_PATH = sys.argv[1]
    if not CSV_PATH:
        print("Usage: python stream_csv_trajectory.py path/to/angles.csv")
        sys.exit(1)
    path = Path(CSV_PATH)
    if not path.exists():
        print(f"Error: file not found: {path}")
        sys.exit(1)

    traj = load_csv_angles(path)      # shape (N,6)

    n_steps = traj.shape[0]
    if VERBOSE:
        print(f"Loaded trajectory: {n_steps} timesteps, {traj.shape[1]} joints.")

    traj, clipped = check_and_clip_joint_limits(traj)

    dt_target = T_DESIRED / max(n_steps, 1)
    dt = max(dt_target, MIN_DT)
    if VERBOSE:
        print(f"Timing: target total {T_DESIRED}s -> dt_target={dt_target:.4f}s; using dt={dt:.4f}s (min_dt={MIN_DT})")

    speeds = compute_api_speeds_from_trajectory(traj, dt)
    if VERBOSE:
        print("Sample speeds (first 10):", speeds[:10])

    stream_trajectory(traj, speeds, dt)


if __name__ == "__main__":
    main()
