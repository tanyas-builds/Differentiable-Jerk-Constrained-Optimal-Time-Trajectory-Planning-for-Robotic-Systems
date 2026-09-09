#!/usr/bin/env python3
"""
deploy_trajectory.py
Stream a MATLAB-generated (.mat) joint-angle trajectory to the myCobot 280 Pi
over direct serial, at a controlled update rate.

This is the hardware-in-the-loop counterpart to the MATLAB planners in src/ —
it takes the joint-angle/velocity arrays those planners produce and actually
drives the physical arm, at a send rate capped to respect the controller's
encoder baud-rate ceiling (see README: "The Problem That Started This").

Assumptions:
 - Running locally on the robot (Raspberry Pi), serial device /dev/ttyAMA0
 - pymycobot is installed (`pip install pymycobot`)
 - .mat files contain 'q_matdeg' (joint angles, degrees) and a matching
   per-step speed array

Usage:
    python deploy_trajectory.py
"""
import time
from pymycobot import MyCobot280
from scipy.io import loadmat

# ---------------- Robot connection ----------------
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 1000000

mc = MyCobot280(SERIAL_PORT, BAUD_RATE)

# ---------------- Sanity move (home) ----------------
mc.send_angles([0, 0, 0, 0, 0, 0], 60)
time.sleep(2)

# ---------------- Load trajectory data ----------------
mat = loadmat('v2angles676.mat')
speedmat = loadmat('v2velocity676norm.mat')

angles_list = mat['q_matdeg'].T                  # shape: (n_steps, 6)
speeds_list = speedmat['v_matnorm'].flatten()     # shape: (n_steps,)

# ---------------- Stream trajectory ----------------
dt = 0.005  # base send interval in seconds

for q_matdeg, v_matnorm in zip(angles_list, speeds_list):
    speed = int(max(1, min(100, v_matnorm)))  # enforce pymycobot's [1,100] speed range
    mc.send_angles(list(q_matdeg), speed)
    time.sleep(dt)

print("Trajectory playback done.")
