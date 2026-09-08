# Optimal-Time Trajectory Planning for Robotic Systems with Smooth Jerk
### MS Thesis Research — Electrical & Computer Engineering, Purdue University Northwest

![License](https://img.shields.io/badge/license-CC%20BY%204.0-blue)
![Platform](https://img.shields.io/badge/Platform-UR3%20%7C%20myCobot%20280%20Pi-orange)
![Algorithm](https://img.shields.io/badge/Algorithm-W--MOPSO-green)

## Overview

Industrial trajectory planners face a tradeoff between speed and smoothness: pushing execution time down tends to push jerk up, and high jerk is what wears out joints and gearboxes. This repo hosts the code behind my MS thesis, which optimizes both at once — time and jerk — for a 6-DOF manipulator, and validates the result on real hardware rather than stopping at simulation.

## The Problem That Started This

Both the UR3 and the myCobot 280 Pi showed visibly jerky motion that never appeared in the MATLAB simulation. The cause: an encoder baud-rate ceiling on the physical controllers, capping how fast joint-angle updates could actually be reported and executed — a hardware constraint the simulation didn't model. The fix was modifying the trajectory solver's main loop to respect that update-rate ceiling.

This is the finding that shaped the rest of the project: a trajectory that looks optimal in MATLAB isn't optimal until it's been run on the actual hardware.

## Approach

The core contribution is a **Weighted Multi-Objective Particle Swarm Optimization (W-MOPSO)** algorithm that generates **Gaussian S-curve trajectories** for a 6-DOF manipulator, jointly optimizing execution time and jerk.

- IK solver: 150 particles / 300 iterations
- W-MOPSO time-optimization: 30 particles / 50–100 iterations
- Benchmarked against two standard trajectory profiles used as baselines — **3-5-3** and **6-7-6** segmented polynomials — across 5 spatial test configurations: Right-Side Sweep, Left-Side Sweep, Overhead Tumble, and 2 more (names TBC)

**Note on scope:** the W-MOPSO / Gaussian S-curve code is still being cleaned up for upload (see Roadmap). What's in this repo today is the 6-7-6 polynomial **baseline** used for that comparison — not the novel algorithm itself.

## What's Actually In This Repo Right Now

- **`src/Polynomial_676_Generator.m`** — builds a 6-7-6 segmented polynomial trajectory (degree-6 → degree-7 → degree-6) across 4 via points, solved as a 22×22 linear system per joint enforcing position/velocity/acceleration/jerk continuity at each segment boundary. Solves IK per waypoint with a PSO-based solver (`ilink6dofelephant`), generates position/velocity/acceleration/jerk/snap profiles, cross-checks the result with forward kinematics against the desired Cartesian path, and tracks end-effector orientation error (Frobenius norm of the rotation matrix) to confirm the gripper holds its target down-facing orientation throughout the motion.
- **`src/Polynomial_676_GeneratorTester.m`** — the same 6-7-6 trajectory, but with IK solved via warm-start chaining (each waypoint seeded from the previous joint solution) plus a total joint-travel cost check.

This is baseline/comparison code — see Roadmap for what's still to be added.

## Results (from the full thesis study)

- **10–20% reduction** in execution time vs. the 3-5-3 / 6-7-6 baselines, across the 5 test configurations
- **Tradeoff:** 15% of motions were *longer* after optimization — the algorithm isn't a strict win on every trial, it trades some execution time for reduced mechanical wear on the joints
- Jerk **mathematically bounded below 200 deg/s³** — this is a guaranteed design bound built into the optimization, not an empirically observed reduction
- Hardware-in-the-loop validation on the myCobot 280 Pi via Python/ROS2, following MATLAB simulation

## Hardware

- **UR3** — where the encoder baud-rate limitation was diagnosed
- **myCobot 280 Pi** — used for hardware-in-the-loop validation via ROS2/Python

## Repository Structure (current)

```text
├── assets/                              # Figures (more to come — jerk/velocity plots)
├── publications/
│   └── southeastcon-2025/               # Paper + slides for the SoutheastCon 2025 publication
├── src/                                 # MATLAB source: 6-7-6 baseline trajectory generator, IK solver, FK verification
├── thesis/                              # Full thesis document — coming soon
├── LICENSE                              # CC BY 4.0
└── README.md
```

## Roadmap

- [ ] Add the W-MOPSO optimizer + Gaussian S-curve trajectory generator (the actual novel algorithm)
- [ ] Add the ROS2/Python hardware-in-the-loop validation code
- [ ] Add jerk/velocity smoothness plots to `assets/`
- [ ] Add the thesis PDF to `thesis/`

## Related Publications

- **T. Chinyai** (lead author), L. Tan, M. Yao. "Time-Optimal Trajectory Planning for Robotic Manipulators with Differentiable Jerk Minimization." *IEEE UEMCON 2025.*
- X. Zhou (lead author), **T. Chinyai** (experiment setup & results validation), L. Tan. "Multi-objective optimization for inverse kinematics of robotic arm with joint configuration." *IEEE SoutheastCon 2025*, pp. 514–519. → [paper](publications/southeastcon-2025/paper.pdf) · [slides](publications/southeastcon-2025/slides.pdf) (slides authored/presented by X. Zhou)
- MS Thesis: *"Optimal-Time Trajectory Planning for Robotic Systems with Smooth Jerk,"* Purdue University Northwest.

## License

CC BY 4.0 — see [LICENSE](LICENSE).
