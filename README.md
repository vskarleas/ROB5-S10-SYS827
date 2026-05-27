# ROB5-S10: SYS827 Robotique en contact

## Authors

* Dounia Bakalem
* Vasileios Filippos Skarleas
* Yanis Sadoun

---

## Project: Modelling, Dynamic Tracking and Impedance Control of a UR5 Robot

All project files are located in the **`Project/`** folder of this repository.

### Overview

This project, completed for the course **SYS827 — Systèmes robotiques en contact** at ÉTS (École de technologie supérieure), develops a complete modelling and control pipeline for a UR5 collaborative robot, from forward kinematics through to impedance-controlled contact tasks. The work was carried out entirely in MATLAB using the Robotics System Toolbox.

### Project Structure

The `Project/` folder contains a modular set of MATLAB scripts and functions organised as follows:

| File / Category | Description |
|---|---|
| `projet_final.m` | Main script — runs the full pipeline |
| `init_UR5.m` | Robot initialisation and URDF loading |
| `gripper.m` | Addition of the gripper rigid body to the model |
| `generate_*.m` | Trajectory generation (circular, vertical contact, circular contact) |
| `simulate_*.m` | Simulation functions (dynamic tracking, impedance control) |
| `plot_*.m` | Plotting and result visualisation |
| `draw_gripper.m` | Gripper visual rendering for animations |
| `draw_table_spring_*.m` | Table and environment visualisation |

### Topics Covered

The project is structured in progressive stages, each building on the previous:

1. **Forward Kinematics** — Modified Denavit–Hartenberg (Craig convention) parameterisation of the UR5, with symbolic and numerical derivation of transformation matrices. Validated against the MATLAB URDF model, the Universal Robots spreadsheet tool, and inverse kinematics cross-checks.

2. **Trajectory Tracking via Inverse Kinematics** — Cartesian trajectory (line + arc) followed point-by-point using `inverseKinematics`, with warm-starting for solution continuity.

3. **Differential Kinematics** — Analytical position Jacobian derived from the forward kinematics expressions. Cross-validated with a geometric Jacobian (cross-product method) and compared against `geometricJacobian` from the Robotics System Toolbox.

4. **Dynamic Model** — Extraction of the mass matrix M(q), gravity vector g(q), and Coriolis/centrifugal term h(q, q̇) via `massMatrix`, `gravityTorque`, and `velocityProduct`. Reconstructed torque verified against `inverseDynamics`.

5. **Gripper Addition** — A rigid body tool (`gripper_tip`) with realistic mass and inertia is attached to `tool0`, modifying the dynamic model and defining the new end-effector reference point.

6. **Dynamic Trajectory Tracking** — A PD controller in task space computes a desired Cartesian acceleration, mapped to joint space via the Jacobian pseudo-inverse. Forward simulation uses Euler integration of the full dynamic model. Demonstrated on a circular trajectory (R = 5 cm, 1 turn, 8 s).

7. **Impedance Control (Admittance Form)** — An outer admittance loop corrects the desired trajectory based on contact forces, feeding a corrected reference to the inner dynamic tracking loop.
   - *Vertical contact task*: descent onto a horizontal table modelled as a spring–damper (Kₑ = 200 N/m, Bₑ = 20 N·s/m).
   - *Circular contact task*: circular motion in the XY plane while maintaining normal contact along Z — representative of polishing, deburring, or precision inspection scenarios.

### Simulation Videos

- Trajectory tracking (IK): [https://youtu.be/Ylp86b_gF9Y](https://youtu.be/Ylp86b_gF9Y)
- Dynamic circular tracking: [https://youtu.be/QURmLeVS6kQ](https://youtu.be/QURmLeVS6kQ)
- Vertical impedance contact: [https://youtu.be/0ZEK1FUempo](https://youtu.be/0ZEK1FUempo)
- Circular impedance contact: [https://youtu.be/AD1ur_CWeTs](https://youtu.be/AD1ur_CWeTs)

### How to Run

1. Open MATLAB (R2025b or later recommended) with the **Robotics System Toolbox** installed.
2. Navigate to the `Project/` folder.
3. Run `projet_final.m` — it calls all initialisation, simulation, and plotting functions in sequence.

### Key Parameters

| Parameter | Value |
|---|---|
| Reference configuration | q = (−91.06°, −111.79°, −104.53°, −55.59°, 90.79°, −1.16°) |
| Gripper length | 0.12 m |
| Gripper mass | 0.8 kg |
| PD gains (tracking) | Kp = 80, Kd = 18 (per axis) |
| Environment stiffness | Kₑ = 200 N/m |
| Environment damping | Bₑ = 20 N·s/m |
| Simulation time step | dt = 0.002 s |

---

### License Information

ROB5-S10-SYS827 © 2026 by Dounia Bakalem, Vasileios Filippos Skarleas, and Yanis Sadoun is licensed under the [Creative Commons Attribution-NonCommercial 4.0 International](https://creativecommons.org/licenses/by-nc/4.0/).

This work also includes content that is not the property of the authors and is subject to copyright and other licenses from their respective owners.
