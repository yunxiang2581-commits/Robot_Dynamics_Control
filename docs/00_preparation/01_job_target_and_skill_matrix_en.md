# Job Target And Skill Matrix

> Chinese version: [01_job_target_and_skill_matrix.md](01_job_target_and_skill_matrix.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [00 Preparation overview](00_preparation_overview_en.md) | Job target and skill matrix | [02 Three-track scope](02_three_project_scope_en.md) |

## Purpose

This document maps job requirements in robot motion control to concrete project capabilities and then assigns those capabilities to A, B, and C.

## Key Takeaways

| Target | Meaning |
| --- | --- |
| Job direction | robot motion control internship, robot control engineer, humanoid motion control |
| Major skills | kinematics, dynamics, simulation, optimal control, ROS engineering, RL, Sim2Real |
| Project strategy | A proves fundamentals, B supports model-based control, C supports RL-based control |

## Keyword Mapping

| Job keyword | Capability | Project support |
| --- | --- | --- |
| Robot control | closed loop, state, command, feedback, error | A PD, IK, Mini-WBC; B WBC |
| Dynamics analysis | joint space, task space, CoM, contact force, torque | A Pinocchio; B OCS2/WBC |
| MuJoCo / Isaac Sim | validating control in simulation | A MuJoCo PD; C MuJoCo RL |
| ROS / ROS2 | software stack and controller interfaces | B `ros-control`; ROS2 later |
| C++ / Python | prototyping and source-code reading | A Python baseline; B C++/ROS reading |
| PID / impedance control | low-level control basics | A PD, later impedance-control extension |
| MPC / WBC / QP | optimization-based control modeling | A QP-IK, Mini-WBC; B NMPC/WBC |
| PPO / SAC / TD3 | RL training and tuning | C `unitree_rl_mjlab`; SAC/TD3 later as comparison |
| Sim2Real | deployment safety and transfer issues | C Sim2Real notes; B real-robot control-chain understanding |

## Capability Matrix

| Capability | A self baseline | B `legged_control` | C `unitree_rl_mjlab` | Interview value |
| --- | --- | --- | --- | --- |
| Model loading | Pinocchio URDF loading | robot model and `ros-control` config reading | MJCF and Unitree model reading | can work with real robot descriptions |
| Kinematics | FK, Jacobian, DLS-IK, QP-IK | task-space control understanding | policy input state understanding | can map task goals to joint-space motion |
| Dynamics | teaching Mini-WBC | NMPC, WBC, contact force, torque | MuJoCo physics | can explain core model-based control variables |
| Optimization | OSQP or cvxpy QP skeleton | OCS2 and QP-WBC | PPO policy optimization | can compare numerical optimization and policy optimization |
| Simulation | MuJoCo PD tracking | Gazebo/ROS simulation notes | MuJoCo Play/Train | can validate control ideas in simulation |
| Deployment awareness | logging, plots, config files | control rate, estimation, interfaces | Sim2Real and safety | can discuss engineering deployment risk |
