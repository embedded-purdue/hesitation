# TD3 Spacecraft Attitude Control & Regulation

High-performance C++17 implementation of **Twin Delayed Deep Deterministic Policy Gradient (TD3)** reinforcement learning for autonomous spacecraft attitude regulation and large-angle slew maneuvers, powered by **LibTorch** (PyTorch C++ API).

Based on the research paper:
> **AAS 20-475**: *Adaptive Continuous Control of Spacecraft Attitude Using Deep Reinforcement Learning*  
> Jacob G. Elkins (Univ. of Alabama), Rohan Sood (Univ. of Alabama), Clemens Rumpf (NASA Ames / STC).

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Repository Structure](#repository-structure)
- [Mathematical & RL Formulation](#mathematical--rl-formulation)
  - [Rotational Dynamics](#rotational-dynamics)
  - [State & Action Spaces](#state--action-spaces)
  - [TD3 Algorithm & Torque Bias Centering](#td3-algorithm--torque-bias-centering)
  - [Reward Formulation & Curriculum](#reward-formulation--curriculum)
- [Prerequisites & Build](#prerequisites--build)
- [Usage](#usage)
  - [Running Evaluation](#running-evaluation)
  - [Running Training](#running-training)
  - [Automated Stage B Pipeline](#automated-stage-b-pipeline)
- [Models & Checkpoints](#models--checkpoints)
- [References](#references)

---

## Overview

Autonomous attitude control is essential for deep-space missions where transmission delays prohibit ground-in-the-loop intervention. This project provides a low-latency, real-time-capable reinforcement learning framework for large-angle spacecraft slew maneuvers. The agent maps attitude quaternion error and angular velocities directly to continuous 3-axis reaction wheel/thruster torques.

The environment simulates the **Lockheed Martin LM50** satellite bus dynamics using rigid-body Euler equations and quaternion kinematics, targeting sub-degree industry-standard pointing accuracy ($0.25^\circ$).

---

## Key Features

- **Pure C++17 / LibTorch Engine**: Eliminates Python GIL and runtime overhead for microsecond step-time simulation and policy inference.
- **Full Rigid-Body 6-DOF Rotational Dynamics**: Symplectic / 4th-order Runge-Kutta numerical integration with principal inertia tensor $I = \mathrm{diag}(0.872, 0.115, 0.797)\ \mathrm{kg}\cdot\mathrm{m}^2$.
- **Gimbal-Lock Free**: Quaternion attitude representation $\mathbf{q} = [q_x, q_y, q_z, q_w]^T$ with shortest-path error calculation.
- **Policy Equilibrium Centering**: Enforces $\pi(\mathbf{s}^*) = \mathbf{0}$ at zero-error rest state $\mathbf{s}^*$ by subtracting underlying actor network bias $\mathbf{a}^* = \pi_{\text{raw}}(\mathbf{s}^*)$.
- **Dynamic Replay Buffer Recomputation**: Transitions stored in experience replay have their rewards dynamically re-evaluated at sampling time based on the active curriculum goal angle.
- **Two-Stage Curriculum Learning**:
  - *Phase 1 ($0 - 60\%$, Episodes 0–6000)*: Coarse acquisition tolerance ($0.50^\circ$) to encourage exploration and robust slew convergence.
  - *Phase 2 ($60 - 100\%$, Episodes 6000–10000)*: Fine precision tolerance ($0.25^\circ$) with annealed learning rate and noise for pinpoint stability.
- **Industry-Standard Pointing Accuracy**:
  - $>99.6\%$ pass rate at $0.25^\circ$ terminal error on unseen held-out validation seeds.
  - Mean terminal attitude error $<0.05^\circ$ with terminal angular rates $\Vert{}\boldsymbol{\omega}\Vert{} \sim 10^{-4}\ \mathrm{rad/s}$.
  - Exactly $0$ large-slew divergence failures.

---

## Repository Structure

```text
td3_attitude/
├── CMakeLists.txt              # CMake build configuration linking LibTorch
├── .gitignore                  # Git ignore rules (bin/, build/, third_party/)
├── README.md                   # Project documentation and guide
├── docs/
│   └── td3SpaceCraftPaper.txt  # Reference paper text (AAS 20-475)
├── scripts/
│   └── run_stageB.py           # Automated Stage B training, evaluation & model selection
├── include/
│   ├── device.hpp              # Torch device selection (CUDA / CPU auto-detection)
│   ├── dynamics.hpp            # Spacecraft physical parameters & inertia tensor
│   ├── env.hpp                 # Gym-style attitude simulation environment & step mechanics
│   ├── network.hpp             # Actor and Twin Critic neural network architectures
│   ├── quat.hpp                # Quaternion algebra (multiplication, conjugate, rotation)
│   ├── replay_buffer.hpp       # Experience replay buffer with dynamic reward recomputation
│   ├── td3_agent.hpp           # TD3 agent implementation, target networks, and soft updates
│   └── vec3.hpp                # 3D vector operations (cross product, norm, dot)
├── src/
│   ├── eval.cpp                # Deterministic evaluation and diagnostic executable
│   └── train.cpp               # Continuous curriculum TD3 training executable
├── models/
│   ├── best/                   # Top qualified policy weights meeting mission criteria
│   │   ├── actor.pt
│   │   └── critic.pt
│   ├── backup/                 # Baseline backup weights
│   │   ├── actor.pt
│   │   └── critic.pt
│   └── checkpoints/            # Periodic training checkpoints organized by experiment
│       ├── biased/             # Initial uncentered exploration checkpoints
│       ├── fs20/               # Frameskip 20 baseline checkpoints
│       ├── stageA/             # Curriculum Phase 1 checkpoints (0.50 deg goal)
│       ├── stageA2/            # Secondary Phase 1 checkpoints
│       └── stageB/             # Stage B full curriculum checkpoints (up to ep 10000)
├── bin/                        # Ignored by Git (compiled binary outputs)
└── build/                      # Ignored by Git (CMake build artifacts)
