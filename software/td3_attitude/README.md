```markdown

# TD3 Spacecraft Attitude Control & Regulation

High-performance C++17 implementation of ****Twin Delayed Deep Deterministic Policy Gradient (TD3)**** reinforcement learning for autonomous spacecraft attitude regulation and large-angle slew maneuvers, powered by ****LibTorch**** (PyTorch C++ API).

Based on the research paper:

> ****AAS 20-475****: **Adaptive Continuous Control of Spacecraft Attitude Using Deep Reinforcement Learning**  

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

The environment simulates the ****Lockheed Martin LM50**** satellite bus dynamics using rigid-body Euler equations and quaternion kinematics, targeting sub-degree industry-standard pointing accuracy ($0.25^\circ$).

---

## Key Features

- ****Pure C++17 / LibTorch Engine****: Eliminates Python GIL and runtime overhead for microsecond step-time simulation and policy inference.

- ****Full Rigid-Body 6-DOF Rotational Dynamics****: Symplectic / 4th-order Runge-Kutta numerical integration with principal inertia tensor $I = \mathrm{diag}(0.872, 0.115, 0.797)\ \mathrm{kg}\cdot\mathrm{m}^2$.

- ****Gimbal-Lock Free****: Quaternion attitude representation $\mathbf{q} = [q_x, q_y, q_z, q_w]^T$ with shortest-path error calculation.

- ****Policy Equilibrium Centering****: Enforces $\pi(\mathbf{s}^*) = \mathbf{0}$ at zero-error rest state $\mathbf{s}^*$ by subtracting underlying actor network bias $\mathbf{a}^* = \pi_{\text{raw}}(\mathbf{s}^*)$.

- ****Dynamic Replay Buffer Recomputation****: Transitions stored in experience replay have their rewards dynamically re-evaluated at sampling time based on the active curriculum goal angle.

- ****Two-Stage Curriculum Learning****:

  - *Phase 1 ($0 - 60\%$, Episodes 0–6000)*: Coarse acquisition tolerance ($0.50^\circ$) to encourage exploration and robust slew convergence.

  - *Phase 2 ($60 - 100\%$, Episodes 6000–10000)*: Fine precision tolerance ($0.25^\circ$) with annealed learning rate and noise for pinpoint stability.

- ****Industry-Standard Pointing Accuracy****:

  - $>99.6\%$ pass rate at $0.25^\circ$ terminal error on unseen held-out validation seeds.

  - Mean terminal attitude error $<0.05^\circ$ with terminal angular rates $\lVert{}\boldsymbol{\omega}\lVert{} \sim 10^{-4}\ \mathrm{rad/s}$.

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

```

---

## Mathematical & RL Formulation

### Rotational Dynamics

The attitude motion of the rigid spacecraft body is governed by Euler's rotational equations:

$$\mathbf{M} = \mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})$$

where:

* $\mathbf{I} = \mathrm{diag}(0.872, 0.115, 0.797)\ \mathrm{kg}\cdot\mathrm{m}^2$ is the principal inertia tensor.

* $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ is the body angular velocity.

* $\mathbf{M} = \mathbf{u} + \mathbf{M}_{\text{dist}}$ is the total applied torque.

Attitude kinematics are propagated via quaternion integration:

$$\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\Omega}(\boldsymbol{\omega})\mathbf{q}, \quad \boldsymbol{\Omega}(\boldsymbol{\omega}) = \begin{bmatrix} 0 & \omega_z & -\omega_y & \omega_x \\ -\omega_z & 0 & \omega_x & \omega_y \\ \omega_y & -\omega_x & 0 & \omega_z \\ -\omega_x & -\omega_y & -\omega_z & 0 \end{bmatrix}$$

### State & Action Spaces

* ****State Space**** ($\mathbb{R}^7$):

$$\mathbf{s}*_t = \big[q_*{e,x},\ q*_{e,y},\ q_*{e,z},\ q_{e,w},\ \omega_x,\ \omega_y,\ \omega_z\big]^T$$





where $\mathbf{q}_e$ is the unit error quaternion between current orientation and target attitude $\mathbf{q}*_d$. The principal eigen-axis rotation error angle is $\phi = 2\arccos(\vert{}q_*{e,w}\vert{})$.

* ****Action Space**** ($\mathbb{R}^3$):

$$\mathbf{a}_t = [u_x, u_y, u_z]^T \in [-0.2, 0.2]^3\ \mathrm{Nm}$$





### TD3 Algorithm & Torque Bias Centering

TD3 addresses value overestimation in continuous action spaces through:

1. ****Clipped Double-Q Learning****: Maintains two critic networks $Q_{\psi_1}, Q_{\psi_2}$, computing target values using $\min(Q_{\psi_1'}, Q_{\psi_2'})$.

2. ****Delayed Policy Updates****: Actor parameters $\theta$ and target parameters are updated every $d = 2$ critic steps.

3. ****Target Action Smoothing****: Clipped Gaussian noise is injected into target actions during critic updates:

$$\tilde{\mathbf{a}} = \mathrm{clip}\left(\pi_{\theta'}(\mathbf{s}') + \epsilon,\ -a_{\text{max}},\ a_{\text{max}}\right), \quad \epsilon \sim \mathrm{clip}\left(\mathcal{N}(0, \sigma^2), -c, c\right)$$



4. ****Zero-Error Torque Centering****: At rest equilibrium $\mathbf{s}^* = [0, 0, 0, 1, 0, 0, 0]^T$, physical actuators must exert zero torque. The effective policy subtracts the raw network's resting offset:

$$\pi(\mathbf{s}) = \pi*_{\text{raw}}(\mathbf{s}) - \pi_*{\text{raw}}(\mathbf{s}^*)$$





### Reward Formulation & Curriculum

The per-step reward incentivizes rapid attitude convergence and rate damping:

$$r_t = \exp\left(-\frac{\phi_t^2}{2\sigma_\phi^2}\right) - \beta \lVert{}\boldsymbol{\omega}*_t\lVert{}^2 + R_*{\text{bonus}}(\phi*_t \le \phi_*{\text{goal}})$$

Transitions are stored with state components rather than static scalar rewards; upon sampling minibatches from the replay buffer, rewards are computed with the active goal tolerance $\phi_{\text{goal}}(e)$ corresponding to the current curriculum episode $e$.

---

## Prerequisites & Build

### Requirements

* ****CMake**** $\ge 3.18$

* ****C++17**** compliant compiler (GCC $\ge 9$ or Clang $\ge 10$)

* ****LibTorch (C++ PyTorch API)****: Available locally under `third_party/libtorch` or installed via system path.

* ****Python 3.8+**** (for execution scripts)

### Compiling

From the project root:

```bash

# Configure the build with CMake

cmake -B build -S .

# Compile train and eval executables

cmake --build build -j$(nproc)

```

The resulting binaries will be placed in the `bin/` directory:

* `bin/train`

* `bin/eval`

---

## Usage

### Running Evaluation

Run deterministic greedy evaluation across held-out random initial attitudes:

```bash

# Evaluate best model on 300 test episodes at 0.25 deg target

./bin/eval models/best/actor.pt models/best/critic.pt 0.25 300 21 10000

# Quick evaluation run with default arguments

./bin/eval

```

****Evaluation CLI Arguments:****

```text

./bin/eval [actor.pt] [critic.pt] [goal_deg] [episodes] [frameskip] [seed_offset] [--center]

```

* `actor.pt`: Path to actor weights (defaults to `models/best/actor.pt`).

* `critic.pt`: Path to critic weights (defaults to `models/best/critic.pt`).

* `goal_deg`: Target pointing accuracy tolerance in degrees (e.g. `0.25` or `0.5`).

* `episodes`: Number of evaluation episodes (default: `300`).

* `frameskip`: Environment action repeat steps (default: `21`, corresponding to $11.43\ \mathrm{Hz}$).

* `seed_offset`: Random seed offset to ensure evaluation on unseen initial conditions (default: `10000`).

* `--center`: Diagnostic flag to inspect torque bias centering.

### Running Training

To train a new policy from scratch with curriculum scheduling:

```bash

./bin/train \

  --episodes 10000 \

  --warmup 500 \

  --lr-start 3e-4 \

  --lr-end 1e-6 \

  --noise-start 0.05 \

  --noise-end 0.005 \

  --goal-switch-frac 0.6 \

  --goal-start-deg 0.5 \

  --goal-final-deg 0.25 \

  --save-prefix stageB \

  --checkpoint-interval 250

```

Periodic checkpoints and final model weights will be automatically saved under `models/checkpoints/\<save-prefix>/`.

### Automated Stage B Pipeline

The orchestrator script handles end-to-end training, checkpoint evaluation across 300 seeds, performance scoring, and automatic selection of the optimal policy weights:

```bash

python3 scripts/run_stageB.py

```

Upon completion, qualifying weights with $\ge 95\%$ pass rate and $0$ failures are saved to `models/best/actor.pt` and `models/best/critic.pt`.

---

## Models & Checkpoints

* ****`models/best/`****: Optimal production policy weights delivering:

* Terminal Pass Rate ($<0.25^\circ$): **100.0%** (evaluation benchmark)

* Mean Terminal Angle Error: **$0.042^\circ$**

* Failures: **0 / 300**



* **`models/backup/`**: Verified baseline fallback weights.

* **`models/checkpoints/`**: Comprehensive checkpoint history partitioned by experiment:

* `biased/`: Exploration checkpoints prior to torque equilibrium centering.

* `fs20/`: Checkpoints evaluated at frameskip 20.

* `stageA/` & `stageA2/`: Phase 1 curriculum checkpoints ($0.50^\circ$ tolerance).

* `stageB/`: Stage B two-phase curriculum checkpoints across all 10,000 training episodes.





---

## References

1. Elkins, J. G., Sood, R., & Rumpf, C. (2020). *Adaptive Continuous Control of Spacecraft Attitude Using Deep Reinforcement Learning*. AAS/AIAA Astrodynamics Specialist Conference, AAS 20-475.

2. Fujimoto, S., van Hoof, H., & Meger, D. (2018). *Addressing Function Approximation Error in Actor-Critic Methods*. International Conference on Machine Learning (ICML).

3. Markley, F. L., & Crassidis, J. L. (2014). *Fundamentals of Spacecraft Attitude Determination and Control*. Springer.

```

```
