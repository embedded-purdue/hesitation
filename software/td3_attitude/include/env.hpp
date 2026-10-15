/**
 * @file env.hpp
 * @brief Spacecraft attitude regulation simulation environment and Markov Decision Process (MDP).
 *
 * Simulates 3-DOF rigid-body spacecraft rotational dynamics governed by Euler's equations
 * and quaternion kinematics. Formulates the reinforcement learning environment for attitude control:
 *
 * System Parameters:
 *   - Integration timestep: DT = 1/240 s (~4.167 ms)
 *   - Control frequency:
 *       * Training frameskip = 21 timesteps (240/21 ≈ 11.43 Hz control cycle)
 *       * Disturbance evaluation frameskip = 6 timesteps (240/6 = 40 Hz control cycle)
 *   - Actuator torque saturation: [-0.5, +0.5] Nm per principal axis
 *   - Maximum safe angular velocity: OMEGA_LIMIT = 0.5 rad/s (absorbing failure threshold)
 *   - Maximum episode duration: MAX_STEPS = 500 control steps (~43.75 s)
 *
 * Reward Function (Stateless Dense Formulation):
 *   r_t = exp(-phi_t / sigma) + B * 1[phi_t <= phi_goal] - beta * max(0, ||w_t|| - 0.3)
 *   where:
 *     sigma = 0.14 * pi rad (~25.2 deg) : Smooth exponential approach gradient across all angles
 *     B     = 1.0                       : Attitude goal-cone achievement bonus
 *     beta  = 5.0                       : Soft penalty for excessive angular velocity (> 0.3 rad/s)
 *     Hard termination penalty = -25.0  : Absorbing failure when ||w|| > 0.5 rad/s
 *
 * Double-Cover Attitude Canonicalization:
 *   After every simulation step and reset, if q_w < 0, the entire quaternion is negated:
 *   q <- -q. Since q and -q represent identical 3D physical attitudes, this ensures q_w >= 0
 *   unambiguously across all observations.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * Reference:
 *   Elkins et al., "Spacecraft Attitude Control Using Deep Reinforcement Learning,"
 *   AAS 20-475.
 */

#pragma once

#include "dynamics.hpp"
#include "quat.hpp"
#include "vec3.hpp"
#include <cmath>
#include <functional>
#include <optional>
#include <random>
#include <torch/torch.h>

/// Base simulation integration timestep [s] (240 Hz).
constexpr double DT = 1.0 / 240.0;

/// Training frameskip: 1 torque sub-step + 20 free-rotation sub-steps = 21 timesteps (11.43 Hz).
constexpr int TRAIN_FRAMESKIP = 21;

/// Disturbance evaluation frameskip: 1 torque + 5 free = 6 timesteps (40 Hz).
constexpr int EVAL_FRAMESKIP  = 6;

/// Maximum episode horizon in control steps (500 steps * 21 * DT ≈ 43.75 s).
constexpr int    MAX_STEPS    = 500;

/// Angular velocity failure limit [rad/s]. Exceeding this triggers early termination.
constexpr double OMEGA_LIMIT  = 0.5;

/// Actuator torque saturation limit per principal body axis [Nm].
constexpr double TORQUE_LIMIT = 0.5;

// ── Reward Shaping Constants ──────────────────────────────────────────────

/// Scale parameter sigma for exponential approach reward exp(-phi / sigma) [rad].
constexpr double REWARD_SIGMA = 0.14 * M_PI;   // radians (~25.2 deg)

/// Goal-cone achievement bonus awarded when attitude error phi <= phi_goal.
constexpr double REWARD_B     = 1.0;

/// Angular-rate soft penalty scaling coefficient beta.
constexpr double REWARD_BETA  = 5.0;

/// Angular-rate penalty threshold [rad/s] above which rates are penalized.
constexpr double OMEGA_SOFT   = 0.3;

/// Absorbing state terminal penalty incurred when ||w|| > OMEGA_LIMIT.
constexpr double OMEGA_TERM_PENALTY = -25.0;

// ── Goal Tolerance Constants ──────────────────────────────────────────────

/**
 * @brief Converts attitude error tolerance in degrees to equivalent quaternion scalar threshold q_s.
 *
 * q_s = cos(phi / 2)
 *
 * @param phi_deg Attitude tolerance angle in degrees.
 * @return double Corresponding quaternion scalar component threshold.
 */
inline double phi_deg_to_qs(double phi_deg) {
  return std::cos((phi_deg * M_PI / 180.0) / 2.0);
}

/// Goal scalar threshold for 0.50 deg attitude tolerance: cos(0.25 deg).
constexpr double GOAL_QS_0_5_DEG  = 0.9999904807207345;

/// Goal scalar threshold for 0.25 deg attitude tolerance: cos(0.125 deg) (LM50 pointing spec).
constexpr double GOAL_QS_0_25_DEG = 0.9999976201773518;

/// Default pointing goal tolerance threshold.
constexpr double GOAL_QS = GOAL_QS_0_25_DEG;

// ── Shared State Observation Tensor Builder ───────────────────────────────

/**
 * @brief Constructs the standardized 11-dimensional observation tensor from physical State.
 *
 * Observation Vector Components:
 *   [0..3]: Attitude unit quaternion q = [qx, qy, qz, qw] (with qw >= 0).
 *   [4..7]: Scaled quaternion rate qdot / OMEGA_LIMIT.
 *   [8..10]: Scaled body angular velocity w / OMEGA_LIMIT.
 *
 * Normalizing rates by OMEGA_LIMIT ensures near-goal values are O(1) for neural network inputs.
 * This function is shared identically across training and evaluation pipelines.
 *
 * @param s Physical spacecraft state {w, q}.
 * @return torch::Tensor 1D CPU float32 tensor of shape [11].
 */
inline torch::Tensor make_state_tensor(const State &s) {
  Quat qdot = omega_mul(s.w, s.q) * 0.5;
  const double sc = 1.0 / OMEGA_LIMIT;
  return torch::tensor(
      {s.q.x, s.q.y, s.q.z, s.q.w,
       qdot.x * sc, qdot.y * sc, qdot.z * sc, qdot.w * sc,
       s.w.x * sc,  s.w.y * sc,  s.w.z * sc},
      torch::TensorOptions().dtype(torch::kFloat32));
}

// ── Step Result Structure ─────────────────────────────────────────────────

/**
 * @brief Return container for Environment::step execution.
 */
struct StepResult {
  State  state;       ///< Next physical spacecraft state after frameskip integration.
  double reward;      ///< Scalar reward evaluated at the transition.
  bool   terminated;  ///< True iff ||w|| > OMEGA_LIMIT (absorbing failure state).
  bool   truncated;   ///< True iff t >= MAX_STEPS (time-horizon expiration, non-absorbing).
  double phi;         ///< Attitude error angle relative to target [rad].

  /// Returns true if the episode has finished either by termination or truncation.
  bool done() const { return terminated || truncated; }
};

// ── Spacecraft Attitude MDP Environment ───────────────────────────────────

/**
 * @brief Continuous attitude control environment for rigid-body spacecraft.
 */
class SpacecraftAttitudeEnv {
public:
  /**
   * @brief Primary constructor specifying custom goal threshold, frameskip, inertia, and disturbances.
   *
   * @param seed           Pseudo-random number generator seed.
   * @param goal_qs        Target quaternion scalar threshold cos(phi_goal / 2).
   * @param frameskip      Integration sub-steps per environment step (default: 21).
   * @param inertia        Principal moments of inertia vector [kg*m^2] (default: LM50 satellite).
   * @param disturbance_fn Optional time-dependent external disturbance torque function M_ext(t) [Nm].
   */
  SpacecraftAttitudeEnv(
      unsigned seed, double goal_qs, int frameskip = TRAIN_FRAMESKIP,
      Vec3 inertia = Vec3(0.872, 0.115, 0.797),
      std::optional<std::function<Vec3(double)>> disturbance_fn = std::nullopt)
      : rng_(seed), goal_qs_(goal_qs), frameskip_(frameskip), I_(inertia),
        I_inv_(elementwise_inv(inertia)),
        disturbance_fn_(std::move(disturbance_fn)) {
    reset();
  }

  /**
   * @brief Convenience constructor defaulting to GOAL_QS (0.25 deg specification).
   *
   * @param seed           Pseudo-random generator seed.
   * @param frameskip      Integration sub-steps per step (default: 21).
   * @param inertia        Principal moments of inertia vector [kg*m^2].
   * @param disturbance_fn Optional disturbance torque function.
   */
  SpacecraftAttitudeEnv(
      unsigned seed, int frameskip = TRAIN_FRAMESKIP,
      Vec3 inertia = Vec3(0.872, 0.115, 0.797),
      std::optional<std::function<Vec3(double)>> disturbance_fn = std::nullopt)
      : SpacecraftAttitudeEnv(seed, GOAL_QS, frameskip, inertia,
                              std::move(disturbance_fn)) {}

  /// Sets the active goal tolerance threshold cos(phi_goal / 2).
  void   set_goal_qs(double goal_qs) { goal_qs_ = goal_qs; }

  /// Retrieves the active goal tolerance threshold.
  double goal_qs() const             { return goal_qs_; }

  /**
   * @brief Resets the spacecraft state to an initial attitude.
   *
   * If initial attitude q0 is not provided, samples a random unit axis and
   * an initial angle uniformly distributed between [30 deg, 150 deg],
   * matching the AAS 20-475 benchmark specification.
   * Enforces quaternion canonicalization (qw >= 0).
   *
   * @param q0 Optional initial attitude quaternion.
   * @param w0 Optional initial body angular velocity vector (defaults to zero).
   * @return State Initial physical state of the spacecraft.
   */
  State reset(std::optional<Quat> q0 = std::nullopt,
              std::optional<Vec3> w0 = std::nullopt) {
    if (q0.has_value()) {
      state_.q = *q0;
    } else {
      Vec3 axis = random_unit_vector();
      std::uniform_real_distribution<double> phi_dist(30.0 * M_PI / 180.0,
                                                      150.0 * M_PI / 180.0);
      double phi = phi_dist(rng_);
      state_.q = Quat::from_axis_angle(axis, phi);
    }
    state_.w = w0.value_or(Vec3(0, 0, 0));
    t_ = 0;

    // Enforce q_w >= 0 canonical form from the start
    canonicalize_quat();
    return state_;
  }

  /**
   * @brief Advances simulation by one environment step (frameskip RK4 integration sub-steps).
   *
   * Control torque is applied on the first sub-step, followed by free rotation
   * for the remaining frameskip - 1 sub-steps (plus any external disturbances).
   *
   * @param action Commanded 3-axis torque vector [Nm], clamped to [-TORQUE_LIMIT, TORQUE_LIMIT].
   * @return StepResult State transition outcome, reward, and done flags.
   */
  StepResult step(const Vec3 &action) {
    Vec3 M = clamp_vec(action, TORQUE_LIMIT);

    for (int i = 0; i < frameskip_; ++i) {
      Vec3 torque = (i == 0) ? M : Vec3(0, 0, 0);
      if (disturbance_fn_.has_value()) {
        double t_sec = t_ * frameskip_ * DT + i * DT;
        torque = torque + (*disturbance_fn_)(t_sec);
      }
      state_ = rk4_step(state_, torque, DT, I_, I_inv_);
    }
    t_ += 1;

    // Keep q_w >= 0 (double-cover symmetry)
    canonicalize_quat();

    double phi = state_.q.angle();
    double qs  = state_.q.w;

    // ── Stateless Dense Reward Evaluation ───────────────────────────────
    // 1. Exponential approach gradient across all angles
    double r_approach = std::exp(-phi / REWARD_SIGMA);

    // 2. Goal-cone achievement bonus
    double r_goal = (qs >= goal_qs_) ? REWARD_B : 0.0;

    // 3. Angular-rate soft damping penalty
    double omega_norm = state_.w.norm();
    double r_omega    = -REWARD_BETA * std::max(0.0, omega_norm - OMEGA_SOFT);

    double reward = r_approach + r_goal + r_omega;

    // ── Hard Absorbing Failure Termination ──────────────────────────────
    bool terminated = false;
    if (omega_norm > OMEGA_LIMIT) {
      reward     = OMEGA_TERM_PENALTY;
      terminated = true;
    }

    // ── Time Horizon Truncation ─────────────────────────────────────────
    bool truncated = (t_ >= MAX_STEPS);

    return StepResult{state_, reward, terminated, truncated, phi};
  }

private:
  /**
   * @brief Canonicalizes quaternion representation such that q_w >= 0.
   */
  void canonicalize_quat() {
    if (state_.q.w < 0.0) {
      state_.q.x = -state_.q.x;
      state_.q.y = -state_.q.y;
      state_.q.z = -state_.q.z;
      state_.q.w = -state_.q.w;
    }
  }

  /**
   * @brief Component-wise vector clamping to [-limit, +limit].
   */
  static Vec3 clamp_vec(const Vec3 &v, double limit) {
    auto c = [limit](double x) { return std::max(-limit, std::min(limit, x)); };
    return Vec3(c(v.x), c(v.y), c(v.z));
  }

  /**
   * @brief Generates a random unit vector uniformly distributed on the unit 2-sphere.
   */
  Vec3 random_unit_vector() {
    std::uniform_real_distribution<double> u_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> theta_dist(0.0, 2.0 * M_PI);
    double u     = u_dist(rng_);
    double theta = theta_dist(rng_);
    double r     = std::sqrt(std::max(0.0, 1.0 - u * u));
    return Vec3(r * std::cos(theta), r * std::sin(theta), u);
  }

  std::mt19937 rng_;       ///< Mersenne Twister pseudo-random number generator.
  double       goal_qs_;   ///< Active goal scalar threshold cos(phi_goal / 2).
  int          frameskip_; ///< Number of RK4 simulation sub-steps per step.
  Vec3         I_, I_inv_; ///< Spacecraft principal moments of inertia and elementwise inverse.
  std::optional<std::function<Vec3(double)>> disturbance_fn_; ///< Optional external disturbance torque.

  State state_;    ///< Current physical spacecraft state.
  int   t_ = 0;    ///< Current step index within the episode.
};
