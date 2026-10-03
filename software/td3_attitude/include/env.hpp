#pragma once
#include "dynamics.hpp"
#include "quat.hpp"
#include "vec3.hpp"
#include <cmath>
#include <functional>
#include <optional>
#include <random>

constexpr double DT = 1.0 / 240.0;

// Paper: 1 torque timestep + 20 free-rotation timesteps = 21 timesteps total
// (240/21 = 11.42857 Hz ~ 11.43 Hz)
constexpr int TRAIN_FRAMESKIP = 20;
// Paper disturbance test: 1 torque + 5 free = 6 timesteps total (240/6 = 40 Hz)
constexpr int EVAL_FRAMESKIP = 6;

constexpr int MAX_STEPS = 500;
constexpr double OMEGA_LIMIT = 0.5;
constexpr double TORQUE_LIMIT = 0.5;

// Goal constants: q_s = cos(phi / 2)
// phi <= 1.00 deg: cos(0.500 deg) = 0.9999619230641713  (paper nominal text
// constant) phi <= 0.50 deg: cos(0.250 deg) = 0.9999904807207345  (best
// stabilizing run) phi <= 0.25 deg: cos(0.125 deg) = 0.9999976201773518  (LM50
// pointing spec)
constexpr double GOAL_QS_1_0_DEG = 0.9999619230641713;
constexpr double GOAL_QS_0_5_DEG = 0.9999904807207345;
constexpr double GOAL_QS_0_25_DEG = 0.9999976201773518;

// Default goal threshold (LM50 accuracy spec of 0.25 deg)
constexpr double GOAL_QS = GOAL_QS_0_25_DEG;

inline double phi_deg_to_qs(double phi_deg) {
  return std::cos((phi_deg * M_PI / 180.0) / 2.0);
}

struct StepResult {
  State state;
  double reward;
  bool terminated; // true if ||w|| > OMEGA_LIMIT (absorbing failure state)
  bool truncated;  // true if t >= MAX_STEPS (time horizon limit)
  double phi;
  bool just_reached_goal;

  bool done() const { return terminated || truncated; }
};

class SpacecraftAttitudeEnv {
public:
  // Primary constructor with explicit goal_qs
  SpacecraftAttitudeEnv(
      unsigned seed, double goal_qs, int frameskip = TRAIN_FRAMESKIP,
      Vec3 inertia = Vec3(0.872, 0.115, 0.797),
      std::optional<std::function<Vec3(double)>> disturbance_fn = std::nullopt)
      : rng_(seed), goal_qs_(goal_qs), frameskip_(frameskip), I_(inertia),
        I_inv_(elementwise_inv(inertia)),
        disturbance_fn_(std::move(disturbance_fn)) {
    reset();
  }

  // Convenience constructor defaulting to GOAL_QS with optional frameskip
  SpacecraftAttitudeEnv(
      unsigned seed, int frameskip = TRAIN_FRAMESKIP,
      Vec3 inertia = Vec3(0.872, 0.115, 0.797),
      std::optional<std::function<Vec3(double)>> disturbance_fn = std::nullopt)
      : SpacecraftAttitudeEnv(seed, GOAL_QS, frameskip, inertia,
                              std::move(disturbance_fn)) {}

  void set_goal_qs(double goal_qs) { goal_qs_ = goal_qs; }
  double goal_qs() const { return goal_qs_; }

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
    prev_qs_ = state_.q.w;
    past_goal_ = state_.q.w >= goal_qs_;
    return state_;
  }

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

    double qs = state_.q.w;

    bool just_reached_goal = (!past_goal_) && qs >= goal_qs_;

    if (qs >= goal_qs_)
      past_goal_ = true;

    double reward;

    if (past_goal_) {
      // Eq. (15)
      reward = -state_.q.x * state_.q.x - state_.q.y * state_.q.y -
               state_.q.z * state_.q.z + state_.q.w * state_.q.w;
    } else {
      // Eq. (14)
      reward = (qs > prev_qs_) ? 0.1 : -0.1;
    }

    prev_qs_ = qs;

    bool terminated = false;
    if (state_.w.norm() > OMEGA_LIMIT) {
      reward = -25.0;
      terminated = true;
    }
    bool truncated = t_ >= MAX_STEPS;
    if (truncated) {
      reward += (qs >= goal_qs_) ? 10.0 : 0.0;
    }

    double phi = state_.q.angle();
    return StepResult{state_,    reward, terminated,
                      truncated, phi,    just_reached_goal};
  }

private:
  static Vec3 clamp_vec(const Vec3 &v, double limit) {
    auto c = [limit](double x) { return std::max(-limit, std::min(limit, x)); };
    return Vec3(c(v.x), c(v.y), c(v.z));
  }

  Vec3 random_unit_vector() {
    std::uniform_real_distribution<double> u_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> theta_dist(0.0, 2.0 * M_PI);
    double u = u_dist(rng_);
    double theta = theta_dist(rng_);
    double r = std::sqrt(std::max(0.0, 1.0 - u * u));
    return Vec3(r * std::cos(theta), r * std::sin(theta), u);
  }

  std::mt19937 rng_;
  double goal_qs_;
  int frameskip_;
  Vec3 I_, I_inv_;
  std::optional<std::function<Vec3(double)>> disturbance_fn_;

  State state_;
  int t_ = 0;
  double prev_qs_ = 0.0;
  bool past_goal_ = false;
};
