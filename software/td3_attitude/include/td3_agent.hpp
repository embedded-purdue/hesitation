/**
 * @file td3_agent.hpp
 * @brief Twin Delayed Deep Deterministic Policy Gradient (TD3) reinforcement learning agent.
 *
 * Implements the TD3 continuous control algorithm (Fujimoto et al., 2018) adapted
 * for spacecraft attitude regulation (Elkins et al., 2020):
 *
 * Algorithmic Features:
 *   1. Clipped Double-Q Learning: Min of twin target critics (q1_t, q2_t) prevents overestimation.
 *   2. Target Policy Smoothing: Zero-mean clipped Gaussian noise added to target actions.
 *   3. Delayed Policy Updates: Actor and target networks update every POLICY_DELAY (= 2) critic iterations.
 *   4. Polyak Averaging: Soft target network parameter updates with parameter TAU = 0.005.
 *   5. Curriculum Support: train_step receives current_goal_qs for sample-time reward recomputation.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * Reference:
 *   Fujimoto et al., "Addressing Function Approximation Error in Actor-Critic Methods," ICML 2018.
 *   Elkins et al., "Spacecraft Attitude Control Using Deep Reinforcement Learning," AAS 20-475.
 */

#pragma once

#include <torch/torch.h>

#include "device.hpp"
#include "network.hpp"
#include "replay_buffer.hpp"

/// Discount factor gamma for future returns.
constexpr double GAMMA = 0.99;

/// Mini-batch size for stochastic gradient descent steps.
constexpr int64_t BATCH_SIZE = 100;

/// Standard deviation of target policy smoothing noise [Nm].
constexpr double POLICY_NOISE = 0.1 * ACTION_SCALE;

/// Saturation clip limit for target policy smoothing noise [Nm].
constexpr double NOISE_CLIP = 0.5 * ACTION_SCALE;

/// Nominal initial exploration noise standard deviation [Nm].
constexpr double EXPLORATION_NOISE = 0.1 * ACTION_SCALE;

/// Frequency of delayed actor and target updates relative to critic updates.
constexpr int POLICY_DELAY = 2;

/// Polyak target network exponential tracking rate (tau = 1 - 0.995 = 0.005).
constexpr double TAU = 1.0 - 0.995;

/**
 * @brief TD3 agent encapsulating Actor, Twin Critics, target networks, and Adam optimizers.
 */
class TD3Agent {
public:
  /**
   * @brief Initializes actor, critic, target networks, and Adam optimizers on target device.
   */
  TD3Agent() : device_(get_device()) {
    actor = Actor();
    critic = Critic();
    actor_target = Actor();
    critic_target = Critic();

    // Move modules to target compute device before creating optimizers or copying parameters.
    actor->to(device_);
    critic->to(device_);
    actor_target->to(device_);
    critic_target->to(device_);

    copy_parameters(*actor, *actor_target);
    copy_parameters(*critic, *critic_target);

    actor_opt = std::make_shared<torch::optim::Adam>(
        actor->parameters(), torch::optim::AdamOptions(3e-4));

    critic_opt = std::make_shared<torch::optim::Adam>(
        critic->parameters(), torch::optim::AdamOptions(3e-4));
  }

  /**
   * @brief Computes a continuous torque action for a given state observation.
   *
   * @param state 1D CPU float32 tensor of shape [STATE_DIM] (11 dimensions).
   * @param noise Standard deviation of exploration Gaussian noise added to action (default: 0.0).
   * @return torch::Tensor 1D CPU float32 tensor of shape [ACTION_DIM] clamped to [-ACTION_SCALE, ACTION_SCALE].
   */
  torch::Tensor select_action(torch::Tensor state, double noise = 0.0) {
    torch::NoGradGuard no_grad;

    auto action = actor->forward(state.to(device_).unsqueeze(0)).squeeze(0);

    if (noise > 0.0) {
      action = action + torch::randn_like(action) * noise;
    }

    return torch::clamp(action, -ACTION_SCALE, ACTION_SCALE).to(torch::kCPU);
  }

  /**
   * @brief Sets learning rate for both Actor and Critic Adam optimizers.
   *
   * @param lr Current learning rate from the annealing schedule.
   */
  void set_lr(double lr) {
    for (auto &group : actor_opt->param_groups()) {
      static_cast<torch::optim::AdamOptions &>(group.options()).lr(lr);
    }

    for (auto &group : critic_opt->param_groups()) {
      static_cast<torch::optim::AdamOptions &>(group.options()).lr(lr);
    }
  }

  /**
   * @brief Queries the underlying uncentered network bias a* = raw_forward(s*).
   *
   * @return Vec3 3D torque vector representing raw uncentered network output at equilibrium.
   */
  Vec3 raw_a_star() {
    torch::NoGradGuard no_grad;
    auto a = actor->a_star(device_).to(torch::kCPU);
    return Vec3(a[0].item<double>(), a[1].item<double>(), a[2].item<double>());
  }

  /**
   * @brief Executes one training step of TD3 using a batch sampled from the replay buffer.
   *
   * Performs:
   *   1. Target action smoothing and evaluation of target Q = min(Q1_target, Q2_target).
   *   2. Bellman target calculation: y = r + gamma * (1 - terminated) * target_Q.
   *   3. Twin critic MSE loss calculation and gradient descent step.
   *   4. Delayed policy gradient update on Actor (every POLICY_DELAY steps).
   *   5. Polyak soft update of target Actor and target Critic networks.
   *
   * @param buffer          Experience replay buffer containing transitions.
   * @param current_goal_qs Current curriculum attitude tolerance cos(phi_goal / 2).
   */
  void train_step(ReplayBuffer &buffer, double current_goal_qs) {
    total_it_++;

    auto [s, a, r, s2, d] = buffer.sample(BATCH_SIZE, current_goal_qs);

    torch::Tensor y;

    {
      torch::NoGradGuard no_grad;

      // Target policy smoothing noise
      auto noise = torch::randn_like(a) * POLICY_NOISE;
      noise = torch::clamp(noise, -NOISE_CLIP, NOISE_CLIP);

      auto a2 = actor_target->forward(s2) + noise;
      a2 = torch::clamp(a2, -ACTION_SCALE, ACTION_SCALE);

      // Clipped double-Q target evaluation
      auto [q1_t, q2_t] = critic_target->forward(s2, a2);
      auto q_t = torch::min(q1_t, q2_t);

      y = r + GAMMA * (1.0 - d) * q_t;
    }

    auto [q1, q2] = critic->forward(s, a);

    auto critic_loss = torch::mse_loss(q1, y) + torch::mse_loss(q2, y);

    critic_opt->zero_grad();
    critic_loss.backward();
    critic_opt->step();

    // Delayed policy updates
    if (total_it_ % POLICY_DELAY == 0) {
      auto actor_action = actor->forward(s);
      auto actor_loss = -critic->q1(s, actor_action).mean();

      actor_opt->zero_grad();
      actor_loss.backward();
      actor_opt->step();

      // Soft update target networks
      polyak_update(*actor, *actor_target);
      polyak_update(*critic, *critic_target);
    }
  }

  /**
   * @brief Serializes Actor and Critic network weights to disk files.
   *
   * @param actor_path  Destination path for actor weights (.pt).
   * @param critic_path Destination path for critic weights (.pt).
   */
  void save(const std::string &actor_path, const std::string &critic_path) {
    torch::save(actor, actor_path);
    torch::save(critic, critic_path);
  }

  /**
   * @brief Loads Actor and Critic weights from disk, synchronizing target networks.
   *
   * @param actor_path  Source file path for actor weights (.pt).
   * @param critic_path Source file path for critic weights (.pt).
   */
  void load(const std::string &actor_path, const std::string &critic_path) {
    torch::load(actor, actor_path, device_);
    torch::load(critic, critic_path, device_);
    copy_parameters(*actor, *actor_target);
    copy_parameters(*critic, *critic_target);
  }

private:
  /**
   * @brief Hard copy of parameters from source network to target network.
   */
  static void copy_parameters(torch::nn::Module &source,
                              torch::nn::Module &target) {
    torch::NoGradGuard no_grad;

    auto source_params = source.parameters();
    auto target_params = target.parameters();

    for (size_t i = 0; i < source_params.size(); ++i) {
      target_params[i].data().copy_(source_params[i].data());
    }
  }

  /**
   * @brief Exponential moving average (Polyak) soft update: target = (1 - tau)*target + tau*net.
   */
  static void polyak_update(torch::nn::Module &net,
                            torch::nn::Module &target_net) {
    torch::NoGradGuard no_grad;

    auto params = net.parameters();
    auto target_params = target_net.parameters();

    for (size_t i = 0; i < params.size(); ++i) {
      target_params[i].data().mul_(1.0 - TAU);
      target_params[i].data().add_(TAU * params[i].data());
    }
  }

  torch::Device device_;  ///< Target compute device (CUDA or CPU).

  Actor actor{nullptr};         ///< Online Actor network.
  Actor actor_target{nullptr};  ///< Target Actor network.

  Critic critic{nullptr};         ///< Online Twin Critic network.
  Critic critic_target{nullptr};  ///< Target Twin Critic network.

  std::shared_ptr<torch::optim::Adam> actor_opt;   ///< Adam optimizer for Actor.
  std::shared_ptr<torch::optim::Adam> critic_opt;  ///< Adam optimizer for Critic.

  int64_t total_it_ = 0;  ///< Cumulative training iterations counter.
};