/**
 * @file network.hpp
 * @brief PyTorch/LibTorch Actor and Twin Critic neural network architectures for TD3.
 *
 * Defines the deep neural network modules implementing the continuous Actor-Critic
 * architecture for spacecraft attitude control:
 *
 * - State dimension:  11 (q: 4, qdot/OMEGA_LIMIT: 4, w/OMEGA_LIMIT: 3)
 * - Action dimension: 3 (control torque vector [Nm], scaled by TORQUE_LIMIT = 0.5)
 *
 * Actor Architecture:
 *   Linear(11 -> 400) -> ReLU -> Linear(400 -> 300) -> ReLU -> Linear(300 -> 3) -> Tanh * ACTION_SCALE
 *   Equilibrium Centering: pi(s) = clamp(raw_pi(s) - raw_pi(s*), -0.5, 0.5)
 *   where s* = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0] is the zero-error equilibrium attitude.
 *   This algebraically eliminates rest-point torque bias.
 *
 * Critic Architecture:
 *   Twin Q-networks (Q1 and Q2) to mitigate overestimation bias in TD3.
 *   Input: State (11) + Action (3) = 14 dimensions.
 *   Linear(14 -> 400) -> ReLU -> Linear(400 -> 300) -> ReLU -> Linear(300 -> 1)
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

#include "env.hpp"
#include <torch/torch.h>

/// State vector dimensionality: 4 (quaternion) + 4 (scaled qdot) + 3 (scaled omega) = 11.
constexpr int64_t STATE_DIM = 11;

/// Action vector dimensionality: 3 (body torque components [Mx, My, Mz]).
constexpr int64_t ACTION_DIM = 3;

/// Maximum action limit [Nm], matching the actuator torque limit from env.hpp.
constexpr double ACTION_SCALE = TORQUE_LIMIT;

/**
 * @brief Deep Actor network mapping attitude state to continuous body control torques.
 */
struct ActorImpl : torch::nn::Module {
  /**
   * @brief Constructs the Actor network layers.
   *
   * @param state_dim  Dimensionality of input state space (default: 11).
   * @param action_dim Dimensionality of output action space (default: 3).
   */
  ActorImpl(int64_t state_dim = STATE_DIM, int64_t action_dim = ACTION_DIM) {
    fc1 = register_module("fc1", torch::nn::Linear(state_dim, 400));
    fc2 = register_module("fc2", torch::nn::Linear(400, 300));
    out = register_module("out", torch::nn::Linear(300, action_dim));
  }

  /**
   * @brief Evaluates the raw uncentered feedforward pass: tanh(out) * ACTION_SCALE.
   *
   * @param s Input state tensor of shape [..., state_dim].
   * @return torch::Tensor Raw uncentered control torque in [-ACTION_SCALE, ACTION_SCALE].
   */
  torch::Tensor raw_forward(torch::Tensor s) {
    auto x = torch::relu(fc1(s));
    x = torch::relu(fc2(x));
    return ACTION_SCALE * torch::tanh(out(x));
  }

  /**
   * @brief Evaluates the centered policy: pi(s) = clamp(raw(s) - raw(s*), -limit, limit).
   *
   * Subtracts the policy output at the target equilibrium state s*
   * (q = [0,0,0,1], w = [0,0,0], index 3 is 1.0, all other 10 elements 0.0).
   * This guarantees that the control torque at zero error is algebraically 0 Nm,
   * eliminating static attitude offset floors.
   *
   * @param s Input state tensor (1D [state_dim] or 2D [batch_size, state_dim]).
   * @return torch::Tensor Centered control torque tensor clamped to [-ACTION_SCALE, ACTION_SCALE].
   */
  torch::Tensor forward(torch::Tensor s) {
    auto a_raw = raw_forward(s);
    if (s.dim() == 1) {
      auto s_star = torch::zeros_like(s);
      s_star[3] = 1.0f;
      auto a_star = raw_forward(s_star);
      return torch::clamp(a_raw - a_star, -ACTION_SCALE, ACTION_SCALE);
    } else {
      auto s_star = torch::zeros({1, s.size(1)}, s.options());
      s_star[0][3] = 1.0f;
      auto a_star = raw_forward(s_star);
      return torch::clamp(a_raw - a_star, -ACTION_SCALE, ACTION_SCALE);
    }
  }

  /**
   * @brief Evaluates and returns the uncentered network bias a* = raw_forward(s*).
   *
   * Useful for diagnostic reporting of the actor's natural zero-state offset.
   *
   * @param device Target compute device where computation is executed.
   * @return torch::Tensor 1D tensor [action_dim] representing a*.
   */
  torch::Tensor a_star(torch::Device device) {
    torch::NoGradGuard no_grad;
    auto s_star = torch::zeros({1, STATE_DIM},
                               torch::TensorOptions().dtype(torch::kFloat32).device(device));
    s_star[0][3] = 1.0f;
    return raw_forward(s_star).squeeze(0);
  }

  torch::nn::Linear fc1{nullptr};  ///< First fully-connected layer (state_dim -> 400).
  torch::nn::Linear fc2{nullptr};  ///< Second fully-connected layer (400 -> 300).
  torch::nn::Linear out{nullptr};  ///< Output linear layer (300 -> action_dim).
};

TORCH_MODULE(Actor);

/**
 * @brief Twin Critic network estimating state-action value functions Q1(s, a) and Q2(s, a).
 */
struct CriticImpl : torch::nn::Module {
  /**
   * @brief Constructs the twin Q-network layers.
   *
   * @param state_dim  Dimensionality of input state space (default: 11).
   * @param action_dim Dimensionality of input action space (default: 3).
   */
  CriticImpl(int64_t state_dim = STATE_DIM, int64_t action_dim = ACTION_DIM) {
    q1_fc1 = register_module("q1_fc1",
                             torch::nn::Linear(state_dim + action_dim, 400));
    q1_fc2 = register_module("q1_fc2", torch::nn::Linear(400, 300));
    q1_out = register_module("q1_out", torch::nn::Linear(300, 1));

    q2_fc1 = register_module("q2_fc1",
                             torch::nn::Linear(state_dim + action_dim, 400));
    q2_fc2 = register_module("q2_fc2", torch::nn::Linear(400, 300));
    q2_out = register_module("q2_out", torch::nn::Linear(300, 1));
  }

  /**
   * @brief Computes twin Q-values (Q1, Q2) for a given batch of states and actions.
   *
   * @param s State tensor of shape [batch_size, state_dim].
   * @param a Action tensor of shape [batch_size, action_dim].
   * @return std::pair<torch::Tensor, torch::Tensor> Pair of 1D tensors {q1, q2} of shape [batch_size].
   */
  std::pair<torch::Tensor, torch::Tensor> forward(torch::Tensor s,
                                                  torch::Tensor a) {
    auto sa = torch::cat({s, a}, /*dim=*/-1);

    auto q1 = torch::relu(q1_fc1(sa));
    q1 = torch::relu(q1_fc2(q1));
    q1 = q1_out(q1).squeeze(-1);

    auto q2 = torch::relu(q2_fc1(sa));
    q2 = torch::relu(q2_fc2(q2));
    q2 = q2_out(q2).squeeze(-1);

    return {q1, q2};
  }

  /**
   * @brief Computes Q1 value only, used for the deterministic policy gradient update in TD3.
   *
   * @param s State tensor of shape [batch_size, state_dim].
   * @param a Action tensor of shape [batch_size, action_dim].
   * @return torch::Tensor Q1 value tensor of shape [batch_size].
   */
  torch::Tensor q1(torch::Tensor s, torch::Tensor a) {
    auto sa = torch::cat({s, a}, /*dim=*/-1);
    auto q1 = torch::relu(q1_fc1(sa));
    q1 = torch::relu(q1_fc2(q1));
    return q1_out(q1).squeeze(-1);
  }

  torch::nn::Linear q1_fc1{nullptr}, q1_fc2{nullptr}, q1_out{nullptr};  ///< Q1 network layers.
  torch::nn::Linear q2_fc1{nullptr}, q2_fc2{nullptr}, q2_out{nullptr};  ///< Q2 network layers.
};

TORCH_MODULE(Critic);
