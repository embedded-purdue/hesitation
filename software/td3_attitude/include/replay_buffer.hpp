/**
 * @file replay_buffer.hpp
 * @brief Preallocated GPU/CPU experience replay buffer with sample-time dynamic reward recomputation.
 *
 * Implements a ring buffer on the target compute device storing transition tuples:
 *   (s, a, phi', qs', ||w'||, s', terminated)
 *
 * Key Design Features:
 *   1. Preallocated Device Tensors: Avoids per-batch host-to-device memory allocation overhead.
 *   2. Sample-Time Reward Recomputation: Instead of storing a fixed scalar reward r,
 *      the buffer stores transition features (phi', qs', ||w'||, terminated) and
 *      evaluates the reward dynamically at sample time based on the currently active
 *      curriculum goal_qs. This guarantees that curriculum transitions (e.g., 0.50 deg -> 0.25 deg)
 *      do not leave stale goal bonuses in the buffer, while preserving all historical experience.
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

#include "device.hpp"
#include "env.hpp"
#include <algorithm>
#include <torch/torch.h>
#include <tuple>

/**
 * @brief Experience replay buffer living directly on the compute device (CUDA/CPU).
 */
class ReplayBuffer {
public:
  /**
   * @brief Constructs and preallocates tensor storage for the replay buffer.
   *
   * @param capacity   Maximum number of transitions stored (nominal: 1,000,000).
   * @param device     Target compute device (CUDA or CPU).
   * @param state_dim  Dimensionality of state vectors (default: 11).
   * @param action_dim Dimensionality of action vectors (default: 3).
   */
  ReplayBuffer(size_t capacity = 1'000'000, torch::Device device = get_device(),
               int64_t state_dim = 11, int64_t action_dim = 3)
      : capacity_(capacity), device_(device) {
    auto f = torch::TensorOptions().dtype(torch::kFloat32).device(device_);
    const int64_t cap = static_cast<int64_t>(capacity_);
    s_       = torch::zeros({cap, state_dim}, f);
    a_       = torch::zeros({cap, action_dim}, f);
    phi2_    = torch::zeros({cap}, f);
    qs2_     = torch::zeros({cap}, f);
    w_norm2_ = torch::zeros({cap}, f);
    s2_      = torch::zeros({cap, state_dim}, f);
    d_       = torch::zeros({cap}, f);
  }

  /**
   * @brief Appends a transition into the circular buffer.
   *
   * @param s          Current state tensor (CPU float32).
   * @param a          Action tensor applied (CPU float32).
   * @param phi2       Attitude error angle of next state s2 [rad].
   * @param qs2        Scalar component of attitude quaternion of next state s2.
   * @param w_norm2    Norm of body angular velocity of next state s2 [rad/s].
   * @param s2         Next state tensor (CPU float32).
   * @param terminated True iff next state reached an absorbing failure condition (||w|| > OMEGA_LIMIT).
   *                   Note: time-horizon truncations (t >= MAX_STEPS) MUST pass false to allow bootstrapping.
   */
  void push(const torch::Tensor &s, const torch::Tensor &a,
            double phi2, double qs2, double w_norm2,
            const torch::Tensor &s2, bool terminated) {
    s_[idx_].copy_(s);
    a_[idx_].copy_(a);
    phi2_[idx_].fill_(static_cast<float>(phi2));
    qs2_[idx_].fill_(static_cast<float>(qs2));
    w_norm2_[idx_].fill_(static_cast<float>(w_norm2));
    s2_[idx_].copy_(s2);
    d_[idx_].fill_(terminated ? 1.0f : 0.0f);

    idx_ = (idx_ + 1) % static_cast<int64_t>(capacity_);
    size_ = std::min(size_ + 1, capacity_);
  }

  /**
   * @brief Uniformly samples a random mini-batch and evaluates rewards dynamically.
   *
   * Evaluates the reward tensor on-device:
   *   r = exp(-phi' / sigma) + B * 1[qs' >= current_goal_qs] - beta * max(0, ||w'|| - 0.3)
   *   terminated => r = OMEGA_TERM_PENALTY (-25.0)
   *
   * @param batch_size      Number of transitions to sample (nominal: 100).
   * @param current_goal_qs Current active goal tolerance threshold cos(phi_goal / 2).
   * @return std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>
   *         Tensors {s, a, r, s2, d} on the compute device.
   */
  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor,
             torch::Tensor>
  sample(size_t batch_size, double current_goal_qs) {
    auto ids = torch::randint(0, static_cast<int64_t>(size_),
                              {static_cast<int64_t>(batch_size)},
                              torch::TensorOptions()
                                  .dtype(torch::kInt64)
                                  .device(device_));
    auto s_batch   = s_.index_select(0, ids);
    auto a_batch   = a_.index_select(0, ids);
    auto s2_batch  = s2_.index_select(0, ids);
    auto d_batch   = d_.index_select(0, ids);

    auto phi_batch = phi2_.index_select(0, ids);
    auto qs_batch  = qs2_.index_select(0, ids);
    auto w_batch   = w_norm2_.index_select(0, ids);

    // Vectorized dynamic reward calculation on GPU/CPU
    auto r_approach = torch::exp(-phi_batch / static_cast<float>(REWARD_SIGMA));
    auto r_goal     = (qs_batch >= static_cast<float>(current_goal_qs)).to(torch::kFloat32) * static_cast<float>(REWARD_B);
    auto r_omega    = -static_cast<float>(REWARD_BETA) * torch::clamp(w_batch - static_cast<float>(OMEGA_SOFT), /*min=*/0.0f);
    auto r_batch    = r_approach + r_goal + r_omega;
    r_batch         = torch::where(d_batch > 0.5f, torch::full_like(r_batch, static_cast<float>(OMEGA_TERM_PENALTY)), r_batch);

    return {s_batch, a_batch, r_batch, s2_batch, d_batch};
  }

  /// Current number of transitions available in the buffer.
  size_t size() const { return size_; }

private:
  size_t capacity_;        ///< Maximum ring buffer capacity.
  size_t size_ = 0;        ///< Current stored transition count.
  int64_t idx_ = 0;        ///< Write pointer index.
  torch::Device device_;   ///< Compute device holding the tensors.
  torch::Tensor s_, a_, phi2_, qs2_, w_norm2_, s2_, d_;  ///< Device storage tensors.
};