#pragma once
#include "device.hpp"
#include <algorithm>
#include <torch/torch.h>
#include <tuple>

// Preallocated ring buffer living on the compute device.
class ReplayBuffer {
public:
  ReplayBuffer(size_t capacity = 1'000'000, torch::Device device = get_device(),
               int64_t state_dim = 11, int64_t action_dim = 3)
      : capacity_(capacity), device_(device) {
    auto f = torch::TensorOptions().dtype(torch::kFloat32).device(device_);
    const int64_t cap = static_cast<int64_t>(capacity_);
    s_ = torch::zeros({cap, state_dim}, f);
    a_ = torch::zeros({cap, action_dim}, f);
    r_ = torch::zeros({cap}, f);
    s2_ = torch::zeros({cap, state_dim}, f);
    d_ = torch::zeros({cap}, f);
  }

  // s, a, s2 are CPU float32 tensors; copy_ handles the transfer.
  void push(const torch::Tensor &s, const torch::Tensor &a, double r,
            const torch::Tensor &s2, bool done) {
    s_[idx_].copy_(s);
    a_[idx_].copy_(a);
    r_[idx_].fill_(r);
    s2_[idx_].copy_(s2);
    d_[idx_].fill_(done ? 1.0 : 0.0);

    idx_ = (idx_ + 1) % static_cast<int64_t>(capacity_);
    size_ = std::min(size_ + 1, capacity_);
  }

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor,
             torch::Tensor>
  sample(size_t batch_size) {
    auto ids = torch::randint(0, static_cast<int64_t>(size_),
                              {static_cast<int64_t>(batch_size)},
                              torch::TensorOptions()
                                  .dtype(torch::kInt64)
                                  .device(device_));
    return {s_.index_select(0, ids), a_.index_select(0, ids),
            r_.index_select(0, ids), s2_.index_select(0, ids),
            d_.index_select(0, ids)};
  }

  size_t size() const { return size_; }

private:
  size_t capacity_;
  size_t size_ = 0;
  int64_t idx_ = 0;
  torch::Device device_;
  torch::Tensor s_, a_, r_, s2_, d_;
};