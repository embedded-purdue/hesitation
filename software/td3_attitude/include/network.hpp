#pragma once

#include "env.hpp"
#include <torch/torch.h>

constexpr int64_t STATE_DIM = 11;
constexpr int64_t ACTION_DIM = 3;

// Reuse the torque limit from env.hpp
constexpr double ACTION_SCALE = TORQUE_LIMIT;

struct ActorImpl : torch::nn::Module {
  ActorImpl(int64_t state_dim = STATE_DIM, int64_t action_dim = ACTION_DIM) {

    fc1 = register_module("fc1", torch::nn::Linear(state_dim, 400));

    fc2 = register_module("fc2", torch::nn::Linear(400, 300));

    out = register_module("out", torch::nn::Linear(300, action_dim));
  }

  torch::Tensor forward(torch::Tensor s) {
    auto x = torch::relu(fc1(s));
    x = torch::relu(fc2(x));
    return ACTION_SCALE * torch::tanh(out(x));
  }

  torch::nn::Linear fc1{nullptr};
  torch::nn::Linear fc2{nullptr};
  torch::nn::Linear out{nullptr};
};

TORCH_MODULE(Actor);

struct CriticImpl : torch::nn::Module {
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

  torch::Tensor q1(torch::Tensor s, torch::Tensor a) {
    auto sa = torch::cat({s, a}, /*dim=*/-1);
    auto q1 = torch::relu(q1_fc1(sa));
    q1 = torch::relu(q1_fc2(q1));
    return q1_out(q1).squeeze(-1);
  }

  torch::nn::Linear q1_fc1{nullptr}, q1_fc2{nullptr}, q1_out{nullptr};
  torch::nn::Linear q2_fc1{nullptr}, q2_fc2{nullptr}, q2_out{nullptr};
};
TORCH_MODULE(Critic);
