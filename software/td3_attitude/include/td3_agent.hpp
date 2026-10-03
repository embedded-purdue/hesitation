#pragma once

#include <torch/torch.h>

#include "device.hpp"
#include "network.hpp"
#include "replay_buffer.hpp"

constexpr double GAMMA = 0.99;
constexpr int64_t BATCH_SIZE = 100;

constexpr double POLICY_NOISE = 0.1 * ACTION_SCALE;
constexpr double NOISE_CLIP = 0.5 * ACTION_SCALE;
constexpr double EXPLORATION_NOISE = 0.1 * ACTION_SCALE;

constexpr int POLICY_DELAY = 2;
constexpr double TAU = 1.0 - 0.995;

class TD3Agent {
public:
  TD3Agent() : device_(get_device()) {
    actor = Actor();
    critic = Critic();
    actor_target = Actor();
    critic_target = Critic();

    // Move everything BEFORE building optimizers or copying parameters.
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

  // state: CPU float32 [state_dim]. Returns a CPU tensor [action_dim].
  torch::Tensor select_action(torch::Tensor state, double noise = 0.0) {
    torch::NoGradGuard no_grad;

    auto action = actor->forward(state.to(device_).unsqueeze(0)).squeeze(0);

    if (noise > 0.0) {
      action = action + torch::randn_like(action) * noise;
    }

    return torch::clamp(action, -ACTION_SCALE, ACTION_SCALE).to(torch::kCPU);
  }

  void set_lr(double lr) {
    for (auto &group : actor_opt->param_groups()) {
      static_cast<torch::optim::AdamOptions &>(group.options()).lr(lr);
    }

    for (auto &group : critic_opt->param_groups()) {
      static_cast<torch::optim::AdamOptions &>(group.options()).lr(lr);
    }
  }

  void train_step(ReplayBuffer &buffer) {
    total_it_++;

    auto [s, a, r, s2, d] = buffer.sample(BATCH_SIZE);

    torch::Tensor y;

    {
      torch::NoGradGuard no_grad;

      auto noise = torch::randn_like(a) * POLICY_NOISE;
      noise = torch::clamp(noise, -NOISE_CLIP, NOISE_CLIP);

      auto a2 = actor_target->forward(s2) + noise;
      a2 = torch::clamp(a2, -ACTION_SCALE, ACTION_SCALE);

      auto [q1_t, q2_t] = critic_target->forward(s2, a2);
      auto q_t = torch::min(q1_t, q2_t);

      y = r + GAMMA * (1.0 - d) * q_t;
    }

    auto [q1, q2] = critic->forward(s, a);

    auto critic_loss = torch::mse_loss(q1, y) + torch::mse_loss(q2, y);

    critic_opt->zero_grad();
    critic_loss.backward();
    critic_opt->step();

    if (total_it_ % POLICY_DELAY == 0) {
      auto actor_action = actor->forward(s);
      auto actor_loss = -critic->q1(s, actor_action).mean();

      actor_opt->zero_grad();
      actor_loss.backward();
      actor_opt->step();

      polyak_update(*actor, *actor_target);
      polyak_update(*critic, *critic_target);
    }
  }

  void save(const std::string &actor_path, const std::string &critic_path) {
    torch::save(actor, actor_path);
    torch::save(critic, critic_path);
  }

  // Works for weights saved from either CPU or GPU.
  void load(const std::string &actor_path, const std::string &critic_path) {
    torch::load(actor, actor_path, device_);
    torch::load(critic, critic_path, device_);
    copy_parameters(*actor, *actor_target);
    copy_parameters(*critic, *critic_target);
  }

private:
  static void copy_parameters(torch::nn::Module &source,
                              torch::nn::Module &target) {
    torch::NoGradGuard no_grad;

    auto source_params = source.parameters();
    auto target_params = target.parameters();

    for (size_t i = 0; i < source_params.size(); ++i) {
      target_params[i].data().copy_(source_params[i].data());
    }
  }

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

  torch::Device device_;

  Actor actor{nullptr};
  Actor actor_target{nullptr};

  Critic critic{nullptr};
  Critic critic_target{nullptr};

  std::shared_ptr<torch::optim::Adam> actor_opt;
  std::shared_ptr<torch::optim::Adam> critic_opt;

  int64_t total_it_ = 0;
};