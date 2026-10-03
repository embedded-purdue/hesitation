#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <torch/torch.h>
#include <vector>

#include "env.hpp"
#include "network.hpp"
#include "td3_agent.hpp"

int main(int argc, char *argv[]) {
  std::string actor_path = "actor.pt";
  std::string critic_path = "critic.pt";
  double goal_deg = 0.25;
  int64_t eval_episodes = 10;
  int frameskip = TRAIN_FRAMESKIP; // 21 (11.43 Hz)

  if (argc > 1) actor_path = argv[1];
  if (argc > 2) critic_path = argv[2];
  if (argc > 3) goal_deg = std::stod(argv[3]);
  if (argc > 4) eval_episodes = std::stoll(argv[4]);
  if (argc > 5) frameskip = std::stoi(argv[5]);

  std::cout << "Loading model:\n"
            << "  Actor:     " << actor_path << "\n"
            << "  Critic:    " << critic_path << "\n"
            << "  Goal:      " << goal_deg << " deg\n"
            << "  Episodes:  " << eval_episodes << "\n"
            << "  Frameskip: " << frameskip << " (" << (240.0 / frameskip) << " Hz)\n\n";

  TD3Agent agent;
  try {
    agent.load(actor_path, critic_path);
  } catch (const std::exception &e) {
    std::cerr << "Error loading weights: " << e.what() << "\n";
    return 1;
  }

  std::vector<double> closest_phis;
  std::vector<double> terminal_phis;
  std::vector<double> ep_rewards;

  closest_phis.reserve(eval_episodes);
  terminal_phis.reserve(eval_episodes);
  ep_rewards.reserve(eval_episodes);

  double target_qs = phi_deg_to_qs(goal_deg);
  constexpr double RAD_TO_DEG = 180.0 / M_PI;
  int successful_terminal = 0;
  int successful_closest = 0;

  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    SpacecraftAttitudeEnv env(static_cast<unsigned>(ep), target_qs, frameskip);

    State state = env.reset();

    double min_phi = state.q.angle();
    double ep_reward = 0.0;
    double final_phi = min_phi;

    bool done = false;

    while (!done) {
      Quat qdot = omega_mul(state.w, state.q) * 0.5;

      auto state_tensor = torch::tensor(
          {state.q.x, state.q.y, state.q.z, state.q.w, qdot.x, qdot.y, qdot.z,
           qdot.w, state.w.x, state.w.y, state.w.z},
          torch::TensorOptions().dtype(torch::kFloat32));

      // Deterministic greedy evaluation (noise = 0.0)
      auto action_tensor = agent.select_action(state_tensor, 0.0);

      Vec3 action(action_tensor[0].item<double>(),
                  action_tensor[1].item<double>(),
                  action_tensor[2].item<double>());

      StepResult result = env.step(action);

      state = result.state;
      ep_reward += result.reward;
      final_phi = result.phi;
      min_phi = std::min(min_phi, result.phi);
      done = result.done();
    }

    closest_phis.push_back(min_phi);
    terminal_phis.push_back(final_phi);
    ep_rewards.push_back(ep_reward);

    if (min_phi * RAD_TO_DEG <= goal_deg) successful_closest++;
    if (final_phi * RAD_TO_DEG <= goal_deg) successful_terminal++;
  }

  double avg_closest_phi =
      std::accumulate(closest_phis.begin(), closest_phis.end(), 0.0) /
      closest_phis.size();

  double avg_terminal_phi =
      std::accumulate(terminal_phis.begin(), terminal_phis.end(), 0.0) /
      terminal_phis.size();

  double avg_reward =
      std::accumulate(ep_rewards.begin(), ep_rewards.end(), 0.0) /
      ep_rewards.size();

  std::cout << "Evaluation over " << eval_episodes << " episodes (Target: " << goal_deg << " deg)\n";
  std::cout << "Average closest phi:  " << std::fixed << std::setprecision(6)
            << avg_closest_phi * RAD_TO_DEG << " deg\n";
  std::cout << "Average terminal phi: " << std::fixed << std::setprecision(6)
            << avg_terminal_phi * RAD_TO_DEG << " deg\n";
  std::cout << "Average episode reward: " << std::fixed << std::setprecision(4)
            << avg_reward << "\n";
  std::cout << "Accuracy Pass Rate (Closest <= " << goal_deg << " deg):  "
            << successful_closest << "/" << eval_episodes << " ("
            << (100.0 * successful_closest / eval_episodes) << "%)\n";
  std::cout << "Accuracy Pass Rate (Terminal <= " << goal_deg << " deg): "
            << successful_terminal << "/" << eval_episodes << " ("
            << (100.0 * successful_terminal / eval_episodes) << "%)\n";

  std::cout << "\nPer-episode results:\n";
  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    std::cout << "Episode " << std::setw(2) << ep + 1
              << ": closest phi = " << std::setw(8) << std::setprecision(6) << closest_phis[ep] * RAD_TO_DEG
              << " deg, terminal phi = " << std::setw(8) << std::setprecision(6) << terminal_phis[ep] * RAD_TO_DEG
              << " deg, reward = " << std::setw(8) << std::setprecision(3) << ep_rewards[ep] << "\n";
  }

  return 0;
}
