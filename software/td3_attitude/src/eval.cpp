/**
 * @file eval.cpp
 * @brief Deterministic greedy evaluation and diagnostic reporting executable for TD3 attitude policies.
 *
 * Evaluates trained Actor checkpoints across pseudo-random held-out environment seeds:
 *   - Greedy deterministic control action selection (zero exploration noise).
 *   - Evaluates terminal pointing accuracy at 0.50 deg and 0.25 deg (LM50 specification).
 *   - Tracks closest attitude error, terminal attitude error, and terminal angular rate ||w||.
 *   - Detects and categorizes large-slew failure modes (closest phi >= 10 deg or ||w|| > 0.5 rad/s).
 *   - Reports underlying network torque bias a* = raw_pi(s*) and verified centered policy output pi(s*).
 *   - Supports --seed-offset argument to guarantee evaluation on unseen initial conditions.
 *   - Supports --center diagnostic override flag.
 *
 * Usage:
 *   ./bin/eval [actor.pt] [critic.pt] [goal_deg] [episodes] [frameskip] [seed_offset] [--center]
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * Reference:
 *   Elkins et al., "Spacecraft Attitude Control Using Deep Reinforcement Learning,"
 *   AAS 20-475.
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <torch/torch.h>
#include <vector>

#include "env.hpp"
#include "network.hpp"
#include "td3_agent.hpp"

/**
 * @brief Main evaluation executable entry point.
 */
int main(int argc, char *argv[]) {
  std::string actor_path    = "models/best/actor.pt";
  std::string critic_path   = "models/best/critic.pt";
  if (!std::filesystem::exists(actor_path) && std::filesystem::exists("actor.pt")) {
    actor_path = "actor.pt";
    critic_path = "critic.pt";
  }
  double      goal_deg      = 0.5;
  int64_t     eval_episodes = 300;
  int         frameskip     = TRAIN_FRAMESKIP;  // 21 by default (11.43 Hz)
  int64_t     seed_offset   = 10000;            // Held-out seed offset to avoid train-eval contamination
  bool        center        = false;

  // Flexible argument parsing supporting both flags and positional options
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--center") {
      center = true;
    } else if (arg == "--help") {
      std::cout << "Usage: " << argv[0]
                << " [actor.pt] [critic.pt] [goal_deg] [episodes] [frameskip] [seed_offset] [--center]\n";
      return 0;
    } else {
      positional.push_back(arg);
    }
  }

  if (positional.size() > 0) actor_path     = positional[0];
  if (positional.size() > 1) critic_path    = positional[1];
  if (positional.size() > 2) goal_deg       = std::stod(positional[2]);
  if (positional.size() > 3) eval_episodes  = std::stoll(positional[3]);
  if (positional.size() > 4) frameskip      = std::stoi(positional[4]);
  if (positional.size() > 5) seed_offset    = std::stoll(positional[5]);

  std::cout << "Loading model:\n"
            << "  Actor:       " << actor_path   << "\n"
            << "  Critic:      " << critic_path  << "\n"
            << "  Goal:        " << goal_deg      << " deg\n"
            << "  Episodes:    " << eval_episodes << "\n"
            << "  Frameskip:   " << frameskip
            << " (" << (240.0 / frameskip) << " Hz)\n"
            << "  Seed offset: " << seed_offset   << "\n"
            << "  Center:      " << (center ? "YES (pi(s) - a*)" : "NO") << "\n\n";

  TD3Agent agent;
  try {
    agent.load(actor_path, critic_path);
  } catch (const std::exception &e) {
    std::cerr << "Error loading weights: " << e.what() << "\n";
    return 1;
  }

  // Diagnostic equilibrium state evaluation: s* = {q = [0,0,0,1], w = [0,0,0]}
  Vec3 raw_a_star = agent.raw_a_star();
  State s_star{Vec3(0, 0, 0), Quat(0, 0, 0, 1)};
  torch::Tensor s_star_tensor = make_state_tensor(s_star);
  torch::Tensor a_star_tensor = agent.select_action(s_star_tensor, 0.0);
  Vec3 a_star(a_star_tensor[0].item<double>(),
              a_star_tensor[1].item<double>(),
              a_star_tensor[2].item<double>());

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "Underlying raw net a*        = ["
            << raw_a_star.x << ", " << raw_a_star.y << ", " << raw_a_star.z << "] Nm  (||raw_a*|| = "
            << raw_a_star.norm() << " Nm)\n";
  std::cout << "Policy output at goal pi(s*) = ["
            << a_star.x << ", " << a_star.y << ", " << a_star.z << "] Nm  (||pi(s*)|| = "
            << a_star.norm() << " Nm)\n\n";

  double target_qs = phi_deg_to_qs(goal_deg);
  constexpr double RAD_TO_DEG = 180.0 / M_PI;

  std::vector<double> closest_phis;
  std::vector<double> terminal_phis;
  std::vector<double> terminal_omegas;
  std::vector<double> ep_rewards;
  std::vector<double> init_phis;
  std::vector<bool>   omega_terms;

  closest_phis.reserve(eval_episodes);
  terminal_phis.reserve(eval_episodes);
  terminal_omegas.reserve(eval_episodes);
  ep_rewards.reserve(eval_episodes);
  init_phis.reserve(eval_episodes);
  omega_terms.reserve(eval_episodes);

  int successful_terminal = 0;
  int successful_closest  = 0;
  int term_pass_0_5       = 0;
  int term_pass_0_25      = 0;

  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    // Unique held-out seed per episode
    unsigned seed = static_cast<unsigned>(seed_offset + ep);
    SpacecraftAttitudeEnv env(seed, target_qs, frameskip);

    State  state    = env.reset();
    double init_phi = state.q.angle();
    double min_phi  = init_phi;
    double ep_reward = 0.0;
    double final_phi = init_phi;
    bool   omega_terminated = false;
    bool   done    = false;

    while (!done) {
      auto s_tensor = make_state_tensor(state);
      // Greedy deterministic evaluation without exploration noise
      auto a_tensor = agent.select_action(s_tensor, 0.0);
      Vec3 action(a_tensor[0].item<double>(),
                  a_tensor[1].item<double>(),
                  a_tensor[2].item<double>());

      if (center) {
        action = action - a_star;
        action = Vec3(std::clamp(action.x, -TORQUE_LIMIT, TORQUE_LIMIT),
                      std::clamp(action.y, -TORQUE_LIMIT, TORQUE_LIMIT),
                      std::clamp(action.z, -TORQUE_LIMIT, TORQUE_LIMIT));
      }

      StepResult result = env.step(action);

      state    = result.state;
      ep_reward += result.reward;
      final_phi  = result.phi;
      min_phi    = std::min(min_phi, result.phi);
      if (result.terminated) omega_terminated = true;
      done       = result.done();
    }

    closest_phis.push_back(min_phi);
    terminal_phis.push_back(final_phi);
    terminal_omegas.push_back(state.w.norm());
    ep_rewards.push_back(ep_reward);
    init_phis.push_back(init_phi);
    omega_terms.push_back(omega_terminated);

    if (min_phi   * RAD_TO_DEG <= goal_deg) successful_closest++;
    if (final_phi * RAD_TO_DEG <= goal_deg) successful_terminal++;
    if (final_phi * RAD_TO_DEG <= 0.50)    term_pass_0_5++;
    if (final_phi * RAD_TO_DEG <= 0.25)    term_pass_0_25++;
  }

  // ── Summary Statistical Aggregations ─────────────────────────────────
  double avg_closest_phi   = std::accumulate(closest_phis.begin(), closest_phis.end(), 0.0) / closest_phis.size();
  double avg_terminal_phi  = std::accumulate(terminal_phis.begin(), terminal_phis.end(), 0.0) / terminal_phis.size();
  double min_terminal_phi  = *std::min_element(terminal_phis.begin(), terminal_phis.end());
  double max_terminal_phi  = *std::max_element(terminal_phis.begin(), terminal_phis.end());
  double min_closest_phi   = *std::min_element(closest_phis.begin(), closest_phis.end());
  double max_closest_phi   = *std::max_element(closest_phis.begin(), closest_phis.end());
  double avg_term_omega    = std::accumulate(terminal_omegas.begin(), terminal_omegas.end(), 0.0) / terminal_omegas.size();
  double avg_reward        = std::accumulate(ep_rewards.begin(), ep_rewards.end(), 0.0) / ep_rewards.size();

  std::cout << "========================================================\n";
  std::cout << "Evaluation Summary (" << eval_episodes << " episodes, frameskip " << frameskip << ")\n";
  std::cout << "  Center flag:           " << (center ? "ENABLED" : "DISABLED") << "\n";
  std::cout << "  Action offset a*:      [" << a_star.x << ", " << a_star.y << ", " << a_star.z << "] Nm (norm " << a_star.norm() << " Nm)\n";
  std::cout << "  Average initial phi:   " << std::setprecision(4)
            << std::accumulate(init_phis.begin(), init_phis.end(), 0.0) / init_phis.size() * RAD_TO_DEG << " deg\n";
  std::cout << "  Terminal phi (mean):   " << std::setprecision(6) << avg_terminal_phi * RAD_TO_DEG << " deg\n";
  std::cout << "  Terminal phi (min):    " << std::setprecision(6) << min_terminal_phi * RAD_TO_DEG << " deg\n";
  std::cout << "  Terminal phi (max):    " << std::setprecision(6) << max_terminal_phi * RAD_TO_DEG << " deg\n";
  std::cout << "  Closest phi (mean):    " << std::setprecision(6) << avg_closest_phi  * RAD_TO_DEG << " deg\n";
  std::cout << "  Closest phi (min):     " << std::setprecision(6) << min_closest_phi  * RAD_TO_DEG << " deg\n";
  std::cout << "  Closest phi (max):     " << std::setprecision(6) << max_closest_phi  * RAD_TO_DEG << " deg\n";
  std::cout << "  Mean terminal ||w||:   " << std::scientific << std::setprecision(6) << avg_term_omega << " rad/s\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  Average episode reward:" << avg_reward << "\n";
  std::cout << "  Pass Rate (Terminal <= 0.50 deg): "
            << term_pass_0_5  << "/" << eval_episodes
            << " (" << (100.0 * term_pass_0_5  / eval_episodes) << "%)\n";
  std::cout << "  Pass Rate (Terminal <= 0.25 deg): "
            << term_pass_0_25 << "/" << eval_episodes
            << " (" << (100.0 * term_pass_0_25 / eval_episodes) << "%)\n";

  // ── Large-Slew Failure Analysis ───────────────────────────────────────
  int failure_count = 0;
  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    if (closest_phis[ep] * RAD_TO_DEG >= 10.0 || omega_terms[ep]) {
      failure_count++;
    }
  }
  std::cout << "  Large-slew failures (closest >= 10 deg OR omega-term): "
            << failure_count << "/" << eval_episodes << "\n";
  std::cout << "========================================================\n";

  // ── Detailed Per-Episode Results ──────────────────────────────────────
  std::cout << "\nPer-episode results:\n";
  std::cout << std::setw(5)  << "Ep"
            << std::setw(10) << "Init(deg)"
            << std::setw(13) << "Closest(deg)"
            << std::setw(13) << "Terminal(deg)"
            << std::setw(15) << "Term||w||(rad/s)"
            << std::setw(12) << "Reward"
            << std::setw(9)  << "OmegaTerm"
            << "\n";
  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    bool failure = (closest_phis[ep] * RAD_TO_DEG >= 10.0 || omega_terms[ep]);
    std::cout << std::setw(5)  << (ep + 1)
              << std::setw(10) << std::fixed << std::setprecision(3)
              << (init_phis[ep]       * RAD_TO_DEG)
              << std::setw(13) << std::setprecision(6)
              << (closest_phis[ep]    * RAD_TO_DEG)
              << std::setw(13) << std::setprecision(6)
              << (terminal_phis[ep]   * RAD_TO_DEG)
              << std::setw(15) << std::scientific << std::setprecision(4)
              << terminal_omegas[ep]
              << std::setw(12) << std::fixed << std::setprecision(3)
              << ep_rewards[ep]
              << std::setw(9)  << (omega_terms[ep] ? "YES" : "no")
              << (failure ? "  <-- FAIL" : "")
              << "\n";
  }

  // ── Specific Failure Breakdown Listing ────────────────────────────────
  std::cout << "\nFailing episodes (closest >= 10 deg OR omega-term):\n";
  bool any = false;
  for (int64_t ep = 0; ep < eval_episodes; ++ep) {
    if (closest_phis[ep] * RAD_TO_DEG >= 10.0 || omega_terms[ep]) {
      any = true;
      std::cout << "  Ep " << std::setw(3) << (ep + 1)
                << ": init=" << std::setw(7) << std::setprecision(3)
                << (init_phis[ep] * RAD_TO_DEG) << " deg"
                << "  closest=" << std::setw(9) << std::setprecision(4)
                << (closest_phis[ep] * RAD_TO_DEG) << " deg"
                << "  terminal=" << std::setw(9)
                << (terminal_phis[ep] * RAD_TO_DEG) << " deg"
                << "  term_w=" << std::setw(10) << std::scientific << std::setprecision(3)
                << terminal_omegas[ep] << " rad/s"
                << "  reward=" << std::setw(8) << std::fixed << std::setprecision(2)
                << ep_rewards[ep]
                << (omega_terms[ep] ? "  [OMEGA-TERM]" : "  [STUCK]")
                << "\n";
    }
  }
  if (!any) std::cout << "  (none)\n";

  return 0;
}
