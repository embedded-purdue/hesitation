/**
 * @file train.cpp
 * @brief Continuous TD3 training executable for spacecraft attitude regulation with curriculum learning.
 *
 * Implements the full training pipeline for spacecraft attitude control:
 *   - Continuous two-stage curriculum: Stage 1 at 0.50 deg -> Stage 2 at 0.25 deg tolerance.
 *   - Linearly annealed Adam learning rate: lr_start -> lr_end across all episodes.
 *   - Linearly annealed exploration noise: noise_start -> noise_end [Nm].
 *   - Built-in actor policy centering at zero-error equilibrium.
 *   - Sample-time dynamic reward recomputation from experience replay buffer.
 *   - Safe non-clobbering checkpointing: writes only to specified prefix paths.
 *
 * Usage:
 *   ./bin/train [options]
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * Reference:
 *   Elkins et al., "Spacecraft Attitude Control Using Deep Reinforcement Learning,"
 *   AAS 20-475.
 */

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <torch/torch.h>

#include "env.hpp"
#include "network.hpp"
#include "replay_buffer.hpp"
#include "td3_agent.hpp"

/**
 * @brief Configuration parameters for training run.
 */
struct TrainConfig {
  int64_t total_episodes       = 10000;   ///< Total number of training episodes.
  int64_t warmup_episodes      = 500;     ///< Initial random torque exploration episodes.
  double  lr_start             = 3e-4;    ///< Initial Adam learning rate.
  double  lr_end               = 1e-6;    ///< Final Adam learning rate.
  double  noise_start          = 0.05;    ///< Initial exploration noise standard deviation [Nm].
  double  noise_end            = 0.005;   ///< Final exploration noise standard deviation [Nm].
  double  goal_switch_frac     = 0.6;     ///< Fraction of episodes executed at goal_start_deg (nominal: 0.6 = 60%).
  double  goal_start_deg       = 0.5;     ///< Curriculum Phase 1 pointing tolerance [deg].
  double  goal_final_deg       = 0.25;    ///< Curriculum Phase 2 pointing tolerance [deg].
  std::string save_prefix      = "scratch"; ///< Output file prefix for checkpoints and final weights.
  int64_t checkpoint_interval  = 250;     ///< Frequency of periodic model checkpoints in episodes.
};

/**
 * @brief Displays command-line argument usage options.
 */
void print_usage(const char *prog) {
  std::cout << "Usage: " << prog << " [options]\n"
            << "Options:\n"
            << "  --episodes <N>                Total training episodes (default: 10000)\n"
            << "  --warmup <N>                  Warmup exploration episodes (default: 500)\n"
            << "  --lr-start <lr>               Initial learning rate (default: 3e-4)\n"
            << "  --lr-end <lr>                 Final learning rate (default: 1e-6)\n"
            << "  --noise-start <Nm>            Initial exploration noise (default: 0.05)\n"
            << "  --noise-end <Nm>              Final exploration noise (default: 0.005)\n"
            << "  --goal-switch-frac <f>        Fraction of episodes at goal_start (default: 0.6)\n"
            << "  --goal-start-deg <deg>        Initial goal tolerance, degrees (default: 0.5)\n"
            << "  --goal-final-deg <deg>        Final goal tolerance, degrees (default: 0.25)\n"
            << "  --save-prefix <prefix>        Output filename prefix (default: scratch)\n"
            << "  --checkpoint-interval <N>     Save checkpoint every N episodes (default: 250)\n"
            << "  --help                        Show this help\n";
}

/**
 * @brief Parses command-line arguments into TrainConfig structure.
 */
TrainConfig parse_args(int argc, char *argv[]) {
  TrainConfig cfg;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--episodes"           && i + 1 < argc) cfg.total_episodes      = std::stoll(argv[++i]);
    else if (arg == "--warmup"        && i + 1 < argc) cfg.warmup_episodes     = std::stoll(argv[++i]);
    else if (arg == "--lr-start"      && i + 1 < argc) cfg.lr_start            = std::stod(argv[++i]);
    else if (arg == "--lr-end"        && i + 1 < argc) cfg.lr_end              = std::stod(argv[++i]);
    else if (arg == "--noise-start"   && i + 1 < argc) cfg.noise_start         = std::stod(argv[++i]);
    else if (arg == "--noise-end"     && i + 1 < argc) cfg.noise_end           = std::stod(argv[++i]);
    else if (arg == "--goal-switch-frac" && i + 1 < argc) cfg.goal_switch_frac = std::stod(argv[++i]);
    else if (arg == "--goal-start-deg"   && i + 1 < argc) cfg.goal_start_deg   = std::stod(argv[++i]);
    else if (arg == "--goal-final-deg"   && i + 1 < argc) cfg.goal_final_deg   = std::stod(argv[++i]);
    else if (arg == "--save-prefix"   && i + 1 < argc) cfg.save_prefix         = argv[++i];
    else if (arg == "--checkpoint-interval" && i + 1 < argc) cfg.checkpoint_interval = std::stoll(argv[++i]);
    else if (arg == "--help") { print_usage(argv[0]); std::exit(0); }
  }
  return cfg;
}

/**
 * @brief Main execution function running the TD3 training loop.
 */
int main(int argc, char *argv[]) {
  TrainConfig cfg = parse_args(argc, argv);
  std::filesystem::path ckpt_dir = std::filesystem::path("models/checkpoints") / cfg.save_prefix;
  std::filesystem::create_directories(ckpt_dir);

  // Episode index at which curriculum switches from goal_start_deg to goal_final_deg
  int64_t switch_ep = static_cast<int64_t>(cfg.goal_switch_frac * cfg.total_episodes);

  std::cout << "========================================================\n"
            << "TD3 Spacecraft Attitude Training (from scratch)\n"
            << "  Total Episodes:       " << cfg.total_episodes << "\n"
            << "  Warmup Episodes:      " << cfg.warmup_episodes << "\n"
            << "  LR Schedule:          " << cfg.lr_start << " -> " << cfg.lr_end << "\n"
            << "  Noise Schedule:       " << cfg.noise_start << " -> " << cfg.noise_end << " Nm\n"
            << "  Goal Phase 1:         0 - " << switch_ep << " ep @ " << cfg.goal_start_deg << " deg\n"
            << "  Goal Phase 2:         " << switch_ep << " - " << cfg.total_episodes << " ep @ " << cfg.goal_final_deg << " deg\n"
            << "  Frameskip:            " << TRAIN_FRAMESKIP << " (11.43 Hz)\n"
            << "  Save Prefix:          " << cfg.save_prefix << "\n"
            << "  Reward sigma:         " << REWARD_SIGMA * 180.0 / M_PI << " deg\n"
            << "  Reward B:             " << REWARD_B << "\n"
            << "  Reward beta:          " << REWARD_BETA << "\n"
            << "========================================================\n";

  TD3Agent agent;
  ReplayBuffer buffer;

  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<double> torque_dist(-TORQUE_LIMIT, TORQUE_LIMIT);

  for (int64_t ep = 0; ep < cfg.total_episodes; ++ep) {
    // ── Curriculum: goal switches in-place at switch_ep ─────────────────
    double current_goal_qs;
    if (ep < switch_ep) {
      current_goal_qs = phi_deg_to_qs(cfg.goal_start_deg);
    } else {
      current_goal_qs = phi_deg_to_qs(cfg.goal_final_deg);
    }

    // Anneal learning rate and exploration noise linearly across training
    double global_frac = static_cast<double>(ep) /
                         std::max<int64_t>(1, cfg.total_episodes - 1);
    double lr    = cfg.lr_start + global_frac * (cfg.lr_end - cfg.lr_start);
    double noise = cfg.noise_start + global_frac * (cfg.noise_end - cfg.noise_start);

    agent.set_lr(lr);

    unsigned seed = rng();
    SpacecraftAttitudeEnv env(seed, current_goal_qs, TRAIN_FRAMESKIP);
    State s = env.reset();

    double ep_reward = 0.0;
    bool   done      = false;
    double min_phi   = s.q.angle();
    double final_phi = min_phi;

    while (!done) {
      Vec3 action_vec;

      if (ep < cfg.warmup_episodes) {
        // Pure uniform random exploration during warmup
        action_vec = Vec3(torque_dist(rng), torque_dist(rng), torque_dist(rng));
      } else {
        auto s_tensor  = make_state_tensor(s);
        auto a_tensor  = agent.select_action(s_tensor, noise);
        action_vec = Vec3(a_tensor[0].item<double>(),
                          a_tensor[1].item<double>(),
                          a_tensor[2].item<double>());
      }

      StepResult step = env.step(action_vec);

      auto s_tensor  = make_state_tensor(s);
      auto a_tensor  = torch::tensor({action_vec.x, action_vec.y, action_vec.z},
                                     torch::kFloat32);
      auto s2_tensor = make_state_tensor(step.state);

      // Replay buffer stores transition features for dynamic sample-time reward computation.
      // Only terminated (||w|| > limit) is an absorbing state.
      // Truncation (time horizon) is NOT absorbing: TD3 bootstraps on timeout.
      buffer.push(s_tensor, a_tensor, step.phi, step.state.q.w, step.state.w.norm(),
                  s2_tensor, step.terminated);

      s         = step.state;
      ep_reward += step.reward;
      final_phi  = step.phi;
      min_phi    = std::min(min_phi, step.phi);
      done       = step.done();

      if (buffer.size() >= BATCH_SIZE && ep >= cfg.warmup_episodes) {
        agent.train_step(buffer, current_goal_qs);
      }
    }

    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    if (ep % 10 == 0) {
      std::cout << "Ep " << std::setw(6) << ep
                << " | Goal: " << std::setw(4) << std::fixed << std::setprecision(2)
                << ((ep < switch_ep) ? cfg.goal_start_deg : cfg.goal_final_deg) << "deg"
                << " | R: " << std::setw(8) << std::setprecision(2) << ep_reward
                << " | MinPhi: " << std::setw(7) << std::setprecision(3)
                << (min_phi * RAD_TO_DEG) << "deg"
                << " | FinalPhi: " << std::setw(7) << std::setprecision(3)
                << (final_phi * RAD_TO_DEG) << "deg"
                << " | LR: " << std::scientific << std::setprecision(2) << lr
                << " | Noise: " << std::fixed << std::setprecision(4) << noise << " Nm"
                << "\n";
    }

    // Periodic checkpointing
    if (ep > 0 && ep % cfg.checkpoint_interval == 0) {
      std::string ckpt_actor  = (ckpt_dir / (cfg.save_prefix + "_ep" + std::to_string(ep) + "_actor.pt")).string();
      std::string ckpt_critic = (ckpt_dir / (cfg.save_prefix + "_ep" + std::to_string(ep) + "_critic.pt")).string();
      agent.save(ckpt_actor, ckpt_critic);
      std::cout << "--> Checkpoint: " << ckpt_actor << "\n";
    }
  }

  // Final model save inside models/checkpoints/<prefix>/
  std::string final_actor  = (ckpt_dir / (cfg.save_prefix + ".pt")).string();
  std::string final_critic = (ckpt_dir / (cfg.save_prefix + "_critic.pt")).string();
  agent.save(final_actor, final_critic);

  std::cout << "\n[Training Complete]\n"
            << "Saved final policy to:\n"
            << "  " << final_actor << "\n"
            << "  " << final_critic << "\n";

  return 0;
}
