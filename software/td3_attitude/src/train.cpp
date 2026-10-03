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

struct TrainConfig {
  bool warmstart = false;
  std::string warmstart_actor = "";
  std::string warmstart_critic = "";

  bool auto_curriculum = false; // 2-stage curriculum from scratch
  int64_t total_episodes = 5000;
  int64_t warmup_episodes = 50;

  double lr_start = 1e-4;
  double lr_end = 1e-6;

  double goal_deg = 0.25; // LM50 spec
  std::string save_prefix = "actor_0.25deg";
  int64_t checkpoint_interval = 500;
};

void print_usage(const char *prog) {
  std::cout << "Usage: " << prog << " [options]\n"
            << "Options:\n"
            << "  --warmstart <actor.pt> <critic.pt>   Warm-start from pre-trained weights\n"
            << "  --curriculum                         Enable 2-stage curriculum from scratch (0.5 deg -> 0.25 deg)\n"
            << "  --episodes <N>                       Total training episodes (default: 5000 for warmstart, 10000 for scratch)\n"
            << "  --warmup <N>                         Warmup exploration episodes (default: 0 for warmstart, 50 for scratch)\n"
            << "  --lr-start <lr>                      Initial learning rate (default: 1e-4 for warmstart, 3e-4 for scratch)\n"
            << "  --lr-end <lr>                        Final learning rate (default: 1e-6)\n"
            << "  --goal-deg <deg>                     Target attitude tolerance in degrees (default: 0.25)\n"
            << "  --save-prefix <prefix>               Output filename prefix (default: actor_0.25deg)\n"
            << "  --checkpoint-interval <N>            Save checkpoint every N episodes (default: 500)\n"
            << "  --help                               Show this help\n";
}

TrainConfig parse_args(int argc, char *argv[]) {
  TrainConfig cfg;
  bool user_episodes = false;
  bool user_lr_start = false;
  bool user_warmup = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--warmstart" && i + 2 < argc) {
      cfg.warmstart = true;
      cfg.warmstart_actor = argv[++i];
      cfg.warmstart_critic = argv[++i];
    } else if (arg == "--curriculum") {
      cfg.auto_curriculum = true;
    } else if (arg == "--episodes" && i + 1 < argc) {
      cfg.total_episodes = std::stoll(argv[++i]);
      user_episodes = true;
    } else if (arg == "--warmup" && i + 1 < argc) {
      cfg.warmup_episodes = std::stoll(argv[++i]);
      user_warmup = true;
    } else if (arg == "--lr-start" && i + 1 < argc) {
      cfg.lr_start = std::stod(argv[++i]);
      user_lr_start = true;
    } else if (arg == "--lr-end" && i + 1 < argc) {
      cfg.lr_end = std::stod(argv[++i]);
    } else if (arg == "--goal-deg" && i + 1 < argc) {
      cfg.goal_deg = std::stod(argv[++i]);
    } else if (arg == "--save-prefix" && i + 1 < argc) {
      cfg.save_prefix = argv[++i];
    } else if (arg == "--checkpoint-interval" && i + 1 < argc) {
      cfg.checkpoint_interval = std::stoll(argv[++i]);
    } else if (arg == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    }
  }

  if (cfg.warmstart) {
    if (!user_warmup) cfg.warmup_episodes = 0;
    if (!user_lr_start) cfg.lr_start = 1e-4;
    if (!user_episodes) cfg.total_episodes = 5000;
  } else if (!cfg.auto_curriculum) {
    if (!user_warmup) cfg.warmup_episodes = 50;
    if (!user_lr_start) cfg.lr_start = 3e-4;
    if (!user_episodes) cfg.total_episodes = 10000;
  } else {
    // Auto curriculum from scratch
    if (!user_warmup) cfg.warmup_episodes = 50;
    if (!user_lr_start) cfg.lr_start = 3e-4;
    if (!user_episodes) cfg.total_episodes = 10000;
  }

  return cfg;
}

int main(int argc, char *argv[]) {
  TrainConfig cfg = parse_args(argc, argv);
  std::filesystem::create_directories("checkpoints");

  TD3Agent agent;
  ReplayBuffer buffer;

  if (cfg.warmstart) {
    std::cout << "[Warm-start] Loading weights from:\n"
              << "  Actor:  " << cfg.warmstart_actor << "\n"
              << "  Critic: " << cfg.warmstart_critic << "\n";
    agent.load(cfg.warmstart_actor, cfg.warmstart_critic);
  }

  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<double> torque_dist(-TORQUE_LIMIT, TORQUE_LIMIT);

  double target_qs = phi_deg_to_qs(cfg.goal_deg);

  std::cout << "========================================================\n"
            << "TD3 Spacecraft Attitude Training\n"
            << "  Warm-start:          " << (cfg.warmstart ? "YES" : "NO") << "\n"
            << "  Total Episodes:      " << cfg.total_episodes << "\n"
            << "  Warmup Episodes:     " << cfg.warmup_episodes << "\n"
            << "  LR Schedule:         " << cfg.lr_start << " -> " << cfg.lr_end << "\n"
            << "  Goal Tolerance:      " << cfg.goal_deg << " deg (q_s = "
            << std::setprecision(9) << target_qs << ")\n"
            << "  Frameskip:           " << TRAIN_FRAMESKIP << " (11.43 Hz)\n"
            << "  Save Prefix:         " << cfg.save_prefix << "\n"
            << "========================================================\n";

  for (int64_t ep = 0; ep < cfg.total_episodes; ++ep) {
    double current_goal_qs = target_qs;
    double lr = cfg.lr_start;

    if (cfg.auto_curriculum && !cfg.warmstart) {
      // 2-stage curriculum: Stage 1 (0..6000) at 0.5 deg; Stage 2 (6000..10000) at 0.25 deg
      int64_t stage1_end = (cfg.total_episodes * 6) / 10;
      if (ep < stage1_end) {
        current_goal_qs = GOAL_QS_0_5_DEG;
        double frac = static_cast<double>(ep) / std::max<int64_t>(1, stage1_end - 1);
        lr = 3e-4 + frac * (1e-5 - 3e-4);
      } else {
        current_goal_qs = target_qs;
        double frac = static_cast<double>(ep - stage1_end) /
                      std::max<int64_t>(1, cfg.total_episodes - stage1_end - 1);
        lr = 1e-4 + frac * (cfg.lr_end - 1e-4);
      }
    } else {
      double frac = static_cast<double>(ep) / std::max<int64_t>(1, cfg.total_episodes - 1);
      lr = cfg.lr_start + frac * (cfg.lr_end - cfg.lr_start);
    }

    agent.set_lr(lr);

    unsigned seed = rng();
    SpacecraftAttitudeEnv env(seed, current_goal_qs, TRAIN_FRAMESKIP);
    State s = env.reset();

    double ep_reward = 0.0;
    bool done = false;
    double min_phi = s.q.angle();
    double final_phi = min_phi;

    while (!done) {
      Vec3 action_vec;

      if (ep < cfg.warmup_episodes) {
        // Random exploration during initial warmup
        action_vec = Vec3(torque_dist(rng), torque_dist(rng), torque_dist(rng));
      } else {
        Quat qdot = omega_mul(s.w, s.q) * 0.5;
        auto state_tensor = torch::tensor(
            {s.q.x, s.q.y, s.q.z, s.q.w, qdot.x, qdot.y, qdot.z, qdot.w, s.w.x, s.w.y, s.w.z},
            torch::kFloat32);

        auto action_tensor = agent.select_action(state_tensor, EXPLORATION_NOISE);
        action_vec = Vec3(action_tensor[0].item<double>(),
                          action_tensor[1].item<double>(),
                          action_tensor[2].item<double>());
      }

      StepResult step = env.step(action_vec);

      Quat qdot = omega_mul(s.w, s.q) * 0.5;
      auto s_tensor = torch::tensor(
          {s.q.x, s.q.y, s.q.z, s.q.w, qdot.x, qdot.y, qdot.z, qdot.w, s.w.x, s.w.y, s.w.z},
          torch::kFloat32);

      auto action_tensor = torch::tensor(
          {action_vec.x, action_vec.y, action_vec.z}, torch::kFloat32);

      auto s2 = step.state;
      Quat qdot2 = omega_mul(s2.w, s2.q) * 0.5;
      auto s2_tensor = torch::tensor(
          {s2.q.x, s2.q.y, s2.q.z, s2.q.w, qdot2.x, qdot2.y, qdot2.z, qdot2.w, s2.w.x, s2.w.y, s2.w.z},
          torch::kFloat32);

      // Replay buffer done flag MUST be step.terminated (absorbing state, ||w|| > 0.5),
      // NOT step.truncated (time horizon limit t >= MAX_STEPS), so TD3 can bootstrap on timeout!
      buffer.push(s_tensor, action_tensor, step.reward, s2_tensor, step.terminated);

      s = step.state;
      ep_reward += step.reward;
      final_phi = step.phi;
      min_phi = std::min(min_phi, step.phi);
      done = step.done();

      if (buffer.size() >= BATCH_SIZE && ep >= cfg.warmup_episodes) {
        agent.train_step(buffer);
      }
    }

    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    if (ep % 10 == 0) {
      std::cout << "Ep " << std::setw(5) << ep
                << " | Reward: " << std::setw(8) << std::fixed << std::setprecision(2) << ep_reward
                << " | Min phi: " << std::setw(6) << std::setprecision(3) << (min_phi * RAD_TO_DEG) << " deg"
                << " | Final phi: " << std::setw(6) << std::setprecision(3) << (final_phi * RAD_TO_DEG) << " deg"
                << " | LR: " << std::scientific << std::setprecision(2) << lr << "\n";
    }

    // Periodic checkpointing
    if (ep > 0 && ep % cfg.checkpoint_interval == 0) {
      std::string ckpt_actor = "checkpoints/" + cfg.save_prefix + "_ep" + std::to_string(ep) + "_actor.pt";
      std::string ckpt_critic = "checkpoints/" + cfg.save_prefix + "_ep" + std::to_string(ep) + "_critic.pt";
      agent.save(ckpt_actor, ckpt_critic);
      std::cout << "--> Saved checkpoint: " << ckpt_actor << "\n";
    }
  }

  std::string final_actor = cfg.save_prefix + ".pt";
  std::string final_critic = cfg.save_prefix + "_critic.pt";
  agent.save(final_actor, final_critic);

  // Also save nominal actor.pt and critic.pt for convenience
  agent.save("actor.pt", "critic.pt");

  std::cout << "\n[Training Complete]\n"
            << "Saved final policy to:\n"
            << "  " << final_actor << "\n"
            << "  " << final_critic << "\n"
            << "  actor.pt (current link)\n"
            << "  critic.pt (current link)\n";

  return 0;
}
