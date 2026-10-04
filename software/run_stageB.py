#!/usr/bin/env python3
import os
import sys
import glob
import re
import json
import time
import shutil
import subprocess

PROJECT_ROOT = "/home/eroxyi/Desktop/Projects/hesitation/software/td3_attitude"
SCRATCH_DIR = os.path.expanduser("~/.gemini/antigravity-cli/brain/74217c02-45d5-4c02-b663-726970ec1083/scratch")
os.makedirs(SCRATCH_DIR, exist_ok=True)
os.chdir(PROJECT_ROOT)

print(f"[{time.strftime('%X')}] Starting Stage B training (10,000 episodes)...", flush=True)

train_cmd = [
    "./bin/train",
    "--episodes", "10000",
    "--warmup", "500",
    "--lr-start", "3e-4",
    "--lr-end", "1e-6",
    "--noise-start", "0.05",
    "--noise-end", "0.005",
    "--goal-switch-frac", "0.6",
    "--goal-start-deg", "0.5",
    "--goal-final-deg", "0.25",
    "--save-prefix", "stageB",
    "--checkpoint-interval", "250"
]

train_log_path = os.path.join(SCRATCH_DIR, "stageB_train.log")
with open(train_log_path, "w") as f:
    p = subprocess.Popen(train_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in p.stdout:
        f.write(line)
        f.flush()
        if "Ep " in line or "Checkpoint:" in line or "Saved final" in line or "Training Complete" in line:
            print(line.strip(), flush=True)
    p.wait()
    if p.returncode != 0:
        print(f"Training failed with return code {p.returncode}", file=sys.stderr, flush=True)
        sys.exit(p.returncode)

print(f"\n[{time.strftime('%X')}] Training completed successfully! Evaluating all checkpoints...", flush=True)

# Find all checkpoints
actor_files = sorted(glob.glob("checkpoints/stageB_ep*_actor.pt"))
actor_files.sort(key=lambda x: int(re.search(r"stageB_ep(\d+)_actor", x).group(1)))
if os.path.exists("stageB.pt"):
    actor_files.append("stageB.pt")

eval_log_path = os.path.join(SCRATCH_DIR, "stageB_eval.log")
results = []

with open(eval_log_path, "w") as log_f:
    for actor in actor_files:
        if actor == "stageB.pt":
            critic = "stageB_critic.pt"
            label = "stageB_ep10000 (final)"
        else:
            ep = re.search(r"stageB_ep(\d+)_actor", actor).group(1)
            critic = f"checkpoints/stageB_ep{ep}_critic.pt"
            label = f"stageB_ep{ep}"

        print(f"[{time.strftime('%X')}] Evaluating {label}...", flush=True)
        eval_cmd = ["./bin/eval", actor, critic, "0.25", "300", "21", "10000"]
        res = subprocess.run(eval_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        log_f.write(f"=== {label} ===\n" + res.stdout + "\n")
        log_f.flush()

        out = res.stdout
        m_pass = re.search(r"Pass Rate \(Terminal <= 0\.25 deg\):\s*(\d+)/(\d+)\s*\(([\d\.]+)%\)", out)
        m_closest = re.search(r"Closest phi \(mean\):\s*([\d\.]+)\s*deg", out)
        m_term = re.search(r"Terminal phi \(mean\):\s*([\d\.]+)\s*deg", out)
        m_failures = re.search(r"Large-slew failures \(closest >= 10 deg OR omega-term\):\s*(\d+)/(\d+)", out)
        m_raw_a_star = re.search(r"\|\|raw_a\*\|\|\s*=\s*([\d\.]+)\s*Nm", out)

        pass_count = int(m_pass.group(1)) if m_pass else 0
        total_ep = int(m_pass.group(2)) if m_pass else 300
        pass_pct = float(m_pass.group(3)) if m_pass else 0.0
        mean_closest = float(m_closest.group(1)) if m_closest else 999.0
        mean_term = float(m_term.group(1)) if m_term else 999.0
        failures = int(m_failures.group(1)) if m_failures else 999
        raw_a_norm = float(m_raw_a_star.group(1)) if m_raw_a_star else 0.0

        # Extract list of failing episodes
        fail_eps = []
        for line in out.splitlines():
            m_fail_line = re.match(r"^\s*Ep\s+(\d+):\s+init=([\d\.]+)\s+deg\s+closest=([\d\.]+)\s+deg\s+terminal=([\d\.]+)\s+deg.*(\[.*\])", line)
            if m_fail_line:
                fail_eps.append({
                    "ep": int(m_fail_line.group(1)),
                    "init_deg": float(m_fail_line.group(2)),
                    "closest_deg": float(m_fail_line.group(3)),
                    "terminal_deg": float(m_fail_line.group(4)),
                    "tag": m_fail_line.group(5)
                })

        entry = {
            "label": label,
            "actor": actor,
            "critic": critic,
            "pass_count": pass_count,
            "total_ep": total_ep,
            "pass_pct": pass_pct,
            "mean_closest": mean_closest,
            "mean_term": mean_term,
            "failures": failures,
            "raw_a_norm": raw_a_norm,
            "fail_eps": fail_eps
        }
        results.append(entry)
        print(f"  -> Pass: {pass_count}/{total_ep} ({pass_pct:.2f}%) | Closest: {mean_closest:.4f} deg | Term: {mean_term:.4f} deg | Failures: {failures}", flush=True)

# Save JSON results
summary_json_path = os.path.join(SCRATCH_DIR, "stageB_summary.json")
with open(summary_json_path, "w") as f:
    json.dump(results, f, indent=2)

# Select best checkpoint
# Success criteria: Terminal pass at 0.25 deg >= 95%, and 0 of 300 failures (closest >= 10 deg or omega-term).
qualifying = [r for r in results if r["failures"] == 0 and r["pass_pct"] >= 95.0]
if qualifying:
    best = max(qualifying, key=lambda r: (r["pass_pct"], -r["mean_term"]))
    print(f"\n[SUCCESS] Qualifying checkpoint found: {best['label']} with {best['pass_pct']:.2f}% pass rate and 0 failures!", flush=True)
else:
    best = min(results, key=lambda r: (r["failures"], -r["pass_pct"], r["mean_term"]))
    print(f"\n[NOTICE] No checkpoint met >=95% with 0 failures. Best overall: {best['label']} ({best['pass_pct']:.2f}%, {best['failures']} failures)", flush=True)

shutil.copy(best["actor"], "best_actor.pt")
shutil.copy(best["critic"], "best_critic.pt")
print(f"Copied {best['actor']} -> best_actor.pt", flush=True)
print(f"Copied {best['critic']} -> best_critic.pt", flush=True)

print(f"\n[{time.strftime('%X')}] Stage B complete!", flush=True)
