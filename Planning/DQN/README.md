# `DQN` — Visit-Order Optimization with a Lightweight DQN

This module computes a **visit sequence** for user-specified **stops / waypoints** on a **directed graph** (nodes `0..59`). It uses a tiny **Deep Q-Network (NumPy-only)** with a robust **BFS fallback** to avoid dead-ends and loops. The final plan is written to **`dqn_paths.json`**, which `helper_path_sender` consumes directly for runtime streaming.

---

## 0) Purpose

- From a list of stops `[a1, a2, …, an]`, produce an **ordered route** that starts at `0`, visits all stops once, and ends at `59`.
- Penalize **loops** and **revisits**, gently encourage progress through **branch nodes**, and prefer short transitions.
- Output includes both **per-leg node sequences** (`"p1"`, `"p2"`, …) and a **global concatenated sequence** (`"total"`).

---

## 1) Graph Model & Reward (exactly as in code)

**Graph**
- `graph = { node: [neighbors] }` with nodes `0…59` (directed).

**Rewards**
- Goal reached (`next == goal`) → **+100**
- Branch node (out-degree > 1) → **+5**
- Loop node (member of SCC with size > 1) → **−1000**
- Ordinary move → **−1**
- Revisit penalty → **−10 × (times the node was already visited)**; `PENALTY_P = 10`

**Why this works**
- Large negative for loops kills oscillation.
- Revisit penalty suppresses needless back-and-forth.
- Small positive on branches helps the agent commit through decisions.
- If DQN still fails to reach the goal, we **fall back to BFS** shortest path.

---

## 2) Learning & Inference Pipeline

**Model**
- `SimpleDQN(in=num_nodes, hid=64, out=num_nodes)`  
  Two-layer MLP with ReLU; implemented in **NumPy** (no PyTorch/TF).

**Schedule**
- `episodes = 5000`, `max_steps = 100`
- `epsilon` linearly decays **0.5 → 0.0**
- `alpha = 1e-3`, `gamma = 0.9`

**Per (start, goal)**
1. Train a fresh `SimpleDQN` with the reward shaping above.
2. At inference, from state `s` choose `argmax Q(s, a)` among allowed actions `graph[s]`.
3. If the agent stalls/cycles or fails to reach `goal` within `max_steps`, **BFS** provides a deterministic **fallback**.

**Global sequence**
- Expand `[0] + stops + [59]` pairwise: `(0→a1), (a1→a2), …, (an→59)`.
- Solve each pair; **deduplicate** consecutive nodes while concatenating.
- Emit `"p1"`, `"p2"`, … for each leg, and `"total"` for the full tour.

---

## 3) ROS2 Node Interface

**Topics**
- **Input**: `/dqn_path_input` (`std_msgs/Int32MultiArray`)  
  Example payload: `[4, 7, 13]` → internally becomes `[0, 4, 7, 13, 59]`.
- **Outputs**  
  - `/dqn_done` (`std_msgs/Bool`) → `True` when `dqn_paths.json` is ready  
  - `/dqn_path_result` (`std_msgs/String`) → pretty-printed JSON (same content as file)  
  - **File**: `dqn_paths.json`

**QoS**
- `RELIABLE`, `KEEP_LAST(10)`, `TRANSIENT_LOCAL` (late joiners still get the last result).

**`dqn_paths.json` (example)**
    
    {
      "p1": [0, 1, 2, 3, 4],
      "p2": [4, 5, 6, 7],
      "p3": [7, 8, 9, 39, 40, 41],
      "total": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 39, 40, 41, ..., 59]
    }

---

## 4) Failure & Improvement Log (highlights)

- **V0 (rule / Q-table-like)** — In branch-heavy regions the agent **looped**.  
  → Switched to a **trainable DQN**.

- **V1 (DQN)** — Reached goals but sometimes **revisited nodes**.  
  → Added **visit-count penalty** (−10 × count).

- **V1.1** — Slow convergence on some pairs.  
  → **Linear epsilon decay** + **branch-node bonus** (+5) improved exploration.

- **V2** — Rare failures persisted.  
  → Integrated a **BFS fallback**; now every pair yields a feasible path deterministically.

---

## 5) Usage & Workflow

1. Launch the node (package names per your tree):
   
       ros2 run planning dqn_path_planner

2. Publish stops to the input:
   
       ros2 topic pub /dqn_path_input std_msgs/Int32MultiArray '{data: [4, 7, 13]}'

3. Wait for:
   - `/dqn_done` → `True`
   - `/dqn_path_result` → JSON output

4. Confirm `dqn_paths.json` is written, then start `helper_path_sender` to stream according to `"total"`.

**Notes**
- At startup the node publishes `/dqn_done=False`, then `True` on completion.
- The graph topology lives in the node source; update it when the track changes.

---

## 6) Determinism, Performance, Tuning

**Determinism**
- Fix RNG seeds for `random` and `numpy` if you need reproducible runs; optionally cache solved pairwise paths.

**Performance**
- Reduce `episodes` (e.g., 1000–3000) for quicker results on easy pairs.
- Increase `hidden_dim` (64→128) for larger graphs; adjust `alpha`.
- Precompute frequent pairs offline and store them.

**BFS-only mode**
- For time-critical deployments, you can use BFS for “easy” pairs and keep DQN only for ambiguous subgraphs.

---
