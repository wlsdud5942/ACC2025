# DQN

This module computes a **visit sequence** for user-specified **stops / waypoints** on a **directed graph** (nodes `0..59`). It uses a tiny **Deep Q-Network (NumPy-only)** with a robust **BFS fallback** to avoid dead-ends and loops. The final plan is written to **`dqn_paths.json`**, which `helper_path_sender` consumes directly for runtime streaming.

---

## 0) Purpose

- From a list of stops `[a1, a2, …, an]`, produce an **ordered route** that starts at `0`, visits all stops once, and ends at `59`.
- Penalize **loops** and **revisits**, gently encourage progress through **branch nodes**, and prefer short transitions.
- Output includes both **per-leg node sequences** (`p1`, `p2`, … in `paths`) and a **global concatenated sequence** (`total`).

---

## 1) Graph Model & Reward (exactly as in code)

**Graph**
- Directed adjacency: `graph = { node: [neighbors] }` with nodes `0…59`.
- Optional metadata: out-degree, SCC membership (precomputed once at startup).

**Rewards** (per step from `s → a`, reaching `next`):
- Goal reached (`next == goal`) → **+100**
- Branch node (`out_degree(next) > 1`) → **+5**
- Loop node (`next` ∈ SCC with size > 1) → **−1000**
- Ordinary move → **−1**
- Revisit penalty → **−PENALTY_P × visits(next)`**, with `PENALTY_P = 10`

**Why this works**
- Large negative for loops kills oscillation.
- Revisit penalty suppresses needless back-and-forth.
- Small positive on branches helps the agent commit through decisions.
- If DQN stalls or fails within `max_steps`, **BFS** gives a deterministic fallback.

---

## 2) Learning & Inference Pipeline

**Model**
- `SimpleDQN(in=num_nodes, hid=64, out=num_nodes)` — two-layer MLP with ReLU; **NumPy** only (no PyTorch/TF).

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
- Emit `paths = {"p1": [...], "p2": [...], ...}` per leg, and `total` for the full tour.

---

## 3) ROS2 Node Interface

**Topics**
- **Input**: `/dqn_path_input` (`std_msgs/Int32MultiArray`)
  - Example payload: `[4, 7, 13]` → internally expands to `[0, 4, 7, 13, 59]`.
- **Outputs**
  - `/dqn_done` (`std_msgs/Bool`) → `True` when `dqn_paths.json` is ready
  - `/dqn_path_result` (`std_msgs/String`) → pretty-printed JSON (same content as file)
  - **File**: `dqn_paths.json`

**QoS**
- `RELIABLE`, `KEEP_LAST(10)`, `TRANSIENT_LOCAL` (late joiners still receive the last result).

**Services (optional)**
- `~clear_cache` (std_srvs/Trigger): drop cached pairwise results.
- `~seed` (std_srvs/SetBool or custom): toggle fixed seed determinism.

---

## 4) Parameters

| Parameter | Type | Default | Notes |
|---|---|---:|---|
| `episodes` | int | 5000 | DQN training episodes per pair.
| `max_steps` | int | 100 | Max steps per episode.
| `epsilon_start` | float | 0.5 | Linear decay start.
| `epsilon_end` | float | 0.0 | Linear decay end.
| `alpha` | float | 1e-3 | Learning rate.
| `gamma` | float | 0.9 | Discount factor.
| `hidden_dim` | int | 64 | DQN hidden layer width.
| `penalty_revisit` | float | 10.0 | `PENALTY_P`.
| `reward_goal` | float | 100.0 | Goal reward.
| `reward_branch` | float | 5.0 | Branch node bonus.
| `penalty_loop` | float | 1000.0 | SCC loop penalty.
| `use_bfs_fallback` | bool | true | Deterministic fallback.
| `seed` | int | 1234 | RNG seed (set `<0` to disable fixed seed).
| `graph_source` | string | `config/graph.yaml` | Adjacency list.
| `output_path` | string | `dqn_paths.json` | Output file path.

---

## 5) `dqn_paths.json` Schema & Example

**Canonical schema** :
```json
{
  "paths": {
    "p1": [0, 1, 2, 3, 4],
    "p2": [4, 5, 6, 7],
    "p3": [7, 8, 9, 39, 40, 41]
  },
  "total": ["p1", "p2", "p3"],
  "stops": ["p1", "p3"],
  "path_mode_map": {"p1": 0, "p2": 1, "p3": 2}
}
```
- `paths[p_k]` is the **node sequence** for leg `k`.
- `total` is the **ordered list of leg IDs** composing the full tour.
- Optional `stops` and `path_mode_map` are used by the FSM/controller.



---

## 6) Usage & Workflow

1. Launch the node (package names per your tree):
   ```bash
   ros2 run planning dqn_path_planner
   ```
2. Publish stops to the input:
   ```bash
   ros2 topic pub /dqn_path_input std_msgs/Int32MultiArray '{data: [4, 7, 13]}'
   ```
3. Watch for `/dqn_done == True` and inspect `/dqn_path_result`.
4. Confirm `dqn_paths.json` is written, then start `helper_path_sender` to stream according to `total`.

**Notes**
- On startup, the node publishes `/dqn_done=False`, then `True` on completion.
- Update the **graph topology** (`graph.yaml`) whenever the track changes.

---

## 7) Determinism, Performance, Tuning

**Determinism**
- Fix seeds for `random` and `numpy`; set `seed >= 0` in params; optionally cache solved pairwise paths.

**Performance**
- Reduce `episodes` (e.g., 1000–3000) for easy pairs; increase `hidden_dim` for larger graphs.
- Precompute frequent pairs offline and store them; enable **BFS-only** mode for trivial subgraphs.

**BFS-only mode**
- For time-critical deployments, use BFS for “easy” pairs and keep DQN only for ambiguous regions.

---

## 8) Known Limitations
- Graph must be **sane** (connected enough) for DQN/BFS to find feasible routes.
- Reward shaping is hand-tuned; for very large graphs consider heuristic search or MILP.
- NumPy DQN is **CPU-only** and minimal by design; migrate to PyTorch/TensorRT if scaling up.
