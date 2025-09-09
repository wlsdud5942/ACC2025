# DQN

For a static loop, we only need to decide **which nodes to visit and in what order**. We deliberately chose a **NumPy-only DQN** so the plan is computed **immediately** with **low latency and memory**, and we keep a **BFS fallback** to guarantee a result. This spec maps 1:1 to the provided implementation (`DQNPathPlanner`).

---

## 0) Why DQN (our intent)
- **Immediate application**: no long training cycles or large memory footprints; results are available right away.  
- **Low latency & memory**: 2-layer MLP (hidden=64) + ReLU runs on CPU in milliseconds.  
- **Static-loop best fit**: the map is fixed; a tiny model is more than sufficient; no heavyweight retraining needed.  
- **Determinism guard**: if exploration stalls, **BFS** returns a valid path.

---

## 1) Graph & reward (mirrors the code)

**Graph**
- `graph: Dict<int, List<int>>` with nodes `0..59`.  
- **Branch nodes**: `out_degree > 1`.  
- **Loop nodes**: members of SCCs with size > 1 (via `networkx.strongly_connected_components`).

**Reward function** `get_reward_dynamic(goal)`
- `next == goal` → **+100** (`R_goal`): strongly drives completion.  
- `next ∈ loop_nodes` → **−1000** (`C_loop`): avoids cyclic traps.  
- `next ∈ branch_nodes` → **+5** (`R_branch`): encourages decisive progress at forks.  
- otherwise step cost → **−1** (`C_step`): biases toward shorter moves.  
- **Revisit penalty** `−PENALTY_P × visits(next)` with `PENALTY_P = 10`: suppresses back-and-forth.

> Rationale: keep a **short-path bias** while guaranteeing **safety (no loops)**, **decisiveness at forks**, and **efficiency**.

---

## 2) DQN model & learning (mirrors the code)

- `SimpleDQN(in=num_nodes, hid=64, out=num_nodes)` — **ReLU** hidden, **linear** output.  
- Loss: **MSE**, target \( y = r + \gamma \max_{a'} Q(s', a') \). (Huber is a drop-in alternative if needed.)  
- Hyper-params: `episodes=5000`, `max_steps=100`, `alpha=1e-3`, `gamma=0.9`, `epsilon_start=0.5 → 0.0`.  
- Policy: **ε-greedy** with linear decay.  
- State: **one-hot** vector of the current node.  
- Action: **only among allowed neighbors** of the current node.

**Training loop**
- See `dqn_path(start, goal)`: episode rollouts update weights online; inference selects `argmax Q` over neighbors; if no progress, fall back to `bfs_shortest_path`.

---

## 3) Output & pipeline

- From stops `[a1, …, an]`, expand to `(0→a1→…→an→59)`.  
- Solve each pair; assemble **per-leg sequences** and a **global concatenated `total`** (deduplicating repeated endpoints).  
- Save to `dqn_paths.json`, publish `/dqn_done=True` and `/dqn_path_result`.

**Example (JSON schema)**

~~~json
{
  "paths": {
    "p1": [0, 1, 2, 3, 4],
    "p2": [4, 5, 6, 7],
    "p3": [7, 8, 9, 39, 40, 41]
  },
  "total": ["p1", "p2", "p3"],
  "stops": ["p1", "p3"],
  "path_mode_map": { "p1": 0, "p2": 1, "p3": 2 }
}
~~~

---

## 4) How we evaluate

**Metrics**
- **Total travel length**, **# revisits**, **# loop entries**, **fork indecision rate**, **compute latency (ms)**, **memory (MB)**.  
- On-car: **lap time**, **off-track rate**, **event timeliness** (arrival time error vs. schedule).

**Procedure**
1. **Offline replay**: multiple random seeds → mean/std/95th percentile per metric.  
2. **Noise/missing-data tests**: inject noise or missing stops/edge costs to assess robustness.  
3. **On-car A/B**: DQN visit order vs. manual/greedy baselines.

> We log both **linear scores** and **non-linear penalties** (e.g., logistic loop penalty) for sensitivity analysis.

---

## 5) Further work

- **Exploration design**: compare linear/exponential ε-schedules, **softmax** (temperature τ), **UCB/Thompson**, **NoisyNet**.  
- **Model scaling**: for larger graphs, migrate to **PyTorch DQN**, **GNN/Transformer**-based scorers.  
- **Time-series preprocessing**: encode stop reliability / sensor events with **LSTM/NLSTM**; feed as weights into the reward.  

---

## 6) ROS 2 interface

- **Input**: `/dqn_path_input (std_msgs/Int32MultiArray)`  
- **Outputs**: `/dqn_done (std_msgs/Bool)`, `/dqn_path_result (std_msgs/String)`, file `dqn_paths.json`  
- **QoS**: RELIABLE, KEEP_LAST(10), TRANSIENT_LOCAL  
- Node name: `dqn_path_planner`; publishes `/dqn_done=False` on startup, `True` when ready.

---

## 7) Parameters

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
