# Decentralized Traffic Simulator

A C++ simulation framework for modeling and evaluating **decentralized, peer-to-peer traffic signal control** across a mesh network of intersections. The project demonstrates how traffic lights can make autonomous phase decisions using only local sensor data and neighbor-to-neighbor message passing — no central controller required.

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [Key Features](#key-features)
- [Simulation Modes](#simulation-modes)
- [Traffic Scenarios](#traffic-scenarios)
- [Performance Metrics & Level of Service](#performance-metrics--level-of-service)
- [Building & Running](#building--running)
- [Test Programs](#test-programs)
- [Design Notes](#design-notes)

---

## Overview

Traditional traffic control relies on a centralized system that has a single point of failure and limited scalability. This simulator models an alternative: each intersection runs a **DecentralizedController** that:

1. Reads its own queue lengths and wait times.
2. Exchanges lightweight status messages with neighboring intersections.
3. Independently computes an optimal signal phase using a scoring function that accounts for local demand, upstream pressure, downstream blocking, and fairness bias.

The network of intersections is arranged as an **N×N mesh**, where each node communicates only with its four cardinal neighbors.

---

## Project Structure

```
PDC_PROJECT/
├── Intersection.h              # Core intersection model, lane management, vehicle queue logic
├── CONTROLLER.h / .cpp         # Centralized (Webster-formula) controller — baseline reference
├── DecentralizedController.h / .cpp  # Peer-to-peer decentralized signal controller
├── TrafficGenerator.h / .cpp   # Stochastic vehicle spawner with configurable scenarios
├── NxNMeshNetwork.h            # N×N mesh network; supports sequential, parallel & async modes
├── MessageBuffer.h             # Message batching and run-length / delta compression helpers
├── RealTimeDeadline.h          # Real-time deadline enforcement with statistics tracking
├── TrafficSimulator.cpp        # Single-intersection simulation entry point (main)
├── test_batching.cpp           # Benchmarks: batching enabled vs disabled
└── test_deadline.cpp           # Smoke test for deadline enforcement
```

---

## Architecture

### Intersection (`Intersection.h`)

The fundamental simulation unit. Each intersection manages:

- **4 directions** × **2 lanes each** (dedicated left-turn lane + straight/right lane).
- **5 signal phases**: `NORTH_SOUTH_GREEN`, `NORTH_SOUTH_YELLOW`, `EAST_WEST_GREEN`, `EAST_WEST_YELLOW`, `PEDESTRIAN_CROSSING`.
- **Vehicle types**: `CAR`, `TRUCK`, `EMERGENCY` (emergency vehicles are priority-flagged).
- **Timing constraints**: configurable minimum green, maximum green, yellow time, and pedestrian crossing duration.
- Per-tick vehicle processing, wait time accumulation, queue length reporting, and level-of-service computation.

### Centralized Controller (`CONTROLLER.h/.cpp`)

A baseline Webster-formula controller that computes cycle length from total intersection demand and splits green time proportionally between N-S and E-W axes. Used as a performance reference.

### Decentralized Controller (`DecentralizedController.h/.cpp`)

Each controller instance:

- **Builds a `NeighborMessage`** from the local intersection state (queue lengths, wait times, current phase, predicted discharge rate).
- **Receives messages** from up to 4 neighbors via `NeighborLink` pairs.
- **Scores N-S vs E-W** using a weighted combination of:
  - Local queue length difference
  - Local average wait time difference
  - Upstream pressure from neighbors feeding into each axis
  - Downstream blocking by neighbors on each axis
  - Consensus bias (how many neighbors prefer the same axis)
  - Fairness bias (penalizes repeated selection of the same axis)
- **Selects phase and duration**, clamped to timing constraints.
- Optionally enforces a **real-time decision deadline** — if the scoring computation exceeds the deadline, a fast queue-length fallback is used instead.

### N×N Mesh Network (`NxNMeshNetwork.h`)

Supports four simulation execution models:

| Mode | Description |
|---|---|
| **Sequential Centralized** | Each intersection updated one-by-one using the Webster controller |
| **Sequential Decentralized** | Each intersection updated one-by-one; neighbors exchange messages each tick |
| **Parallel Epoch** | All intersections update simultaneously in a synchronized epoch using `std::thread` |
| **Fully Asynchronous** | Each intersection runs on its own thread with lock-free mailbox messaging |

Thread safety is provided by `ThreadSafeIntersection` (mutex-wrapped) and `NodeMailbox` (atomic shared pointer for latest-message delivery).

### Message Buffer (`MessageBuffer.h`)

Implements **message batching** to reduce inter-node communication overhead:

- Groups multiple neighbor updates into a single `BatchedMessage` with a configurable batch interval.
- Includes a `SimpleCompressor` with **run-length encoding** and **delta encoding** for queue length arrays.
- Tracks batching statistics (total messages, batched messages, reduction ratio, average batch size).

### Real-Time Deadline Enforcer (`RealTimeDeadline.h`)

Wraps the controller's decision computation with wall-clock timing:

- Records per-decision latency using `std::chrono::high_resolution_clock`.
- Tracks total decisions, deadline misses, fallback invocations, max and average decision times.
- Configurable deadline in milliseconds (default: 50 ms).

---

## Key Features

- **Fully decentralized control** — no global coordinator; each node uses only local state + neighbor messages.
- **Multi-mode execution** — sequential, parallel epoch, and fully asynchronous simulation in one codebase.
- **Pedestrian phase support** — intersections accept pedestrian crossing requests with a configurable cooldown.
- **Emergency vehicle priority** — emergency vehicles are flagged at creation and can be extended for preemption logic.
- **Message batching & compression** — reduces simulated communication load with run-length and delta encoding.
- **Real-time deadline enforcement** — guarantees bounded decision latency with automatic fallback.
- **Level-of-service grading** — final report classifies intersection performance from A (Excellent) to F (Forced Flow).

---

## Simulation Modes

### Single Intersection (`TrafficSimulator.cpp`)

Simulates one named intersection (`IBA_Junction`) for 300 seconds under `RUSH_HOUR_NS` loading. Prints per-second state and a final performance report including average wait time, throughput (vehicles/min), max queue length, and LOS grade.

### N×N Mesh Network (`NxNMeshNetwork.h`)

Instantiate with:

```cpp
NxNMeshNetwork network(N, ScenarioType::RUSH_HOUR_NS);
network.step_decentralized(current_time, delta_time);   // sequential decentralized
network.step_centralized(current_time, delta_time);     // sequential centralized
network.step_parallel_epoch(current_time, delta_time);  // parallel epoch
network.step_async(...);                                // fully asynchronous
```

Collect aggregate statistics after the run:

```cpp
auto stats = network.collect_stats(simulation_duration_seconds);
// stats.total_processed, stats.avg_wait, stats.throughput_vpm
```

---

## Traffic Scenarios

| Scenario | Spawn Rate | Description |
|---|---|---|
| `NORMAL` | 0.30 / tick | Uniform random direction split |
| `RUSH_HOUR_NS` | 0.70 / tick | 80% of vehicles arrive from North/South |
| `RUSH_HOUR_EW` | 0.70 / tick | 80% of vehicles arrive from East/West |
| `PEAK_SWITCH` | 0.60 / tick | N-S dominant for first 100 s, then switches to E-W |
| `ACCIDENT` | 0.50 / tick | Asymmetric loading simulating partial blockage |
| `LOW_TRAFFIC` | 0.10 / tick | Sparse arrivals for off-peak testing |

Turn distribution per vehicle: 20% left, 50% straight, 30% right.

---

## Performance Metrics & Level of Service

The simulator reports the following at the end of each run:

| Metric | Formula |
|---|---|
| Vehicles Processed | Count of vehicles cleared through the intersection |
| Cumulative Wait Time | Sum of `(departure_time − arrival_time)` across all vehicles |
| Average Wait Time | Cumulative wait ÷ vehicles processed |
| System Throughput | Vehicles processed ÷ simulation minutes |
| Max Queue Length | Peak instantaneous queue across all directions |
| Intersection LOS | HCM-style grade based on average delay |

**Level of Service grades:**

| Grade | Avg Wait |
|---|---|
| A — Excellent | < 10 s |
| B — Very Good | 10 – 20 s |
| C — Good / Stable | 20 – 35 s |
| D — Fair / Unstable | 35 – 55 s |
| F — Forced Flow | > 55 s |

---

## Building & Running

### Requirements

- C++17 or later
- A POSIX-compliant system with `pthread` support (for parallel/async modes)
- No external libraries required

### Compile

```bash
# Single-intersection simulator
g++ -std=c++17 -O2 -pthread \
    TrafficSimulator.cpp TrafficGenerator.cpp CONTROLLER.cpp DecentralizedController.cpp \
    -o traffic_sim

# Message batching benchmark
g++ -std=c++17 -O2 -pthread \
    test_batching.cpp TrafficGenerator.cpp CONTROLLER.cpp DecentralizedController.cpp \
    -o test_batching

# Deadline enforcement smoke test
g++ -std=c++17 -O2 -pthread \
    test_deadline.cpp TrafficGenerator.cpp CONTROLLER.cpp DecentralizedController.cpp \
    -o test_deadline
```

### Run

```bash
./traffic_sim      # single intersection, 300 s RUSH_HOUR_NS
./test_batching    # batching benchmark across 3×3 mesh
./test_deadline    # deadline enforcement smoke test
```

Pre-compiled binaries (`traffic_sim`, `test_batching`, `test_deadline`) are included in the repository for convenience.

---

## Test Programs

### `test_batching.cpp`

Runs a 3×3 mesh for 60 seconds under `RUSH_HOUR_NS` three times:

1. No batching
2. Batching with a 5-second interval (~80% estimated message reduction)
3. Batching with a 10-second interval (~90% estimated message reduction)

Outputs a comparison table of vehicles processed, average wait, throughput, message reduction, and wall-clock simulation time.

### `test_deadline.cpp`

Exercises the `DecentralizedController` deadline path directly:

- Adds 10 vehicles to a test intersection.
- Runs 5 decision cycles with a 50 ms deadline enforced.
- Prints per-cycle phase decisions and full deadline statistics (total decisions, misses, fallback count, max and average decision time).

---

## Design Notes

- **Header-only core**: `Intersection.h`, `NxNMeshNetwork.h`, `MessageBuffer.h`, and `RealTimeDeadline.h` are fully self-contained headers, making integration into larger projects straightforward.
- **Fairness bias**: The decentralized controller tracks which axis was last preferred and applies a small penalty to prevent starvation of the minority direction.
- **Pedestrian cooldown**: The single-intersection simulator enforces a 30-second cooldown between pedestrian phases to model realistic push-button behavior.
- **Lock-free mailboxes**: The async mesh mode uses `std::atomic_store` / `std::atomic_load` on `shared_ptr` for latest-message delivery without contention on the hot path.
- **Extensibility**: `VehicleType::EMERGENCY` is modeled and priority-flagged; full preemption logic (jumping the queue, forcing a phase change) can be layered on top of the existing `set_phase` API.
