# CLAUDE.md — rmw_rtt_bench

## Project Overview

**rmw_rtt_bench** is a ROS 2 latency benchmarking suite that measures round-trip time (RTT) and one-way latency for pub/sub communication. It supports configurable RMW implementations (Fast DDS, CycloneDDS, Zenoh, etc.) and Quality of Service (QoS) settings.

The benchmark uses a ping-pong pattern with 4 timestamps (t0–t3) to compute RTT, ponger processing time, and estimated one-way latency. Results are written to CSV files with per-message detail and optional aggregate statistics.

## Repository Structure

```
rmw_rtt_bench/
├── CMakeLists.txt                          # ament_cmake build configuration (C++17)
├── package.xml                             # ROS 2 package manifest (format 3)
├── README.md                               # Usage guide (Japanese/English)
├── include/
│   └── rmw_rtt_bench/
│       └── latency_common.hpp              # Shared utilities: QoS builder, CLI helpers
├── src/
│   ├── latency_rtt.cpp                     # Unified pinger/ponger node (legacy, dual-role)
│   ├── latency_rtt_pinger.cpp              # Dedicated pinger node
│   ├── latency_rtt_ponger.cpp              # Dedicated ponger node
│   └── latency_stats.cpp                   # Statistics computation (mean, median, p95, p99)
├── msg/
│   └── Rtt.msg                             # Custom message: timestamps, seq, payload
├── launch/
│   ├── rtt_zenoh.launch.py                 # Combined pinger+ponger+optional Zenoh router
│   ├── rtt_zenoh_pinger.launch.py          # Pinger-only launch for Zenoh
│   └── rtt_zenoh_ponger.launch.py          # Ponger-only launch for Zenoh
└── scripts/
    ├── run_payload_sweep.py                # Automate testing across payload sizes
    └── summarize_rtt.py                    # Analyze CSV results and compute statistics
```

## Build & Run

### Prerequisites

- ROS 2 (tested with Jazzy and later)
- At least one RMW implementation installed (`rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, or `rmw_zenoh_cpp`)

### Build

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select rmw_rtt_bench --symlink-install
source install/setup.bash
```

### Run (basic example)

```bash
# Terminal 1: ponger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run rmw_rtt_bench rtt_ponger -- --duration 65

# Terminal 2: pinger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run rmw_rtt_bench rtt_pinger -- --hz 100 --payload-size 1024 --duration 60 --csv results/rtt.csv
```

### Run with Zenoh launch file

```bash
ros2 launch rmw_rtt_bench rtt_zenoh.launch.py start_router:=true hz:=100 payload_size:=1024 duration_pinger:=60
```

### Payload sweep

```bash
python3 scripts/run_payload_sweep.py 256:4096:512 --per-size-duration 10 --hz 100 --out results/sweep.csv
```

## Architecture

### Measurement Methodology

```
Pinger                    Ponger
  |                         |
  t0: publish request  -->  |
  |                     t1: receive request
  |                     t2: publish reply
  |  <-- t3: receive reply
  |
RTT = t3 - t0
Processing = t2 - t1
One-way estimate = (RTT - Processing) / 2
```

### Executables

| Target | Source | Purpose |
|--------|--------|---------|
| `ros2_latency_rtt` | `latency_rtt.cpp` + `latency_stats.cpp` | Legacy unified node with `--role pinger\|ponger` |
| `rtt_pinger` | `latency_rtt_pinger.cpp` + `latency_stats.cpp` | Dedicated pinger (preferred) |
| `rtt_ponger` | `latency_rtt_ponger.cpp` | Dedicated ponger (preferred) |

The dedicated pinger/ponger executables are the current approach. The unified `ros2_latency_rtt` binary is retained for backward compatibility.

### Message Definition (`msg/Rtt.msg`)

```
builtin_interfaces/Time t0_pub_send    # Pinger publish time
builtin_interfaces/Time t1_sub_recv    # Ponger receive time
builtin_interfaces/Time t2_sub_send    # Ponger reply time
uint32 seq                             # Sequence number
uint32 payload_size_bytes              # Payload size
uint8[] payload                        # Variable-length payload
```

### CSV Output Format

```
seq,t0_ns,t1_ns,t2_ns,t3_ns,rtt_ns,proc_ns,oneway_est_ns,payload_size,rmw,qos_rel,qos_hist,qos_depth,transport_tag,host,notes
```

## Key Code Conventions

### C++ Style

- **Standard:** C++17 (set in CMakeLists.txt)
- **Naming:** `snake_case` for functions/variables/files, `PascalCase` for classes/structs
- **Private members:** trailing underscore (`cfg_`, `pub_req_`, `seq_`)
- **Namespace:** `rmw_rtt_bench` (aliased as `rlb` within source files)
- **Header guards:** `#pragma once`
- **Include order:** STL headers → ROS 2 headers → local headers
- **Error handling:** Functions return `bool` with `std::string & error` out-parameter for parse/validation; `RCLCPP_ERROR`/`RCLCPP_WARN`/`RCLCPP_INFO` macros for logging

### Patterns

- Nodes derive from `rclcpp::Node`
- Periodic publishing via `create_wall_timer()` callbacks
- Subscription callbacks use `std::bind` with `std::placeholders`
- QoS configured through `QoSOptions` struct → `build_qos()` helper in `latency_common.hpp`
- CLI arguments parsed by custom `parse_*_args()` functions (not ROS 2 parameter server)
- CSV output via `std::ofstream` with periodic flush every 1000 lines
- Statistics module (`latency_stats.cpp`) exposes both C++ and C interfaces

### Python Scripts

- Python 3, using `subprocess` to orchestrate ROS 2 commands
- `argparse` for CLI argument handling
- CSV manipulation via Python `csv` module (no pandas dependency)

## Key Configuration Options

### Command-Line Arguments (pinger)

| Argument | Default | Description |
|----------|---------|-------------|
| `--hz` | 100.0 | Publishing frequency |
| `--payload-size` | 0 | Payload size in bytes |
| `--duration` | 30 | Run duration in seconds |
| `--trials` | -1 (unlimited) | Max round-trip count |
| `--timeout` | (none) | Safety timeout in seconds |
| `--csv` | results/rtt.csv | Output CSV path |
| `--req-topic` | /latency_rtt_req | Request topic name |
| `--rep-topic` | /latency_rtt_rep | Reply topic name |
| `--qos-reliability` | reliable | `reliable` or `best_effort` |
| `--qos-history` | keep_last | `keep_last` or `keep_all` |
| `--qos-depth` | 10 | QoS history depth |
| `--intra-process` | false | Enable intra-process comms |
| `--transport-tag` | (empty) | Metadata tag for CSV |
| `--summary` | false | Print stats on completion |
| `--append-summary` | false | Append stats to CSV |

### Environment Variables

- `RMW_IMPLEMENTATION` — selects the RMW backend (e.g., `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, `rmw_zenoh_cpp`)
- `ROS_DOMAIN_ID` — DDS domain isolation

## Dependencies

### Build-time
- `ament_cmake` — ROS 2 CMake build tool
- `rclcpp` — ROS 2 C++ client library
- `builtin_interfaces` — Standard ROS 2 time messages
- `rosidl_default_generators` — Message type code generation
- `rmw` — RMW abstraction layer

### Runtime
- `rclcpp`, `builtin_interfaces`, `rosidl_default_runtime`, `rmw`
- One or more RMW implementations (not declared as hard dependencies)

### Test
- `ament_lint_auto`, `ament_lint_common` (linting only; no automated test suite)

## Testing

There is no automated test suite. Testing is done manually by running pinger/ponger pairs and verifying CSV output. The `ament_lint_auto` / `ament_lint_common` test dependencies enable ROS 2 linting checks via:

```bash
colcon test --packages-select rmw_rtt_bench
```

## CI/CD

No CI/CD pipelines are currently configured.

## Important Notes for AI Assistants

- The project has no external C/C++ dependencies beyond the ROS 2 core stack.
- `latency_rtt.cpp` is the legacy unified node; prefer modifying `latency_rtt_pinger.cpp` and `latency_rtt_ponger.cpp` for new features.
- Message generation is handled by `rosidl_generate_interfaces()` in CMakeLists.txt. If the `Rtt.msg` fields change, all source files referencing `Rtt` fields need updating.
- The `latency_stats.cpp` module has a dual C/C++ interface — maintain both if modifying.
- Boolean CLI parameters accept multiple forms: `true`/`false`, `yes`/`no`, `on`/`off`, `1`/`0`, `True`/`False`, `TRUE`/`FALSE`.
- QoS durability is hardcoded to `Volatile` — not user-configurable.
- All timestamps are in nanoseconds internally; statistics are reported in milliseconds.
- Launch files are Python-based (ROS 2 launch system) and only target Zenoh currently.
- The `run_payload_sweep.py` script calls `rtt_pinger` via `ros2 run` subprocess calls — it requires the workspace to be sourced.
