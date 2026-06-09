# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

FPGA accelerator for 3D (x, y, theta) Value Iteration path planning, targeting Ultra96-V2 (Zynq UltraScale+ ZU3EG). Goal: solve a 14000×800×60 campus map in <60 s. The same VI algorithm is realized in several coordinated implementations that share a single 16-bit data contract, plus a faithful u64 reference model (`vi_rs/vi_reference`) mirroring the ROS1 original:

- **`vi_fpga/`** — the Vitis HLS kernel (`vi_sweep`), its Linux user-space driver (UIO + u-dma-buf), a C CLI, a host-side reference solver, and the Petalinux/board bring-up. This is the hardware vertical.
- **`vi_matlab/`** — a MATLAB HDL Coder variant of the streaming kernel plus algorithm experiments/benchmarks; mirrors the streaming HLS kernel.
- **`vi_rs/`** — a Rust Cargo workspace with two coordinated VI models: the 16-bit HLS data contract (`vi_core` + the `vi_algorithm` bitboard primitives) and a faithful u64 port of the ROS1 `value_iteration` node (`vi_reference`). The fast CPU solvers (reference, frontier, block, pyramid, stream-mimic) live in `vi_reference::solvers` — u64, bit-exact with the ROS1 original — and `vi_bench` is the benchmark harness.
- **`vi_ros2/`** — a ROS2 (Humble) Rust node (`vi_node`) that calls `vi_rs` (the u64 `vi_reference` solvers) to serve goals, builds the penalty field + goal mask inside `ValueIterator`, and publishes value/policy maps. rclrs is wired; the node builds via colcon in the Docker image.

Phase plan and design specs live in `docs/superpowers/specs/` and `docs/superpowers/plans/` — read them before making non-trivial changes to algorithms, datatypes, or memory layout. Specs are written in Japanese.

## Repository layout (important)

The C/HLS/driver/Petalinux code lives **under `vi_fpga/`**, not at the repo root. The root `Makefile` is a thin wrapper that delegates the software/FPGA/EDF targets into `vi_fpga/...` (and `matlab-*` / `rs-*` / `ros2-*` into their own trees), so the commands below are run from the repo root. Note: recursive `$(MAKE)` invocation fails under Windows GnuWin32 (`e=87`); run these on Linux/WSL (which is also where the Ultra96-targeted builds belong), or invoke the sub-Makefile directly (`make -C vi_fpga/host test-host`).

## Build & Test

### Software (driver + host CLI), under `vi_fpga/`

- `make driver` — build `libvi_sweep.a` / `.so` (UIO + u-dma-buf Linux ops + mock) via `vi_fpga/driver/uio/`.
- `make host` — build `vi_fpga/host/vi_cli` linked against the Linux libvi_sweep (depends on `driver`).
- `make test-host` — build the mock-only lib and run all host unit tests (`vi_fpga/host/test/test_*.c`). No FPGA needed.
- `make test-hw` — HW integration via SSH. Requires `VI_TARGET_HOST=<ultra96-hostname>`; runs `vi_fpga/host/test/hw/run_smoke.sh` then `run_big.sh`, which scp the CLI + generated maps to the target and execute `vi_cli --verify` there.
- Run a single host test: `make -C vi_fpga/host test/test_penalty.run` (pattern: `test/<name>.run`).
- Host-only CLI with the mock backend (no UIO needed, useful for local debugging): `make -C vi_fpga/host cli-mock` → `vi_fpga/host/vi_cli_mock`.

### FPGA build (`vi_fpga/Makefile`)

Tools must be on `PATH` — invoke bare `vitis-run` / `vivado` (Vitis 2025.2). Do **not** prefix with `source settings.sh`. Tile and streaming kernels have fully separate build paths. All TCL scripts live in `vi_fpga/tcl/`; build artifacts go to `vi_fpga/build/`. From the root wrapper, select the kernel with `KERNEL=tile` / `KERNEL=stream`; invoking `vi_fpga/Makefile` directly instead selects it via a `tile`/`stream` goal (`make -C vi_fpga csim stream`).

- `make csim KERNEL=stream` — HLS C-simulation of streaming kernel (`vi_fpga/hls/stream/`); `KERNEL=tile` for the tile kernel (`vi_fpga/hls/tile/`).
- `make hls KERNEL=tile` — HLS synth + IP export (tile) into `vi_fpga/build/hls_build_tile/`, IP to `ip_repo_tile/`; `KERNEL=stream` for streaming.
- `make bitstream KERNEL=tile` — HLS + Vivado synthesis + bitstream for tile kernel, project `vi_fpga/build/vi_tile/`; `KERNEL=stream` → `vi_fpga/build/vi_stream/`.
- `make clean-fpga` — clean both tile and stream build artifacts (`make -C vi_fpga clean`; append `tile`/`stream` to clean one).
- After regenerating HLS IP, sync the register header into the driver: `make sync-hw-header KERNEL=tile` or `KERNEL=stream` (copies `xvi_sweep_hw.h` / `xvi_sweep_stream_hw.h` into `vi_fpga/driver/uio/generated/`; review the diff).

### Rust workspace (`vi_rs/`)

A 5-crate Cargo workspace (`vi_core`, `vi_algorithm`, `vi_reference`, `vi_fixtures`, `vi_bench`) plus a standalone `vi_ml/` crate that is **deliberately not a workspace member** (use undefined; left untouched). `vi_fixtures` (synthetic u16 maps/transitions) is now orphaned — its only consumers were the u16 solvers, removed in the u64 migration. Driven from the repo-root Makefile (these targets are current):

- `make rs-test` — `cd vi_rs && cargo test --workspace`.
- `make rs-bench` — criterion microbenchmarks (`cargo bench -p vi_bench`) over the u64 `vi_reference` solvers (the `bitboard` microbench still exercises `vi_algorithm`).
- `make rs-bench-summary` — `bench_summary` CLI: a `benchmark_vi.m`-compatible macro comparison table across sizes/map-types over every u64 solver, emits CSV/markdown. Each solver is bit-exact with the ROS1 original (mismatch=0 vs the Reference oracle).
- `make rs-bench-parallel` — same; retained for `make`-target compatibility but a no-op (the u64 solvers are serial-only).
- Run a single crate's tests: `cd vi_rs && cargo test -p vi_reference`.

The u64 solvers in `vi_reference` are the bit-exact regression oracle; there is no parallel CPU path yet (the old `vi_algorithm` rayon `parallel` feature was dropped with the u16 solvers).

### MATLAB kernel (`vi_matlab/`)

Requires MATLAB R2024b+ with HDL Coder, HDL Verifier, Fixed-Point Designer, SoC Blockset. Driven from the repo root:

- `make matlab-sim` — run the MATLAB `matlab.unittest` suite (`run_matlab_tests.m`).
- `make matlab-hdl` — export packaged HDL IP for the repo Vivado flow (`export_repo_ip`).
- `make matlab-cosim` — HDL Verifier cosimulation via Xsim.
- `make matlab-bitstream` — Vivado bitstream from the exported MATLAB HDL IP.
- `make matlab-bench` / `matlab-bench-codegen` — MATLAB-native and MATLAB Coder C-generation benchmarks (`benchmark_vi`). Pass `REBUILD=1` to force a clean MEX rebuild.

The MATLAB kernel is a variant alongside tile and stream HLS kernels. Algorithm functions in `vi_matlab/src/` mirror the streaming HLS kernel (`vi_fpga/hls/stream/src/`). Constants in `vi_params.m` must stay synchronized with `vi_stream_types.h`.

### ROS2 node (`vi_ros2/`)

ROS2 Humble Rust node, built via `colcon` + `cargo-ament-build` inside a Docker image. Runs the u64 `vi_reference` solvers; interface-equivalent to the ROS1 node and builds/links via colcon (the tf2-based robot-pose lookup for `cmd_vel` is still a `(0,0,0)` stub). Driven from the repo root:

- `make ros2-docker` — build the dev image (`vi_ros2/docker/Dockerfile`), tag `vi_ros2_dev:humble` (override `VI_ROS2_DOCKER_IMG`).
- `make ros2-shell` — interactive shell in the image with the repo mounted at `/workspace`.
- `make ros2-build` / `make ros2-test` — run `scripts/ros2_build.sh` / `scripts/ros2_test.sh` in the container.

Two packages:

- `vi_interfaces/` — ament_cmake package defining `action/Vi.action` only; `rosidl_generator_rs` emits Rust types for rclrs.
- `vi_node/` — the rclrs node, built on the u64 `vi_reference::ValueIterator` + `solvers::solve`. **`vi_node` is deliberately outside the `vi_rs` Cargo workspace** (its `Cargo.toml` has an explicit empty `[workspace]`) so its `path = "../../vi_rs/*"` deps don't pull it into that workspace. `rclrs`/`nav_msgs`/`vi_interfaces` are wired as `*` deps and `[patch.crates-io]`-redirected (repo `.cargo/config.toml`) to colcon-built crates; the **binary links only via colcon** (`make ros2-build`), not plain `cargo build`.
- The rclrs-free library (`bridge`/`npy`/`solver_factory`/`sweep_thread` + the `oracle` equivalence tests) runs via `cargo test --lib` **inside the Docker image**; on the host it is checkable via a `/tmp` isolation crate that `#[path]`-includes those modules (the repo `.cargo/config.toml` ROS patches block a plain host build). A plain `cargo test --test ...` does NOT work — it forces cargo to build the rclrs binary, which only links under colcon (so the oracle tests live in the library, not `tests/`).

The external ROS interface is **interface-equivalent** to the ROS1 `value_iteration` catkin package (action name `vi_controller`, `/map` in, `value_function`/`policy`/`cmd_vel` out) but uses ROS2-native message types. See `docs/superpowers/specs/2026-05-29-vi-ros2-design.md`.

### EDF / Petalinux (`vi_fpga/petalinux/`)

Docker-based Yocto/EDF build for the Ultra96-V2 Linux image (delegates to `vi_fpga/petalinux/`):

- `make edf-docker` — build the Docker container for the EDF environment.
- `make edf-shell` — open an interactive shell in the container.
- `make edf-setup XSA=<path>` — initialize the EDF project from an XSA hardware description.
- `make edf-build MACHINE=<machine>` — run the full Yocto/EDF build.
- `make clean-edf` — clean EDF build artifacts.

## Architecture

The HLS hardware vertical (`vi_fpga/`) has four integrated layers sharing the same 16-bit data contract defined in `vi_fpga/hls/tile/src/vi_types.h` (tile) and `vi_fpga/hls/stream/src/vi_stream_types.h` (streaming). The MATLAB (`vi_matlab/`) and Rust (`vi_rs/vi_core`) ports replicate that same contract. Keep them all in sync.

Datatypes: `value_t`/`penalty_t` are `ap_uint<16>`; offsets `ap_int<8>`. Sentinels: `PENALTY_OBSTACLE = 0xFFFF` (impassable); `PENALTY_GOAL = 0xFFFE` — **when read as a neighbor's penalty it must be treated as 0** so the goal cell's value stays pinned at 0 (this convention is load-bearing; see the testbench, `vi_fpga/host/src/penalty.c`, and `vi_rs/vi_core/src/goal.rs`). Transition table is a packed `(dix, diy, dit)` word per `(action, theta)` — 6×60 = 360 entries, precomputed on ARM and DMA'd into the kernel.

### 1. HLS kernel (`vi_fpga/hls/tile/` and `vi_fpga/hls/stream/`)
Two kernel architectures share the data contract but differ in how they sweep the grid:

- **Tile kernel** (`vi_fpga/hls/tile/`): Dataflow pipeline `vi_sweep_top` = `load_tiles` → `compute_bellman` → `store_tiles`, processing 32×32 tiles with a 6-cell halo (TILE_W_H = 44). Two CUs are instantiated in the Vivado BD for red/black tile sweeping.
- **Streaming kernel** (`vi_fpga/hls/stream/`): Strip-based row streaming via `vi_sweep_stream`. Processes horizontal strips using 13-row line buffers (`WINDOW_ROWS = 2*HALO_MAX+1`). Pipeline: `load_store_row` feeds rows → `stream_strip` manages the line buffer → `compute_row` does per-cell Bellman updates. Two CUs split the map vertically.

### 2. Device layer (`vi_fpga/driver/uio/`)
`vi_device.h` defines a `vi_device_ops_t` vtable (init/shutdown/read_reg/write_reg/wait_irq/map_buf) with two implementations:
- `vi_device_linux.c` — real UIO + u-dma-buf (requires the device-tree overlay in `vi_fpga/driver/dts/vi_sweep.dtsi` applied via Petalinux on the target).
- `vi_device_mock.c` — in-memory software model used for host unit tests and `cli-mock`.

`libvi_sweep.c` sits on top of the vtable and exposes the public API (`libvi_sweep.h`). Build flavors:
- `libvi_sweep.a` / `.so` — full build, both backends.
- `libvi_sweep_mock.a` — built with `-DVI_MOCK_ONLY`, no Linux ops; used by `test-host` and `cli-mock`. Any code touching `vi_linux_ops` must be guarded by `#ifndef VI_MOCK_ONLY`.

Register offsets come from the HLS-generated `xvi_sweep_hw.h`; never hand-edit `vi_fpga/driver/uio/generated/xvi_sweep_hw.h` — regenerate via `sync-hw-header` after an HLS rebuild.

### 3. Host CLI + reference (`vi_fpga/host/`)
`vi_cli.c` loads a PGM map + YAML metadata (`map_pgm.c`), builds the penalty field (`penalty.c`), computes the transition table (`transitions.c`), opens the vi_sweep device, runs sweeps, and optionally verifies against `vi_reference_c.c` (pure-C value iteration matching the HLS testbench reference). `--verify` asserts bit-exact equality vs the reference; this is the oracle for HW correctness. Unit tests in `vi_fpga/host/test/` exercise each module and a full mock-backed run (`test_vi_run_mock.c`, `test_reference_eq.c`).

### 4. FPGA/board bring-up (`vi_fpga/tcl/`, `vi_fpga/vivado/`, `vi_fpga/pynq/`)
`create_bd_*.tcl` / `create_project_*.tcl` (in `vi_fpga/tcl/`) build the Vivado block design wrapping two `vi_sweep` CUs with AXI and interrupts. `vi_fpga/pynq/` holds bitstream + hwh + a PYNQ-side overlay helper for pre-Linux-driver experimentation on Ultra96-V2.

### 5. Rust algorithm port (`vi_rs/`)
- `vi_core` — the immutable 16-bit data contract: `types`, algorithm constants (`params`: `PENALTY_OBSTACLE`/`PENALTY_GOAL`/`STEP_COST`/`N_THETA`/…), the bit-exact `cost_of`, packed↔unpacked transition conversion (`transitions`), and `make_goal_mask` (`goal`). Mirrors the HLS/C/MATLAB contract.
- `vi_reference` — a faithful u64 port of the ROS1 `value_iteration` node (`ValueIterator` / `State` / `Action`, PROB_BASE = 2^18 fixed point), reproducing its quirks (including the original's int-division and margin-penalty bugs). `solvers::solve(&mut ValueIterator, U64Solver, max_iter)` dispatches the 10 fast solvers — `Reference`, `Frontier2D/3D{,Tau,TopK,CoarseTheta}`, `FrontierStack`, `BlockRefine`, `PyramidSweep`, `StreamMimic` — each of which applies the original per-cell Bellman update over an active set, so the converged value of reachable cells is bit-exact with the ROS1 original (proven by the `solvers::test_support` parity tests). This is the active CPU model. Design: `docs/superpowers/specs/2026-06-08-vi-reference-faithful-port-design.md`, `2026-06-09-vi-u64-fast-solvers-design.md`.
- `vi_algorithm` — now just the value-type-agnostic `bitboard` primitives (3-D θ-periodic dilation, 2-D AND/OR, enumerate, ndarray conv) that `vi_reference`'s frontier solvers reuse. The u16 `Solver`/`VIContext` family that used to live here was ported to `vi_reference` and removed.
- `vi_fixtures` — `gen_test_map` / `gen_transitions` equivalents (u16). Orphaned after the u64 migration (its consumers were the removed u16 solvers); kept in the workspace but unused.
- `vi_bench` — criterion benches + the `bench_summary` / `bench_map` CLIs, all over the u64 `vi_reference` solvers; the `bitboard` microbench still exercises `vi_algorithm`.

## Conventions

- C code: `-std=c11 -Wall -Wextra -Werror`. Keep new code warning-clean or the build breaks.
- When changing the HLS data contract (types, tile size, sentinels, transition packing), update **in lockstep**: `vi_fpga/hls/tile/src/vi_types.h`, `vi_fpga/hls/stream/src/vi_stream_types.h`, `vi_fpga/host/src/vi_reference_c.c`, `vi_fpga/host/src/penalty.c`/`transitions.c`, the mock device, `vi_matlab/.../vi_params.m`, and `vi_rs/vi_core` (`params.rs`/`goal.rs`/`transitions.rs`). Then re-run `make -C vi_fpga/host test-host` and `make rs-test`.
- Goal-cell handling: the `PENALTY_GOAL` → 0 substitution when read as a neighbor's penalty is required across all implementations — do not "simplify" it away.
- HW tests are SSH-driven. They assume the target already has the bitstream loaded and the `vi_sweep` overlay applied; they do not program the FPGA themselves.
