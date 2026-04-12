# Line-Buffer Streaming VI Kernel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the tile-based vi_sweep HLS kernel with a line-buffer streaming architecture that achieves ~100× faster convergence by enabling full-map Gauss-Seidel propagation within each sweep.

**Architecture:** Two CUs (forward + reverse scan) process the map via sliding-window line buffers, streaming row-by-row through X-direction strips. Both CUs share DDR (lock-free, like Ueda et al.'s multi-threaded CPU approach). HP0/HP1 port separation eliminates DDR bandwidth contention.

**Tech Stack:** Vitis HLS 2025.2, Vivado 2025.2, PYNQ on Ultra96-V2 (ZU3EG)

---

## File Map

### New files (HLS kernel)

| File | Responsibility |
|------|---------------|
| `fpga/hls/vi_sweep_stream/src/vi_stream_types.h` | Constants (HALO_MAX, STRIP_W_MAX, WINDOW_ROWS) + type aliases |
| `fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.h` | Top-level function declaration |
| `fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.cpp` | Top-level: load transitions, iterate strips, report max_delta |
| `fpga/hls/vi_sweep_stream/src/load_store_row.h` | Row load/store function declarations |
| `fpga/hls/vi_sweep_stream/src/load_store_row.cpp` | DDR burst read/write of one row (value + penalty) with halo + OOB fill |
| `fpga/hls/vi_sweep_stream/src/compute_row.h` | Bellman row update declaration |
| `fpga/hls/vi_sweep_stream/src/compute_row.cpp` | Pipelined Bellman update for one row: LOOP_X × LOOP_T at II=1 |
| `fpga/hls/vi_sweep_stream/src/stream_strip.h` | Strip streaming declaration |
| `fpga/hls/vi_sweep_stream/src/stream_strip.cpp` | Sliding window management: init window, row loop, evict/load |
| `fpga/hls/vi_sweep_stream/hls_config.cfg` | Vitis HLS project config |

### New files (testbench)

| File | Responsibility |
|------|---------------|
| `fpga/hls/vi_sweep_stream/tb/vi_sweep_stream_tb.cpp` | C-sim testbench: 20×20 basic + 40×20 strip-boundary test |
| `fpga/hls/vi_sweep_stream/tb/vi_reference.h` | Copy from `vi_sweep/tb/vi_reference.h` (unchanged) |
| `fpga/hls/vi_sweep_stream/tb/vi_reference.cpp` | Copy from `vi_sweep/tb/vi_reference.cpp` (unchanged) |

### Modified files

| File | Change |
|------|--------|
| `fpga/scripts/Makefile` | Add `csim_stream`, `hls_stream` targets |
| `fpga/scripts/run_csim_stream.tcl` | New: csim for vi_sweep_stream |
| `fpga/scripts/export_hls_ip_stream.tcl` | New: synthesis + IP export for vi_sweep_stream |
| `fpga/vivado/ultra96v2/create_bd.tcl` | Replace vi_sweep IPs with vi_sweep_stream, add HP1, split data paths |
| `fpga/pynq/vi_overlay.py` | Update register offsets, IP names |
| `fpga/pynq/explore_map.ipynb` | Remove tile padding, update sweep loop |

---

### Task 1: Project scaffolding and types header

**Files:**
- Create: `fpga/hls/vi_sweep_stream/src/vi_stream_types.h`
- Create: `fpga/hls/vi_sweep_stream/hls_config.cfg`
- Create: `fpga/hls/vi_sweep_stream/tb/vi_reference.h` (copy)
- Create: `fpga/hls/vi_sweep_stream/tb/vi_reference.cpp` (copy)

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p fpga/hls/vi_sweep_stream/src
mkdir -p fpga/hls/vi_sweep_stream/tb
```

- [ ] **Step 2: Create vi_stream_types.h**

```cpp
// fpga/hls/vi_sweep_stream/src/vi_stream_types.h
#pragma once

#include <ap_int.h>

// --- Data types (same as tile-based kernel) ---
typedef ap_uint<16> value_t;
typedef ap_uint<16> penalty_t;
typedef ap_int<8>   offset_t;

// --- Constants ---
constexpr int N_ACTIONS   = 6;
constexpr int N_THETA     = 60;
constexpr int HALO_MAX    = 6;     // max |dix|, |diy| at 0.05m
constexpr int WINDOW_ROWS = 2 * HALO_MAX + 1;  // 13
constexpr int STRIP_W_MAX = 256;   // BRAM-limited per CU
constexpr int BUF_W       = STRIP_W_MAX + 2 * HALO_MAX;  // 268

constexpr int TRANS_TABLE_SIZE = N_ACTIONS * N_THETA;  // 360

// Sentinel values
const value_t   MAX_VALUE         = 0xFFFF;
const penalty_t PENALTY_OBSTACLE  = 0xFFFF;
const penalty_t PENALTY_GOAL      = 0xFFFE;
```

- [ ] **Step 3: Create hls_config.cfg**

```ini
part=xczu3eg-sbva484-1-i

[hls]
flow_target=vivado
package.output.format=ip_catalog
package.output.syn=false
sim.O=1
syn.top=vi_sweep_stream
syn.file=src/vi_sweep_stream_top.cpp
syn.file=src/stream_strip.cpp
syn.file=src/compute_row.cpp
syn.file=src/load_store_row.cpp
tb.file=tb/vi_sweep_stream_tb.cpp
tb.file=tb/vi_reference.cpp
```

- [ ] **Step 4: Copy CPU reference files**

```bash
cp fpga/hls/vi_sweep/tb/vi_reference.h fpga/hls/vi_sweep_stream/tb/
cp fpga/hls/vi_sweep/tb/vi_reference.cpp fpga/hls/vi_sweep_stream/tb/
```

- [ ] **Step 5: Commit**

```bash
git add fpga/hls/vi_sweep_stream/
git commit -m "feat(hls): scaffold vi_sweep_stream project with types and config"
```

---

### Task 2: Row load/store functions

**Files:**
- Create: `fpga/hls/vi_sweep_stream/src/load_store_row.h`
- Create: `fpga/hls/vi_sweep_stream/src/load_store_row.cpp`

- [ ] **Step 1: Create load_store_row.h**

```cpp
// fpga/hls/vi_sweep_stream/src/load_store_row.h
#pragma once

#include "vi_stream_types.h"

// Load one row (with halo) from DDR into the given window slot.
// If gy is out of bounds, fills with MAX_VALUE / PENALTY_OBSTACLE.
void load_row(
    value_t   val_row[BUF_W][N_THETA],
    penalty_t pen_row_0[BUF_W],
    penalty_t pen_row_1[BUF_W],
    penalty_t pen_row_2[BUF_W],
    const value_t   *value_table,
    const penalty_t *penalty_table,
    int gy,               // global Y coordinate of this row
    int strip_x0,         // global X start of strip (before halo)
    int strip_w,          // inner strip width (cells, <= STRIP_W_MAX)
    int map_x, int map_y);

// Store one row (inner cells only, no halo) back to DDR.
void store_row(
    const value_t val_row[BUF_W][N_THETA],
    value_t *value_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x);
```

- [ ] **Step 2: Create load_store_row.cpp**

```cpp
// fpga/hls/vi_sweep_stream/src/load_store_row.cpp
#include "load_store_row.h"

void load_row(
    value_t   val_row[BUF_W][N_THETA],
    penalty_t pen_row_0[BUF_W],
    penalty_t pen_row_1[BUF_W],
    penalty_t pen_row_2[BUF_W],
    const value_t   *value_table,
    const penalty_t *penalty_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x, int map_y)
{
    int buf_w = strip_w + 2 * HALO_MAX;
    int gx_start = strip_x0 - HALO_MAX;
    bool y_oob = (gy < 0 || gy >= map_y);

    // Phase A: Fill entire row with sentinels
    FILL_PEN: for (int lx = 0; lx < BUF_W; lx++) {
        #pragma HLS PIPELINE II=1
        penalty_t p = PENALTY_OBSTACLE;
        pen_row_0[lx] = p;
        pen_row_1[lx] = p;
        pen_row_2[lx] = p;
    }
    FILL_VAL_X: for (int lx = 0; lx < BUF_W; lx++) {
        FILL_VAL_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            val_row[lx][it] = MAX_VALUE;
        }
    }

    if (y_oob) return;

    // Phase B: Compute in-bounds X range
    int x0_global = (gx_start < 0) ? 0 : gx_start;
    int x1_global = (gx_start + buf_w > map_x) ? map_x : (gx_start + buf_w);
    int x_count = x1_global - x0_global;
    int lx_offset = x0_global - gx_start;

    if (x_count <= 0) return;

    // Phase C: Burst-read penalty
    int pen_base = gy * map_x + x0_global;
    LOAD_PEN: for (int i = 0; i < x_count; i++) {
        #pragma HLS PIPELINE II=1
        penalty_t p = penalty_table[pen_base + i];
        pen_row_0[lx_offset + i] = p;
        pen_row_1[lx_offset + i] = p;
        pen_row_2[lx_offset + i] = p;
    }

    // Phase D: Burst-read value (contiguous: x * N_THETA)
    int val_base = (gy * map_x + x0_global) * N_THETA;
    LOAD_VAL_X: for (int i = 0; i < x_count; i++) {
        LOAD_VAL_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            val_row[lx_offset + i][it] = value_table[val_base + i * N_THETA + it];
        }
    }
}

void store_row(
    const value_t val_row[BUF_W][N_THETA],
    value_t *value_table,
    int gy,
    int strip_x0,
    int strip_w,
    int map_x)
{
    int val_base = (gy * map_x + strip_x0) * N_THETA;

    STORE_X: for (int ix = 0; ix < strip_w; ix++) {
        int bx = ix + HALO_MAX;
        STORE_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            value_table[val_base + ix * N_THETA + it] = val_row[bx][it];
        }
    }
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep_stream/src/load_store_row.*
git commit -m "feat(hls): add row load/store with halo and OOB fill"
```

---

### Task 3: Bellman row update core

**Files:**
- Create: `fpga/hls/vi_sweep_stream/src/compute_row.h`
- Create: `fpga/hls/vi_sweep_stream/src/compute_row.cpp`

- [ ] **Step 1: Create compute_row.h**

```cpp
// fpga/hls/vi_sweep_stream/src/compute_row.h
#pragma once

#include "vi_stream_types.h"

// Perform Bellman update for one row in the window.
// val_buf is the full sliding window [WINDOW_ROWS][BUF_W][N_THETA].
// win_center is the circular buffer index of the current row.
// cu_id controls X scan direction: 0=forward (ix++), 1=reverse (ix--).
void compute_row(
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA],
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W],
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int win_center,
    int strip_w,
    int cu_id,
    value_t &row_max_delta);
```

- [ ] **Step 2: Create compute_row.cpp**

This is the critical path. BRAM port scheduling mirrors the tile-based kernel exactly:
- Actions 0,1 (forward/backward): theta bank `it` → val_buf port 0,1 + pen_buf_0
- Actions 2,4 (left/fwd-left): theta bank `(it+3)%60` → val_buf port 0,1 + pen_buf_1
- Actions 3,5 (right/fwd-right): theta bank `(it-3+60)%60` → val_buf port 0,1 + pen_buf_2

```cpp
// fpga/hls/vi_sweep_stream/src/compute_row.cpp
#include "compute_row.h"

static inline value_t cost_of(value_t nv, penalty_t np_raw)
{
    if (nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE) return MAX_VALUE;
    penalty_t np = (np_raw == PENALTY_GOAL) ? (penalty_t)0 : np_raw;
    ap_uint<17> sum = (ap_uint<17>)nv + (ap_uint<17>)np;
    return (sum >= MAX_VALUE) ? (value_t)(MAX_VALUE - 1) : (value_t)sum;
}

void compute_row(
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA],
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W],
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W],
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int win_center,
    int strip_w,
    int cu_id,
    value_t &row_max_delta)
{
    #pragma HLS INLINE off
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0

    value_t local_max = 0;

    LOOP_X: for (int ix_raw = 0; ix_raw < STRIP_W_MAX; ix_raw++) {
        #pragma HLS LOOP_TRIPCOUNT min=1 max=256
        if (ix_raw >= strip_w) break;

        int ix = (cu_id == 0) ? ix_raw : (strip_w - 1 - ix_raw);
        int bx = ix + HALO_MAX;

        penalty_t cell_pen = pen_buf_0[win_center][bx];
        bool skip = (cell_pen >= PENALTY_GOAL);

        LOOP_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS DEPENDENCE variable=val_buf type=inter false

            value_t old_val = val_buf[win_center][bx][it];

            // --- 6 actions, BRAM-port-scheduled ---
            // Action 0: forward
            int ny0 = (win_center + (int)delta_table[0][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx0 = bx + (int)delta_table[0][it][0];
            int it_fw = it;
            value_t c0 = cost_of(val_buf[ny0][nx0][it_fw], pen_buf_0[ny0][nx0]);

            // Action 1: backward
            int ny1 = (win_center + (int)delta_table[1][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx1 = bx + (int)delta_table[1][it][0];
            value_t c1 = cost_of(val_buf[ny1][nx1][it_fw], pen_buf_0[ny1][nx1]);

            // Action 2: left
            int ny2 = (win_center + (int)delta_table[2][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx2 = bx + (int)delta_table[2][it][0];
            int it_l = it + (int)delta_table[2][it][2];
            it_l = (it_l < 0) ? it_l + N_THETA : (it_l >= N_THETA) ? it_l - N_THETA : it_l;
            value_t c2 = cost_of(val_buf[ny2][nx2][it_l], pen_buf_1[ny2][nx2]);

            // Action 3: right
            int ny3 = (win_center + (int)delta_table[3][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx3 = bx + (int)delta_table[3][it][0];
            int it_r = it + (int)delta_table[3][it][2];
            it_r = (it_r < 0) ? it_r + N_THETA : (it_r >= N_THETA) ? it_r - N_THETA : it_r;
            value_t c3 = cost_of(val_buf[ny3][nx3][it_r], pen_buf_1[ny3][nx3]);

            // Action 4: forward-left
            int ny4 = (win_center + (int)delta_table[4][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx4 = bx + (int)delta_table[4][it][0];
            value_t c4 = cost_of(val_buf[ny4][nx4][it_l], pen_buf_2[ny4][nx4]);

            // Action 5: forward-right
            int ny5 = (win_center + (int)delta_table[5][it][1] + WINDOW_ROWS) % WINDOW_ROWS;
            int nx5 = bx + (int)delta_table[5][it][0];
            value_t c5 = cost_of(val_buf[ny5][nx5][it_r], pen_buf_2[ny5][nx5]);

            // Min-reduction tree
            value_t min01 = (c0 < c1) ? c0 : c1;
            value_t min23 = (c2 < c3) ? c2 : c3;
            value_t min45 = (c4 < c5) ? c4 : c5;
            value_t min03 = (min01 < min23) ? min01 : min23;
            value_t min_cost = (min03 < min45) ? min03 : min45;

            // Gauss-Seidel in-place update
            value_t new_val = skip ? old_val : min_cost;
            val_buf[win_center][bx][it] = new_val;

            value_t d = (new_val > old_val) ? (value_t)(new_val - old_val)
                                            : (value_t)(old_val - new_val);
            value_t masked_d = skip ? (value_t)0 : d;
            if (masked_d > local_max) local_max = masked_d;
        }
    }

    row_max_delta = local_max;
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep_stream/src/compute_row.*
git commit -m "feat(hls): add pipelined Bellman row update at II=1"
```

---

### Task 4: Strip streaming function

**Files:**
- Create: `fpga/hls/vi_sweep_stream/src/stream_strip.h`
- Create: `fpga/hls/vi_sweep_stream/src/stream_strip.cpp`

- [ ] **Step 1: Create stream_strip.h**

```cpp
// fpga/hls/vi_sweep_stream/src/stream_strip.h
#pragma once

#include "vi_stream_types.h"

// Stream one X-strip through the sliding window.
// cu_id: 0=forward (Y ascending), 1=reverse (Y descending).
void stream_strip(
    value_t   *value_table,
    const penalty_t *penalty_table,
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int map_x, int map_y,
    int strip_x0,
    int strip_w,
    int cu_id,
    value_t &strip_max_delta);
```

- [ ] **Step 2: Create stream_strip.cpp**

```cpp
// fpga/hls/vi_sweep_stream/src/stream_strip.cpp
#include "stream_strip.h"
#include "load_store_row.h"
#include "compute_row.h"

void stream_strip(
    value_t   *value_table,
    const penalty_t *penalty_table,
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int map_x, int map_y,
    int strip_x0,
    int strip_w,
    int cu_id,
    value_t &strip_max_delta)
{
    #pragma HLS INLINE off

    // --- Line buffer declaration (BRAM) ---
    value_t   val_buf[WINDOW_ROWS][BUF_W][N_THETA];
    penalty_t pen_buf_0[WINDOW_ROWS][BUF_W];
    penalty_t pen_buf_1[WINDOW_ROWS][BUF_W];
    penalty_t pen_buf_2[WINDOW_ROWS][BUF_W];

    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_t2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    value_t local_max = 0;

    // --- Initialize window: load WINDOW_ROWS rows around the first data row ---
    INIT_WIN: for (int wr = 0; wr < WINDOW_ROWS; wr++) {
        // For forward (cu_id=0): first data row is iy=0, window covers [-HALO..+HALO]
        // For reverse (cu_id=1): first data row is iy=map_y-1, window covers [map_y-1-HALO..map_y-1+HALO]
        int gy;
        if (cu_id == 0)
            gy = -HALO_MAX + wr;
        else
            gy = (map_y - 1) + HALO_MAX - wr;

        load_row(val_buf[wr], pen_buf_0[wr], pen_buf_1[wr], pen_buf_2[wr],
                 value_table, penalty_table,
                 gy, strip_x0, strip_w, map_x, map_y);
    }

    // --- Stream through all rows ---
    ROW_LOOP: for (int iy_raw = 0; iy_raw < map_y; iy_raw++) {
        #pragma HLS LOOP_TRIPCOUNT min=20 max=800
        int iy = (cu_id == 0) ? iy_raw : (map_y - 1 - iy_raw);
        int win_center = iy_raw % WINDOW_ROWS;

        // Compute Bellman update for current row
        value_t row_delta;
        compute_row(val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                    delta_table, win_center, strip_w, cu_id, row_delta);
        if (row_delta > local_max) local_max = row_delta;

        // Store updated row to DDR
        store_row(val_buf[win_center], value_table,
                  iy, strip_x0, strip_w, map_x);

        // Evict oldest row, load next future row
        if (iy_raw + HALO_MAX + 1 < map_y + HALO_MAX) {
            int next_gy;
            if (cu_id == 0)
                next_gy = iy_raw + HALO_MAX + 1;
            else
                next_gy = (map_y - 1) - (iy_raw + HALO_MAX + 1);

            int evict_slot = (iy_raw + HALO_MAX + 1) % WINDOW_ROWS;
            load_row(val_buf[evict_slot],
                     pen_buf_0[evict_slot], pen_buf_1[evict_slot], pen_buf_2[evict_slot],
                     value_table, penalty_table,
                     next_gy, strip_x0, strip_w, map_x, map_y);
        }
    }

    strip_max_delta = local_max;
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep_stream/src/stream_strip.*
git commit -m "feat(hls): add sliding-window strip streaming with circular buffer"
```

---

### Task 5: Top-level kernel

**Files:**
- Create: `fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.h`
- Create: `fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.cpp`

- [ ] **Step 1: Create vi_sweep_stream_top.h**

```cpp
// fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.h
#pragma once

#include "vi_stream_types.h"

extern "C" void vi_sweep_stream(
    value_t       *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int cu_id,
    value_t *max_delta);
```

- [ ] **Step 2: Create vi_sweep_stream_top.cpp**

```cpp
// fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.cpp
#include "vi_sweep_stream_top.h"
#include "stream_strip.h"

static void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    #pragma HLS INLINE off
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7,  0));
        delta_table[a][t][1] = (offset_t)(w(15, 8));
        delta_table[a][t][2] = (offset_t)(w(23, 16));
    }
}

extern "C" void vi_sweep_stream(
    value_t       *value_table,
    const penalty_t *penalty_table,
    const ap_uint<32> *trans_table,
    int map_x,
    int map_y,
    int cu_id,
    value_t *max_delta)
{
    // --- AXI interface pragmas ---
    #pragma HLS INTERFACE m_axi port=value_table   bundle=gmem0 depth=672000000
    #pragma HLS INTERFACE m_axi port=penalty_table  bundle=gmem1 depth=11200000
    #pragma HLS INTERFACE m_axi port=trans_table    bundle=gmem1 depth=360
    #pragma HLS INTERFACE s_axilite port=value_table
    #pragma HLS INTERFACE s_axilite port=penalty_table
    #pragma HLS INTERFACE s_axilite port=trans_table
    #pragma HLS INTERFACE s_axilite port=map_x
    #pragma HLS INTERFACE s_axilite port=map_y
    #pragma HLS INTERFACE s_axilite port=cu_id
    #pragma HLS INTERFACE s_axilite port=max_delta
    #pragma HLS INTERFACE s_axilite port=return

    // 1. Load transition table into registers
    offset_t delta_table[N_ACTIONS][N_THETA][3];
    #pragma HLS ARRAY_PARTITION variable=delta_table complete dim=0
    load_transitions(trans_table, delta_table);

    // 2. Compute strip layout
    int num_strips = (map_x + STRIP_W_MAX - 1) / STRIP_W_MAX;

    value_t global_max_delta = 0;

    // 3. Iterate X-strips
    STRIP_LOOP: for (int sx_raw = 0; sx_raw < num_strips; sx_raw++) {
        #pragma HLS LOOP_TRIPCOUNT min=1 max=56
        // Forward CU: left-to-right strips; Reverse CU: right-to-left
        int sx = (cu_id == 0) ? sx_raw : (num_strips - 1 - sx_raw);
        int strip_x0 = sx * STRIP_W_MAX;
        int strip_w  = ((strip_x0 + STRIP_W_MAX) > map_x)
                       ? (map_x - strip_x0) : STRIP_W_MAX;

        value_t strip_delta;
        stream_strip(value_table, penalty_table, delta_table,
                     map_x, map_y, strip_x0, strip_w,
                     cu_id, strip_delta);

        if (strip_delta > global_max_delta)
            global_max_delta = strip_delta;
    }

    *max_delta = global_max_delta;
}
```

- [ ] **Step 3: Commit**

```bash
git add fpga/hls/vi_sweep_stream/src/vi_sweep_stream_top.*
git commit -m "feat(hls): add vi_sweep_stream top-level kernel with strip iteration"
```

---

### Task 6: Testbench

**Files:**
- Create: `fpga/hls/vi_sweep_stream/tb/vi_sweep_stream_tb.cpp`

- [ ] **Step 1: Create testbench**

Two test cases: (A) 20×20 basic (fits in 1 strip), (B) 40×20 (requires 2 strips for STRIP_W_MAX=256... actually 40<256, so we need a larger map to test strip boundaries. Use MAP_X=300 to force 2 strips).

```cpp
// fpga/hls/vi_sweep_stream/tb/vi_sweep_stream_tb.cpp
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "../src/vi_sweep_stream_top.h"
#include "vi_reference.h"

static void build_test_map(
    uint16_t *penalty, uint16_t *value,
    int map_x, int map_y, int goal_x, int goal_y)
{
    int map_size = map_x * map_y;
    int state_size = map_size * vi_ref::N_THETA;

    for (int i = 0; i < map_size; i++) penalty[i] = 0;

    // Border obstacles
    for (int x = 0; x < map_x; x++) {
        penalty[0 * map_x + x] = vi_ref::PENALTY_OBSTACLE;
        penalty[(map_y - 1) * map_x + x] = vi_ref::PENALTY_OBSTACLE;
    }
    for (int y = 0; y < map_y; y++) {
        penalty[y * map_x + 0] = vi_ref::PENALTY_OBSTACLE;
        penalty[y * map_x + (map_x - 1)] = vi_ref::PENALTY_OBSTACLE;
    }

    // L-shaped obstacle
    for (int x = 5; x <= std::min(12, map_x - 2); x++)
        penalty[10 * map_x + x] = vi_ref::PENALTY_OBSTACLE;
    for (int y = 6; y <= 10; y++)
        if (12 < map_x)
            penalty[y * map_x + 12] = vi_ref::PENALTY_OBSTACLE;

    // Safety penalty near obstacles
    for (int y = 1; y < map_y - 1; y++)
        for (int x = 1; x < map_x - 1; x++) {
            if (penalty[y * map_x + x] == vi_ref::PENALTY_OBSTACLE) continue;
            bool near = false;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                    if (penalty[(y+dy)*map_x + (x+dx)] == vi_ref::PENALTY_OBSTACLE)
                        near = true;
            if (near) penalty[y * map_x + x] = 100;
        }

    // Goal
    penalty[goal_y * map_x + goal_x] = vi_ref::PENALTY_GOAL;

    // Value init
    for (int i = 0; i < state_size; i++) value[i] = vi_ref::MAX_VALUE;
    for (int it = 0; it < vi_ref::N_THETA; it++)
        value[(goal_y * map_x + goal_x) * vi_ref::N_THETA + it] = 0;
}

static void pack_transitions(
    const vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA],
    uint32_t *packed)
{
    for (int a = 0; a < vi_ref::N_ACTIONS; a++)
        for (int it = 0; it < vi_ref::N_THETA; it++) {
            uint32_t w = ((uint32_t)(uint8_t)trans[a][it].dix)
                       | ((uint32_t)(uint8_t)trans[a][it].diy << 8)
                       | ((uint32_t)(uint8_t)trans[a][it].dit << 16);
            packed[a * vi_ref::N_THETA + it] = w;
        }
}

static int run_test(const char *name, int map_x, int map_y,
                    int goal_x, int goal_y, double xy_res)
{
    printf("\n=== Test: %s (%dx%d, goal=(%d,%d), res=%.3f) ===\n",
           name, map_x, map_y, goal_x, goal_y, xy_res);

    int map_size   = map_x * map_y;
    int state_size = map_size * vi_ref::N_THETA;

    uint16_t *pen_ref = new uint16_t[map_size];
    uint16_t *val_ref = new uint16_t[state_size];
    uint16_t *pen_hls = new uint16_t[map_size];
    uint16_t *val_hls = new uint16_t[state_size];

    build_test_map(pen_ref, val_ref, map_x, map_y, goal_x, goal_y);
    memcpy(pen_hls, pen_ref, map_size * sizeof(uint16_t));
    memcpy(val_hls, val_ref, state_size * sizeof(uint16_t));

    // Transitions
    vi_ref::TransitionEntry trans[vi_ref::N_ACTIONS][vi_ref::N_THETA];
    vi_ref::compute_transitions(xy_res, trans);
    uint32_t trans_packed[vi_ref::N_ACTIONS * vi_ref::N_THETA];
    pack_transitions(trans, trans_packed);

    // CPU reference
    int ref_sweeps = vi_ref::run_vi(val_ref, pen_ref, trans,
                                     map_x, map_y, 0, 200);
    printf("  CPU reference: %d sweeps\n", ref_sweeps);

    // HLS kernel — alternate forward/reverse
    value_t hls_max_delta;
    int hls_sweeps = 0;
    for (int s = 0; s < 200; s++) {
        int cid = s % 2;  // alternate forward/reverse
        vi_sweep_stream(
            (value_t *)val_hls,
            (const penalty_t *)pen_hls,
            (const ap_uint<32> *)trans_packed,
            map_x, map_y,
            cid,
            &hls_max_delta);
        hls_sweeps++;
        if ((uint16_t)hls_max_delta == 0) break;
    }
    printf("  HLS kernel: %d sweeps, final_delta=%d\n",
           hls_sweeps, (int)(uint16_t)hls_max_delta);

    // Verify
    int mismatch = 0;
    int checked = 0;
    for (int iy = 0; iy < map_y; iy++)
        for (int ix = 0; ix < map_x; ix++) {
            if (pen_ref[iy * map_x + ix] >= vi_ref::PENALTY_GOAL) continue;
            for (int it = 0; it < vi_ref::N_THETA; it++) {
                int idx = (iy * map_x + ix) * vi_ref::N_THETA + it;
                checked++;
                int diff = abs((int)val_ref[idx] - (int)val_hls[idx]);
                if (diff > 1) {
                    if (mismatch < 5)
                        printf("  MISMATCH (%d,%d,t=%d): ref=%u hls=%u\n",
                               ix, iy, it, val_ref[idx], val_hls[idx]);
                    mismatch++;
                }
            }
        }

    // Propagation check
    int finite = 0, total_free = 0;
    for (int iy = 0; iy < map_y; iy++)
        for (int ix = 0; ix < map_x; ix++) {
            if (pen_hls[iy * map_x + ix] >= vi_ref::PENALTY_GOAL) continue;
            total_free++;
            for (int it = 0; it < vi_ref::N_THETA; it++)
                if (val_hls[(iy * map_x + ix) * vi_ref::N_THETA + it] < vi_ref::MAX_VALUE) {
                    finite++;
                    break;
                }
        }

    printf("  Checked %d states, %d mismatches\n", checked, mismatch);
    printf("  Propagation: %d / %d free cells\n", finite, total_free);

    if (finite < total_free / 2) {
        printf("  FAIL: propagation insufficient\n");
        mismatch++;
    }

    delete[] pen_ref; delete[] val_ref;
    delete[] pen_hls; delete[] val_hls;

    return mismatch;
}

int main()
{
    printf("=== vi_sweep_stream C-Simulation Testbench ===\n");

    int errors = 0;

    // Test A: 20x20, fits in 1 strip (20 < 256)
    errors += run_test("small_single_strip", 20, 20, 15, 15, 0.05);

    // Test B: 300x20, forces 2 strips (300 > 256)
    errors += run_test("wide_multi_strip", 300, 20, 280, 15, 0.05);

    if (errors > 0) {
        printf("\n*** TESTBENCH FAILED (%d errors) ***\n", errors);
        return 1;
    }
    printf("\n*** TESTBENCH PASSED ***\n");
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add fpga/hls/vi_sweep_stream/tb/vi_sweep_stream_tb.cpp
git commit -m "feat(hls): add streaming kernel testbench with single and multi-strip tests"
```

---

### Task 7: Build scripts

**Files:**
- Create: `fpga/scripts/run_csim_stream.tcl`
- Create: `fpga/scripts/export_hls_ip_stream.tcl`
- Modify: `fpga/scripts/Makefile`

- [ ] **Step 1: Create run_csim_stream.tcl**

```tcl
# fpga/scripts/run_csim_stream.tcl
set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_stream"]
set part       "xczu3eg-sbva484-1-i"

open_project -reset hls_build_stream
set_top vi_sweep_stream
add_files "$hls_dir/src/vi_sweep_stream_top.cpp"
add_files "$hls_dir/src/stream_strip.cpp"
add_files "$hls_dir/src/compute_row.cpp"
add_files "$hls_dir/src/load_store_row.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_stream_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

open_solution -reset "solution1" -flow_target vivado
set_part $part
create_clock -period 6.67 -name default

csim_design

close_project
```

- [ ] **Step 2: Create export_hls_ip_stream.tcl**

```tcl
# fpga/scripts/export_hls_ip_stream.tcl
set script_dir [file normalize [file dirname [info script]]]
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_stream"]
set ip_dst     [file normalize "$script_dir/../vivado/ultra96v2/ip_repo"]
set part       "xczu3eg-sbva484-1-i"

open_project -reset hls_build_stream
set_top vi_sweep_stream
add_files "$hls_dir/src/vi_sweep_stream_top.cpp"
add_files "$hls_dir/src/stream_strip.cpp"
add_files "$hls_dir/src/compute_row.cpp"
add_files "$hls_dir/src/load_store_row.cpp"
add_files -tb "$hls_dir/tb/vi_sweep_stream_tb.cpp"
add_files -tb "$hls_dir/tb/vi_reference.cpp"

open_solution -reset "solution1" -flow_target vivado
set_part $part
create_clock -period 6.67 -name default

csynth_design
export_design -format ip_catalog -output $ip_dst

close_project
puts "INFO: HLS IP (vi_sweep_stream) exported to $ip_dst"
```

- [ ] **Step 3: Add targets to Makefile**

Append to `fpga/scripts/Makefile`:

```makefile
# Line-buffer streaming kernel
csim_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f run_csim_stream.tcl

hls_stream:
	cd $(SCRIPTS_DIR) && vitis_hls -f export_hls_ip_stream.tcl

clean_hls_stream:
	rm -rf $(SCRIPTS_DIR)/hls_build_stream
```

- [ ] **Step 4: Run C-simulation**

```bash
cd fpga/scripts && make csim_stream
```

Expected: `*** TESTBENCH PASSED ***` for both test cases.

- [ ] **Step 5: Commit**

```bash
git add fpga/scripts/run_csim_stream.tcl fpga/scripts/export_hls_ip_stream.tcl fpga/scripts/Makefile
git commit -m "feat(build): add csim_stream and hls_stream build targets"
```

---

### Task 8: Vivado Block Design (HP0/HP1 split)

**Files:**
- Modify: `fpga/vivado/ultra96v2/create_bd.tcl`

- [ ] **Step 1: Rewrite create_bd.tcl**

Replace the entire file. Key changes vs current:
- IP name: `vi_sweep_stream` (was `vi_sweep`)
- Enable HP1 in addition to HP0
- CU0 gmem0/gmem1 → HP0, CU1 gmem0/gmem1 → HP1
- Two separate data SmartConnects (2 masters each)

```tcl
# fpga/vivado/ultra96v2/create_bd.tcl
create_bd_design "vi_bd"

# --- Zynq UltraScale+ PS ---
set zynq [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $zynq

# Enable HP0 + HP1 for data, disable unused HPM1
set_property -dict [list \
    CONFIG.PSU__USE__S_AXI_GP2 {1} \
    CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
    CONFIG.PSU__USE__S_AXI_GP3 {1} \
    CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
] $zynq

# --- 2x vi_sweep_stream HLS IPs ---
set cu0 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep_stream:1.0 vi_sweep_stream_cu0]
set cu1 [create_bd_cell -type ip -vlnv xilinx.com:hls:vi_sweep_stream:1.0 vi_sweep_stream_cu1]

# --- Data SmartConnect for CU0 (2 AXI masters -> HP0) ---
set data_smc0 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 data_smc0]
set_property CONFIG.NUM_SI {2} $data_smc0

# --- Data SmartConnect for CU1 (2 AXI masters -> HP1) ---
set data_smc1 [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 data_smc1]
set_property CONFIG.NUM_SI {2} $data_smc1

# --- Control SmartConnect (1 GP master -> 2 control slaves) ---
set ctrl_smc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 ctrl_smc]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] $ctrl_smc

# --- Reset ---
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0]

# --- Clock and reset wiring ---
set clk [get_bd_pins zynq_ps/pl_clk0]
set rstn [get_bd_pins proc_sys_reset_0/peripheral_aresetn]

connect_bd_net $clk \
    [get_bd_pins data_smc0/aclk] \
    [get_bd_pins data_smc1/aclk] \
    [get_bd_pins ctrl_smc/aclk] \
    [get_bd_pins vi_sweep_stream_cu0/ap_clk] \
    [get_bd_pins vi_sweep_stream_cu1/ap_clk] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk] \
    [get_bd_pins zynq_ps/saxihp0_fpd_aclk] \
    [get_bd_pins zynq_ps/saxihp1_fpd_aclk] \
    [get_bd_pins zynq_ps/maxihpm0_fpd_aclk]

connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins proc_sys_reset_0/ext_reset_in]

connect_bd_net $rstn \
    [get_bd_pins data_smc0/aresetn] \
    [get_bd_pins data_smc1/aresetn] \
    [get_bd_pins ctrl_smc/aresetn] \
    [get_bd_pins vi_sweep_stream_cu0/ap_rst_n] \
    [get_bd_pins vi_sweep_stream_cu1/ap_rst_n]

# --- Interrupt ---
set irq_concat [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 irq_concat]
set_property -dict [list CONFIG.NUM_PORTS {2} CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1}] $irq_concat
connect_bd_net [get_bd_pins vi_sweep_stream_cu0/interrupt] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins vi_sweep_stream_cu1/interrupt] [get_bd_pins irq_concat/In1]
connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins zynq_ps/pl_ps_irq0]
set_property -dict [list CONFIG.PSU__USE__IRQ0 {1} CONFIG.PSU__IRQ_P2F_IRQ0_SELECT {1}] [get_bd_cells zynq_ps]

# --- Control path ---
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_smc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M00_AXI] [get_bd_intf_pins vi_sweep_stream_cu0/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins ctrl_smc/M01_AXI] [get_bd_intf_pins vi_sweep_stream_cu1/s_axi_control]

# --- Data path: CU0 -> HP0, CU1 -> HP1 ---
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu0/m_axi_gmem0] [get_bd_intf_pins data_smc0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu0/m_axi_gmem1] [get_bd_intf_pins data_smc0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_smc0/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu1/m_axi_gmem0] [get_bd_intf_pins data_smc1/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins vi_sweep_stream_cu1/m_axi_gmem1] [get_bd_intf_pins data_smc1/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_smc1/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP1_FPD]

# --- Address assignment ---
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW]
assign_bd_address [get_bd_addr_segs zynq_ps/SAXIGP3/HP1_DDR_LOW]
assign_bd_address [get_bd_addr_segs vi_sweep_stream_cu0/s_axi_control/Reg]
assign_bd_address [get_bd_addr_segs vi_sweep_stream_cu1/s_axi_control/Reg]

validate_bd_design
save_bd_design

puts "INFO: Block design 'vi_bd' created (2 CU streaming, HP0+HP1)"
```

- [ ] **Step 2: Commit**

```bash
git add fpga/vivado/ultra96v2/create_bd.tcl
git commit -m "feat(vivado): update BD for vi_sweep_stream with HP0/HP1 split"
```

---

### Task 9: HLS synthesis + bitstream

- [ ] **Step 1: Run HLS synthesis**

```bash
cd fpga/scripts && make hls_stream
```

Expected: synthesis completes, IP exported to `fpga/vivado/ultra96v2/ip_repo`.
Review `hls_build_stream/solution1/syn/report/csynth.rpt` for:
- BRAM usage ≤ 50% (per CU, so total ≤ budget when doubled)
- LOOP_T achieves II=1
- No timing violations at 6.67ns

- [ ] **Step 2: Build Vivado project + bitstream**

```bash
cd fpga/scripts && make vivado
```

Expected: `vi_bd_wrapper.bit` and `vi_bd_wrapper.hwh` generated.

- [ ] **Step 3: Copy bitstream to pynq directory**

```bash
cp fpga/vivado/ultra96v2/vi_ultra96v2/vi_ultra96v2.runs/impl_1/vi_bd_wrapper.bit fpga/pynq/
cp fpga/vivado/ultra96v2/vi_ultra96v2/vi_ultra96v2.gen/sources_1/bd/vi_bd/hw_handoff/vi_bd.hwh fpga/pynq/vi_bd_wrapper.hwh
```

- [ ] **Step 4: Commit**

```bash
git add fpga/pynq/vi_bd_wrapper.bit fpga/pynq/vi_bd_wrapper.hwh
git commit -m "feat: generate streaming kernel bitstream"
```

---

### Task 10: PYNQ overlay and notebook update

**Files:**
- Modify: `fpga/pynq/vi_overlay.py`
- Modify: `fpga/pynq/explore_map.ipynb`

- [ ] **Step 1: Update vi_overlay.py**

The register layout changes because `num_tiles_x`, `num_tiles_y` are removed. New layout (from HLS synthesis report — verify offsets after Task 9):

```python
"""Value Iteration FPGA overlay — streaming kernel for PYNQ on Ultra96-V2.

After HLS synthesis, verify register offsets in:
  hls_build_stream/solution1/impl/misc/drivers/vi_sweep_stream_v1_0/src/xvi_sweep_stream_hw.h
"""

import numpy as np
from pynq import Overlay, allocate

# --- AXI-Lite register offsets (UPDATE after synthesis) ---
AP_CTRL          = 0x00
ADDR_VALUE_TABLE = 0x10
ADDR_PENALTY     = 0x1C
ADDR_TRANS       = 0x28
ADDR_MAP_X       = 0x34
ADDR_MAP_Y       = 0x3C
ADDR_CU_ID       = 0x44
ADDR_MAX_DELTA   = 0x4C

N_THETA = 60


def _write_addr64(ip, offset, addr):
    ip.write(offset, addr & 0xFFFFFFFF)
    ip.write(offset + 4, (addr >> 32) & 0xFFFFFFFF)


class VIOverlay:
    def __init__(self, bitstream_path: str):
        self.ol = Overlay(bitstream_path)
        self.cu0 = self.ol.vi_sweep_stream_cu0
        self.cu1 = self.ol.vi_sweep_stream_cu1
```

- [ ] **Step 2: Update explore_map.ipynb sweep cell**

Replace cell-5 (FPGA sweep loop). Key changes:
- Remove tile padding (no `num_tiles_x/y`)
- `cu_id` instead of `cu_id` as checkerboard → now 0=forward, 1=reverse
- Both CUs kicked simultaneously each sweep

The updated cell content for the FPGA sweep loop:

```python
# === FPGA sweep loop with per-sweep benchmarking ===
from pynq import Overlay, allocate
from tqdm.auto import tqdm
from vi_overlay import (
    AP_CTRL, ADDR_VALUE_TABLE, ADDR_PENALTY, ADDR_TRANS,
    ADDR_MAP_X, ADDR_MAP_Y, ADDR_CU_ID, ADDR_MAX_DELTA, _write_addr64,
)

print("Loading overlay...")
ol  = Overlay(BITSTREAM)
cu0 = ol.vi_sweep_stream_cu0
cu1 = ol.vi_sweep_stream_cu1

val_buf = allocate(shape=value.shape,   dtype=np.uint16)
pen_buf = allocate(shape=penalty.shape, dtype=np.uint16)
trn_buf = allocate(shape=trans.shape,   dtype=np.uint32)
np.copyto(val_buf, value)
np.copyto(pen_buf, penalty)
np.copyto(trn_buf, trans)
val_buf.sync_to_device()
pen_buf.sync_to_device()
trn_buf.sync_to_device()

for cu in (cu0, cu1):
    _write_addr64(cu, ADDR_VALUE_TABLE, val_buf.device_address)
    _write_addr64(cu, ADDR_PENALTY,     pen_buf.device_address)
    _write_addr64(cu, ADDR_TRANS,       trn_buf.device_address)
    cu.write(ADDR_MAP_X, MAP_X)
    cu.write(ADDR_MAP_Y, MAP_Y)

# CU0=forward, CU1=reverse (fixed, no per-sweep change needed)
cu0.write(ADDR_CU_ID, 0)
cu1.write(ADDR_CU_ID, 1)

history = []
t_start = time.time()
pbar = tqdm(range(MAX_SWEEPS), desc="VI sweeps", unit="sweep")
for sweep in pbar:
    t0 = time.time()
    cu0.write(AP_CTRL, 0x01); cu1.write(AP_CTRL, 0x01)
    while not (cu0.read(AP_CTRL) & 0x02): pass
    while not (cu1.read(AP_CTRL) & 0x02): pass
    d0 = cu0.read(ADDR_MAX_DELTA)
    d1 = cu1.read(ADDR_MAX_DELTA)
    max_delta = max(d0, d1)
    dt_ms = (time.time() - t0) * 1e3
    history.append({"sweep": sweep, "max_delta": int(max_delta), "dt_ms": dt_ms})
    pbar.set_postfix(max_delta=int(max_delta), dt_ms=f"{dt_ms:.1f}")
    if max_delta <= THRESHOLD:
        pbar.close()
        break
else:
    pbar.close()
elapsed = time.time() - t_start

val_buf.sync_from_device()
np.copyto(value, val_buf)
val_buf.freebuffer(); pen_buf.freebuffer(); trn_buf.freebuffer()

converged = history[-1]["max_delta"] <= THRESHOLD
print(
    f"converged={converged}  sweeps={len(history)}  "
    f"elapsed={elapsed:.3f}s  final_delta={history[-1]['max_delta']}"
)

hist_df = pd.DataFrame(history)
hist_df.to_csv(OUT_DIR / "sweep_history.csv", index=False)
hist_df.tail()
```

- [ ] **Step 3: Update penalty cell — remove tile padding**

In cell-3, remove the TILE_W/TILE_H padding logic. The streaming kernel handles arbitrary map sizes natively. Replace the padding block with:

```python
MAP_Y, MAP_X = inflated.shape
print(f"Map: {MAP_X} x {MAP_Y} (no padding needed for streaming kernel)")
```

- [ ] **Step 4: Commit**

```bash
git add fpga/pynq/vi_overlay.py fpga/pynq/explore_map.ipynb
git commit -m "feat(pynq): update overlay and notebook for streaming kernel"
```

---

### Task 11: On-board verification

- [ ] **Step 1: Deploy to Ultra96-V2**

Copy to the board:
- `fpga/pynq/vi_bd_wrapper.bit`
- `fpga/pynq/vi_bd_wrapper.hwh`
- `fpga/pynq/vi_overlay.py`
- `fpga/pynq/explore_map.ipynb`
- `fpga/pynq/demo_vi.ipynb`
- Map files to `./maps/`

- [ ] **Step 2: Run demo_vi.ipynb first (40×40 sanity check)**

Verify the small map converges and values are reasonable. This confirms the bitstream loads and registers work.

- [ ] **Step 3: Run explore_map.ipynb with tsudanuma**

Expected results (vs tile-based baseline):
- Sweep time: ~200 ms (was 6,400 ms) → **32× per-sweep speedup**
- Sweeps to converge: ~10-20 (was ~52) → **3-5× fewer sweeps**
- Total: ~2-4 s (was ~333 s) → **~100× total speedup**

- [ ] **Step 4: Run solo CU benchmark**

```python
# Same diagnostic as before: solo cu0, solo cu1, both
```

Verify HP0/HP1 split eliminated DDR contention: `both ≈ max(solo0, solo1)`.
