# Value Iteration FPGA — Line-Buffer Streaming Kernel Design

**Date:** 2026-04-12
**Parent spec:** `2026-04-10-value-iteration-fpga-design.md` (Phase 1-2)
**Goal:** タイルベース kernel を line-buffer streaming に置き換え、sweep あたりの Gauss-Seidel 伝播を全マップ規模に拡大し、収束時間を ~100 倍短縮する。

---

## 1. Background & Motivation

### 現状の問題

Phase 1-2 で実装したタイルベース kernel (32×32 tile + 6-cell halo) は:

- **sweep あたりの伝播距離が ~38 セル** (tile + halo) に制限される
- 1984×133 @0.15m のマップで収束に **~52 sweep × 6.4 s = 333 秒** かかる
- CPU 参照実装 (8 threads, full-map Gauss-Seidel) の 50 秒と比較して 6.7 倍遅い

### 根本原因

タイル間で Gauss-Seidel 伝播が途切れる。各タイルは DDR からロード → BRAM で更新 → DDR にストアするため、隣接タイルの更新結果は次の sweep まで反映されない。

### 解決策

**Line-buffer streaming:** マップを行単位でストリーミング処理し、スライディングウィンドウ内で in-place Gauss-Seidel 更新する。strip 内では X・Y 両方向に 1 sweep で全域伝播。2 CU が forward/reverse を同時実行し、1 パスで全方位伝播。

---

## 2. Architecture

### 2.1 Overview

```
┌─────────────────────────────────────────────────────────────┐
│  PYNQ Host (ARM)                                             │
│    sweep loop: kick CU0 + CU1 → wait done → check delta     │
└────────┬─────────────────────────────────┬──────────────────┘
         │ CU0 (forward)                   │ CU1 (reverse)
         │ HP0                             │ HP1
┌────────▼─────────────────┐  ┌────────────▼─────────────────┐
│  vi_sweep_stream CU0     │  │  vi_sweep_stream CU1         │
│  cu_id=0: Y+, X+ scan   │  │  cu_id=1: Y-, X- scan        │
│  line-buffer in BRAM     │  │  line-buffer in BRAM          │
└────────┬─────────────────┘  └────────────┬─────────────────┘
         │                                 │
         └────────────┬────────────────────┘
                      │ DDR (shared)
              ┌───────▼───────┐
              │ value_table   │  read/write by both CUs
              │ penalty_table │  read-only
              │ trans_table   │  read-only (360 words)
              └───────────────┘
```

### 2.2 Key Design Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | Line-buffer streaming (タイル廃止) | Gauss-Seidel 伝播を strip 全域に拡大 |
| 2 | 2 CU: forward + reverse 同時実行 | 1 パスで全方位伝播、lock-free (論文と同原理) |
| 3 | X-strip 分割 (STRIP_W_MAX=256) | 2 CU で BRAM を分割しても収まる |
| 4 | Y 方向はストリーミング (分割不要) | スライディングウィンドウで BRAM 使用量は行数に依存しない |
| 5 | HP0/HP1 ポート分離 | DDR 帯域競合を軽減 (実測 22% overhead → 最小化) |
| 6 | cu_id=0 のみ kick で 1 CU 動作可 | デバッグ容易、段階的検証 |

---

## 3. Compile-Time Constants

```c
#define HALO_MAX      6                        // max |diy|, |dix| at 0.05m
#define WINDOW_ROWS   (2 * HALO_MAX + 1)       // 13
#define STRIP_W_MAX   256                       // BRAM-limited per CU
#define N_THETA       60
#define N_ACTIONS      6
#define MAX_VALUE      0xFFFF
#define PENALTY_OBSTACLE 0xFFFF
#define PENALTY_GOAL     0xFFFE
```

---

## 4. BRAM Budget (per CU)

ZU3EG: 216 BRAM36K = 972 KB total, 486 KB per CU.

| Buffer | Size | BRAM36K |
|--------|------|---------|
| `val_buf[WINDOW_ROWS][STRIP_W_MAX][N_THETA]` (partition dim=3) | 13×256×60×2B = 390 KB | 60 slices × ceil(6656B / 4608B) = 60×2 = 120 |
| `pen_buf_{0,1,2}[WINDOW_ROWS][STRIP_W_MAX]` (3 copies) | 3×13×256×2B = 20 KB | 3×2 = 6 |
| `delta_table[6][60][3]` (registers) | 1080 B | 0 (LUT) |
| **合計 per CU** | **~410 KB** | **~126** |
| **2 CU 合計** | **~820 KB** | **~252 / 216** |

252 > 216 なので **BRAM18K を活用**する。ZU3EG は 216 BRAM36K を 432 BRAM18K として使える。val_buf の各 theta slice は 6656B = 53 Kbit で、BRAM18K (18 Kbit) 3 個で収まる。60×3×2CU = 360 BRAM18K + penalty 12 = 372 / 432 = **86%**。収まる。

---

## 5. AXI Interface

### 5.1 m_axi Ports

| Bundle | Port | Direction | Notes |
|--------|------|-----------|-------|
| `gmem0` | `value_table` | R/W | CU0→HP0, CU1→HP1 |
| `gmem1` | `penalty_table` | R | CU0→HP0, CU1→HP1 |
| `gmem1` | `trans_table` | R | 360 words, load once |

### 5.2 s_axilite Registers

| Name | Type | Direction | Description |
|------|------|-----------|-------------|
| `value_table` | `uint64` | in | DDR base address |
| `penalty_table` | `uint64` | in | DDR base address |
| `trans_table` | `uint64` | in | DDR base address |
| `map_x` | `uint32` | in | Map width (cells) |
| `map_y` | `uint32` | in | Map height (cells) |
| `cu_id` | `uint32` | in | 0=forward, 1=reverse |
| `max_delta` | `uint16` | out | Max value change in sweep |

`num_tiles_x`, `num_tiles_y` は削除。strip 数は kernel 内で算出。

---

## 6. Kernel Execution Flow

```c
void vi_sweep_stream(
    value_t *value_table,       // m_axi gmem0
    penalty_t *penalty_table,   // m_axi gmem1
    ap_uint<32> *trans_table,   // m_axi gmem1
    int map_x, int map_y,
    int cu_id,
    value_t *max_delta)         // s_axilite
{
    // 1. Load transition table → registers
    offset_t delta_table[N_ACTIONS][N_THETA][3];
    load_transitions(trans_table, delta_table);

    // 2. Compute strip layout
    int num_strips_x = (map_x + STRIP_W_MAX - 1) / STRIP_W_MAX;

    value_t global_max_delta = 0;

    // 3. Strip loop (X-direction)
    for (int sx_raw = 0; sx_raw < num_strips_x; sx_raw++) {
        int sx = (cu_id == 0) ? sx_raw : (num_strips_x - 1 - sx_raw);
        int strip_x0 = sx * STRIP_W_MAX;
        int strip_w  = min(STRIP_W_MAX, map_x - strip_x0);

        // 4. Row streaming (Y-direction) with sliding window
        stream_strip(value_table, penalty_table, delta_table,
                     map_x, map_y, strip_x0, strip_w,
                     cu_id, &global_max_delta);
    }

    *max_delta = global_max_delta;
}
```

---

## 7. Stream Processing (per strip)

```c
void stream_strip(
    value_t *value_table,
    penalty_t *penalty_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3],
    int map_x, int map_y,
    int strip_x0, int strip_w,
    int cu_id,
    value_t *global_max_delta)
{
    // Line buffers (BRAM)
    value_t   val_buf[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX][N_THETA];
    penalty_t pen_buf_0[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX];
    penalty_t pen_buf_1[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX];
    penalty_t pen_buf_2[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX];
    #pragma HLS ARRAY_PARTITION variable=val_buf complete dim=3
    #pragma HLS BIND_STORAGE variable=val_buf type=ram_t2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_1 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=pen_buf_2 type=ram_2p impl=bram

    int buf_w = strip_w + 2 * HALO_MAX;  // buffer width with halo

    // --- Initialize window: load first WINDOW_ROWS rows ---
    for (int wr = 0; wr < WINDOW_ROWS; wr++) {
        int gy = row_index(cu_id, map_y, -HALO_MAX + wr);
        load_row(val_buf[wr], pen_buf_0[wr], pen_buf_1[wr], pen_buf_2[wr],
                 value_table, penalty_table,
                 gy, strip_x0, buf_w, map_x, map_y);
    }

    // --- Stream through rows ---
    for (int iy_raw = 0; iy_raw < map_y; iy_raw++) {
        int iy = (cu_id == 0) ? iy_raw : (map_y - 1 - iy_raw);
        int win_center = iy_raw % WINDOW_ROWS;  // circular buffer index

        // Compute Bellman update for this row
        compute_row(val_buf, pen_buf_0, pen_buf_1, pen_buf_2,
                    delta_table, win_center,
                    strip_w, cu_id, global_max_delta);

        // Store updated row to DDR
        store_row(val_buf[win_center], value_table,
                  iy, strip_x0, strip_w, map_x);

        // Evict oldest row, load next future row
        int next_gy = row_index(cu_id, map_y, iy_raw + HALO_MAX + 1);
        int evict_slot = (iy_raw + HALO_MAX + 1) % WINDOW_ROWS;
        load_row(val_buf[evict_slot], pen_buf_0[evict_slot],
                 pen_buf_1[evict_slot], pen_buf_2[evict_slot],
                 value_table, penalty_table,
                 next_gy, strip_x0, buf_w, map_x, map_y);
    }
}
```

### 7.1 Circular Buffer Indexing

Window は WINDOW_ROWS エントリの循環バッファ。`iy_raw` が進むたびに最古の行が evict/reload される。BRAM アドレスは `(base + offset) % WINDOW_ROWS` で計算。

### 7.2 Row Compute

```c
void compute_row(
    value_t   val_buf[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX][N_THETA],
    penalty_t pen_buf_0[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX],
    penalty_t pen_buf_1[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX],
    penalty_t pen_buf_2[WINDOW_ROWS][STRIP_W_MAX + 2*HALO_MAX],
    offset_t  delta_table[N_ACTIONS][N_THETA][3],
    int win_center,
    int strip_w,
    int cu_id,
    value_t *global_max_delta)
{
    // X scan direction: cu_id=0 → 0..strip_w-1, cu_id=1 → strip_w-1..0
    LOOP_X: for (int ix_raw = 0; ix_raw < strip_w; ix_raw++) {
        int ix = (cu_id == 0) ? ix_raw : (strip_w - 1 - ix_raw);
        int bx = ix + HALO_MAX;

        penalty_t cell_pen = pen_buf_0[win_center][bx];
        bool skip = (cell_pen >= PENALTY_GOAL);

        LOOP_T: for (int it = 0; it < N_THETA; it++) {
            #pragma HLS PIPELINE II=1

            // 6 actions: same BRAM port scheduling as tile-based kernel
            // Actions 0,1 → theta bank[it]
            // Actions 2,4 → theta bank[it+3]
            // Actions 3,5 → theta bank[it-3]
            // Penalty: 3 copies × 2 ports = 6 reads

            value_t old_val = val_buf[win_center][bx][it];

            // Neighbor lookups (circular buffer row index)
            // ny = (win_center + diy_offset) % WINDOW_ROWS
            // nx = bx + dix
            // nt = (it + dit) wrapped

            value_t c0 = eval_action(val_buf, pen_buf_0, delta_table,
                                     0, it, win_center, bx);
            value_t c1 = eval_action(val_buf, pen_buf_0, delta_table,
                                     1, it, win_center, bx);
            value_t c2 = eval_action(val_buf, pen_buf_1, delta_table,
                                     2, it, win_center, bx);
            value_t c3 = eval_action(val_buf, pen_buf_1, delta_table,
                                     3, it, win_center, bx);
            value_t c4 = eval_action(val_buf, pen_buf_2, delta_table,
                                     4, it, win_center, bx);
            value_t c5 = eval_action(val_buf, pen_buf_2, delta_table,
                                     5, it, win_center, bx);

            // Min-reduction tree
            value_t min01 = min(c0, c1);
            value_t min23 = min(c2, c3);
            value_t min45 = min(c4, c5);
            value_t min_cost = min(min(min01, min23), min45);

            value_t new_val = skip ? old_val : min_cost;
            val_buf[win_center][bx][it] = new_val;  // Gauss-Seidel in-place

            value_t d = (new_val > old_val) ? (new_val - old_val)
                                            : (old_val - new_val);
            if (!skip && d > *global_max_delta)
                *global_max_delta = d;
        }
    }
}
```

---

## 8. DDR Access Pattern

### 8.1 Read (per row load)

```
Value:   (STRIP_W + 2*HALO) × N_THETA × 2B = (256+12) × 60 × 2 = 32,160 B
Penalty: (STRIP_W + 2*HALO) × 2B = 268 × 2 = 536 B
Total per row: ~32.7 KB (contiguous in DDR → single burst)
```

### 8.2 Write (per row store)

```
Value: STRIP_W × N_THETA × 2B = 256 × 60 × 2 = 30,720 B per row
```

### 8.3 Bandwidth per sweep (1984×133 @0.15m)

```
Per strip: 133 rows × (32.7 KB read + 30.7 KB write) = 8.4 MB
8 strips: 67 MB
At HP0 bandwidth ~2 GB/s: 33 ms (compute-dominated, not bandwidth-limited)
```

---

## 9. Vivado BD Changes

### 現行 (タイルベース)

```
vi_sweep_cu0 ─┐
vi_sweep_cu1 ─┤── AXI Interconnect ── HP0 ── DDR
              └── GP0 (s_axilite)
```

### 新規 (line-buffer streaming)

```
vi_sweep_stream_cu0 ── HP0 ── DDR
vi_sweep_stream_cu1 ── HP1 ── DDR
Both ── GP0 (s_axilite)
```

HP ポートを分離し、帯域競合を解消。

---

## 10. Host / PYNQ Changes

### vi_overlay.py

- レジスタオフセット更新 (`NUM_TILES_X`, `NUM_TILES_Y` 削除、レイアウト変更)
- `cu1` を `ol.vi_sweep_stream_cu1` に変更
- `_write_addr64` 等ヘルパーはそのまま

### explore_map.ipynb

- タイルパディングロジック削除 (任意の map_x, map_y を直接渡せる)
- sweep ループ: 両 CU に `cu_id` を書いて同時キック (現行とほぼ同じ)
- `scan_dir` レジスタ不要 (cu_id が走査方向を決定)

```python
cu0.write(ADDR_CU_ID, 0)  # forward
cu1.write(ADDR_CU_ID, 1)  # reverse
cu0.write(AP_CTRL, 0x01); cu1.write(AP_CTRL, 0x01)
while not (cu0.read(AP_CTRL) & 0x02): pass
while not (cu1.read(AP_CTRL) & 0x02): pass
max_delta = max(cu0.read(ADDR_MAX_DELTA), cu1.read(ADDR_MAX_DELTA))
```

### 1 CU デバッグモード

```python
cu0.write(ADDR_CU_ID, 0)
cu0.write(AP_CTRL, 0x01)
while not (cu0.read(AP_CTRL) & 0x02): pass
# 次の sweep で reverse:
cu0.write(ADDR_CU_ID, 1)
cu0.write(AP_CTRL, 0x01)
...
```

---

## 11. Testbench

- 既存の CPU 参照実装 (全マップ in-place Gauss-Seidel) を golden reference として維持
- HLS テストベンチ: 20×20 マップで CU0 (forward) と CU1 (reverse) を交互にシミュレート
- 許容誤差: ±1 (Gauss-Seidel 走査順序の差異)
- 追加テスト: strip 境界をまたぐ伝播の検証 (map_x > STRIP_W_MAX のケース)

---

## 12. Expected Performance

### 1984×133 @0.15m (Tsudanuma downsampled)

| | Tile-based (現行) | Line-buffer streaming |
|---|---|---|
| Sweep time | 6,400 ms | ~200 ms |
| Sweeps to converge | ~52 | ~5-10 |
| **Total** | **~333 s** | **~1-2 s** |
| Speedup | | **~200×** |

### 5888×400 @0.05m (Tsudanuma native)

| | Tile-based (推定) | Line-buffer streaming |
|---|---|---|
| Sweep time | ~57 s | ~600 ms |
| Sweeps to converge | ~155 | ~15-20 |
| **Total** | **~2.5 hours** | **~9-12 s** |

### 14000×800 @0.05m (spec worst case)

| | Line-buffer streaming |
|---|---|
| Strips (per CU) | 28 |
| Sweep time | ~3.4 s |
| Sweeps to converge | ~40-60 |
| **Total** | **~2-3.5 min** |
| Phase 1 target | 60 s |

Worst case は Phase 1 target (60s) を超える可能性があるが、タイルベースの推定 (~数時間) と比較して桁違いの改善。

---

## 13. Implementation Phases

1. **HLS kernel** (`vi_sweep_stream.cpp` + サブモジュール) — C simulation + synthesis
2. **Testbench** (`vi_sweep_stream_tb.cpp`) — 20×20 + strip 境界テスト
3. **Vivado BD** — 2 CU instantiation, HP0/HP1 分離
4. **PYNQ integration** — `vi_overlay.py` 更新, `explore_map.ipynb` 更新
5. **実機ベンチマーク** — タイルベースとの比較
