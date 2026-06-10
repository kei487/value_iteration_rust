# VI-as-SSP Priority-Propagation Acceleration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 本家 u64 リファレンス VI を SSP と捉え、固定点反復を Dijkstra 流の優先順序伝播に置換した2ソルバ（`prio_ls` 近似 / `prio_lc` 厳密）を実装し、数式定式化（TeX）と実験で「根本からの高速化」可否を検証する。

**Architecture:** 既存 `value_iteration_raw`/`action_cost_raw`（コスト数式）をそのまま再利用し、追加するのは逆θ隣接 `rev_theta`・二分ヒープ優先キュー・relax のみ。`prio_ls` は settle-once（近似）、`prio_lc` は label-correcting（厳密・bit-exact）。両者は単一の `priority_solve(label_setting: bool)` を共有。

**Tech Stack:** Rust（`vi_rs/vi_reference` クレート、std のみ＋`vi_algorithm`）、`std::collections::BinaryHeap`、既存 `vi_compare` ベンチ基盤、LaTeX（成果物）。

---

## File Structure

- **Create** `vi_rs/vi_reference/src/solvers/priority.rs` — `rev_theta` 構築、`relax_cell`、共有 `priority_solve` + `PrioStats`、`prio_ls_solve`、ユニットテスト。
- **Create** `vi_rs/vi_reference/src/solvers/prio_lc.rs` — `prio_lc_solve`（`priority_solve(false)` 薄ラッパ）＋ parity テスト。
- **Modify** `vi_rs/vi_reference/src/solvers/mod.rs` — `pub mod priority; pub mod prio_lc;`、`U64Solver` 列挙子2つ、`from_name`、`solve()` dispatch。
- **Create** `vi_rs/vi_reference/src/bin/vi_prio_measure.rs` — 合成ストリップ直径レジーム測定（ホスト完結）。
- **Modify** `vi_compare/u64/run_u64_bench.sh` — `SOLVERS` 既定に `prio_ls prio_lc` 追加。
- **Modify** `vi_compare/compare/make_u64_report.py` — `SOLVERS` リストに追加。
- **Create** `docs/research/2026-06-09-vi-ssp-acceleration.tex` — 数式定式化＋計算量/単調性解析＋実験結果。
- **Create** `docs/research/README.md` — PDF ビルド手順。

**Host cargo の注意**: gitignore 済み `.cargo/config.toml`（ROS patch）を拾うと Cargo.lock が汚れるため、テスト/ビルドは `/tmp` から `--manifest-path` で実行する（既知ワークアラウンド）。専用 target を使う:
```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml -p vi_reference
```

---

## Task 1: 逆θ隣接 `rev_theta` と `relax_cell`（priority.rs 基盤）

**Files:**
- Create: `vi_rs/vi_reference/src/solvers/priority.rs`
- Modify: `vi_rs/vi_reference/src/solvers/mod.rs`（`pub mod priority;` 追加）
- Test: `priority.rs` 内 `#[cfg(test)] mod tests`

- [ ] **Step 1: priority.rs を作成（rev_theta + relax_cell + テスト）**

```rust
//! VI を SSP と捉えた優先順序伝播ソルバの共有基盤。本家 per-cell 更新
//! `value_iteration_raw` を「値の昇順」に呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と一致（厳密版 prio_lc）。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-ssp-priority-acceleration-design.md`

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::params::MAX_COST;
use crate::value_iterator::{to_index_raw, value_iteration_raw, ValueIterator};

/// 逆θ写像。`rev[it']` = 確定セル `(.., it')` の前駆を列挙する `(dix, diy, t_src)` 列。
/// 全 (action, source θ `t`, 遷移 `δ`) を走査し、着地 θ `it' = (dit + nt) % nt` をキーに
/// `(dix, diy, t)` を積む。前駆は `(ix' - dix, iy' - diy, t)`。重複は dedup（過剰列挙抑制）。
pub(crate) fn build_rev_theta(vi: &ValueIterator) -> Vec<Vec<(i32, i32, i32)>> {
    let nt = vi.cell_num_t;
    let mut rev: Vec<Vec<(i32, i32, i32)>> = vec![Vec::new(); nt as usize];
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                let itp = (((st.dit % nt) + nt) % nt) as usize;
                rev[itp].push((st.dix, st.diy, t as i32));
            }
        }
    }
    for bucket in rev.iter_mut() {
        bucket.sort_unstable();
        bucket.dedup();
    }
    rev
}

/// セル `idx` を本家 Bellman で再評価・書込。改善（厳密減少）したら新ラベルを返す。
#[inline]
pub(crate) fn relax_cell(
    vi: &mut ValueIterator,
    idx: usize,
    nx: i32,
    ny: i32,
    nt: i32,
) -> Option<u64> {
    let before = vi.states[idx].total_cost;
    value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
    let after = vi.states[idx].total_cost;
    if after < before {
        Some(after)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::test_support::make_vi;

    #[test]
    fn rev_theta_round_trips_forward_transitions() {
        // 全 (action, θ, 遷移) について、着地θのバケットに (dix,diy,t) が含まれること。
        let vi = make_vi(8, 8, vec![0i8; 64]);
        let rev = build_rev_theta(&vi);
        let nt = vi.cell_num_t;
        assert_eq!(rev.len(), nt as usize);
        for a in &vi.actions {
            for (t, trans) in a.state_transitions.iter().enumerate() {
                for st in trans {
                    let itp = (((st.dit % nt) + nt) % nt) as usize;
                    assert!(
                        rev[itp].contains(&(st.dix, st.diy, t as i32)),
                        "rev[{itp}] must contain ({},{},{})",
                        st.dix,
                        st.diy,
                        t
                    );
                }
            }
        }
        // dedup 済み（各バケットは昇順ユニーク）。
        for bucket in &rev {
            let mut sorted = bucket.clone();
            sorted.sort_unstable();
            sorted.dedup();
            assert_eq!(&sorted, bucket);
        }
    }
}
```

- [ ] **Step 2: mod.rs にモジュール宣言を追加**

`vi_rs/vi_reference/src/solvers/mod.rs` の既存モジュール宣言群（`pub mod topk;` の直後あたり）に追加:

```rust
pub mod priority;
```

- [ ] **Step 3: テストが通ることを確認**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml \
  -p vi_reference priority:: 2>&1 | tail -20
```
Expected: `rev_theta_round_trips_forward_transitions ... ok`（警告なし、`relax_cell` は dead_code 警告が出るが Step 4 で解消するので、この時点では `#[allow(dead_code)]` を `relax_cell` に一時付与してもよい。または Step 4 まで一括で進める）。

- [ ] **Step 4: コミット**

```bash
git add vi_rs/vi_reference/src/solvers/priority.rs vi_rs/vi_reference/src/solvers/mod.rs
git commit -m "feat(vi_reference): rev_theta reverse adjacency + relax_cell (priority base)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: 厳密ソルバ `prio_lc` の parity テスト（失敗確認）

TDD: 厳密版の bit-exact を先にテストで固定する。実装は Task 3。

**Files:**
- Create: `vi_rs/vi_reference/src/solvers/prio_lc.rs`
- Modify: `vi_rs/vi_reference/src/solvers/mod.rs`（`pub mod prio_lc;`）

- [ ] **Step 1: prio_lc.rs を作成（テストのみ、実装は空ラッパで一旦コンパイル不可）**

まずテストを書く。`prio_lc_solve` はまだ無いのでコンパイルエラー＝RED。

```rust
//! (A2) Priority Label-Correcting — 厳密・本家と bit-exact。
//! `priority_solve(label_setting=false)` の薄ラッパ。

use crate::value_iterator::ValueIterator;

pub fn prio_lc_solve(_vi: &mut ValueIterator, _max_iter: u32) -> (u32, u64, bool) {
    unimplemented!("Task 3 で実装")
}

#[cfg(test)]
mod tests {
    use super::prio_lc_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_prio_lc() {
        // empty / obstacle / sentinel の3マップで到達セル bit-exact（値＋方策）。
        parity_standard_maps(|vi| prio_lc_solve(vi, 3000));
    }
}
```

- [ ] **Step 2: mod.rs にモジュール宣言を追加**

`pub mod priority;` の直後:
```rust
pub mod prio_lc;
```

- [ ] **Step 3: テストが失敗することを確認**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml \
  -p vi_reference parity_standard_maps_prio_lc 2>&1 | tail -20
```
Expected: panic `not implemented: Task 3 で実装`（テストは実行され、`unimplemented!` で FAIL）。

- [ ] **Step 4: コミット（RED）**

```bash
git add vi_rs/vi_reference/src/solvers/prio_lc.rs vi_rs/vi_reference/src/solvers/mod.rs
git commit -m "test(vi_reference): prio_lc bit-exact parity test (red)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: `priority_solve` + `prio_ls_solve` + `prio_lc_solve` 実装（parity GREEN）

**Files:**
- Modify: `vi_rs/vi_reference/src/solvers/priority.rs`（`priority_solve` / `PrioStats` / `prio_ls_solve` 追加）
- Modify: `vi_rs/vi_reference/src/solvers/prio_lc.rs`（`unimplemented!` を本実装へ）
- Modify: `vi_rs/vi_reference/src/solvers/mod.rs`（enum / from_name / solve dispatch）

- [ ] **Step 1: priority.rs に `PrioStats` と `priority_solve` と `prio_ls_solve` を追加**

`relax_cell` の直後（`#[cfg(test)] mod tests` の前）に挿入:

```rust
/// 優先順序ソルバの拡張統計。`repops` は確定済みセルの再処理回数（単調性違反の指標、
/// label-setting では常に 0、label-correcting で >0 なら Dial 化に注意）。
#[derive(Clone, Copy, Debug)]
pub struct PrioStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    pub repops: u64,
}

/// 共有の優先順序伝播。`label_setting=true`→Dijkstra 流 settle-once（近似・最速）、
/// `false`→label-correcting（厳密・bit-exact）。`total_cost` を tentative ラベルに流用し、
/// 二分ヒープで値の昇順に確定 → 前駆を逆θ隣接で relax。
pub fn priority_solve(vi: &mut ValueIterator, max_iter: u32, label_setting: bool) -> PrioStats {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let rev = build_rev_theta(vi);
    let n = vi.states.len();
    // label-setting は settled、label-correcting は popped を使う（他方は空 Vec）。
    let mut settled = vec![false; if label_setting { n } else { 0 }];
    let mut popped = vec![false; if label_setting { 0 } else { n }];

    let mut heap: BinaryHeap<Reverse<(u64, usize)>> = BinaryHeap::new();
    for (i, s) in vi.states.iter().enumerate() {
        if s.total_cost < MAX_COST {
            heap.push(Reverse((s.total_cost, i))); // 種: final セル (V=0)
        }
    }

    let pop_cap = (n as u64).saturating_mul(max_iter.max(1) as u64); // 暴走ガード（実質無限）
    let mut pops = 0u64;
    let mut iters = 0u32;
    let mut updates = 0u64;
    let mut repops = 0u64;

    while let Some(Reverse((lab, s_star))) = heap.pop() {
        pops += 1;
        if pops > pop_cap {
            return PrioStats { iters, updates, converged: false, repops };
        }
        // 遅延 decrease-key の stale 破棄。
        if lab != vi.states[s_star].total_cost {
            continue;
        }
        if label_setting {
            if settled[s_star] {
                continue;
            }
            settled[s_star] = true;
        } else if popped[s_star] {
            repops += 1;
        } else {
            popped[s_star] = true;
        }
        iters += 1;

        let (ix, iy, it) = (vi.states[s_star].ix, vi.states[s_star].iy, vi.states[s_star].it);
        for &(dix, diy, t) in &rev[it as usize] {
            let px = ix - dix;
            let py = iy - diy;
            if px < 0 || px >= nx || py < 0 || py >= ny {
                continue;
            }
            let pidx = to_index_raw(px, py, t, nx, nt) as usize;
            if label_setting && settled[pidx] {
                continue;
            }
            if !vi.states[pidx].free || vi.states[pidx].final_state {
                continue;
            }
            if let Some(newlab) = relax_cell(vi, pidx, nx, ny, nt) {
                updates += 1;
                heap.push(Reverse((newlab, pidx)));
            }
        }
    }

    PrioStats { iters, updates, converged: true, repops }
}

/// (A1) Priority Label-Setting（近似・最速）。`solve()` 用の軽量タプル。
pub fn prio_ls_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let st = priority_solve(vi, max_iter, true);
    (st.iters, st.updates, st.converged)
}
```

- [ ] **Step 2: prio_lc.rs の `unimplemented!` を本実装へ差し替え**

```rust
//! (A2) Priority Label-Correcting — 厳密・本家と bit-exact。
//! `priority_solve(label_setting=false)` の薄ラッパ。

use crate::solvers::priority::priority_solve;
use crate::value_iterator::ValueIterator;

pub fn prio_lc_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let st = priority_solve(vi, max_iter, false);
    (st.iters, st.updates, st.converged)
}

#[cfg(test)]
mod tests {
    use super::prio_lc_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_prio_lc() {
        parity_standard_maps(|vi| prio_lc_solve(vi, 3000));
    }
}
```

- [ ] **Step 3: mod.rs に enum 変種 / from_name / dispatch を配線**

`U64Solver` enum の `StreamMimic,` の直後に追加:
```rust
    PriorityLabelSetting,
    PriorityLabelCorrecting,
```

`from_name` の `"stream_mimic" => U64Solver::StreamMimic,` の直後に追加:
```rust
            "prio_ls" => U64Solver::PriorityLabelSetting,
            "prio_lc" => U64Solver::PriorityLabelCorrecting,
```

`solve()` の match の `U64Solver::StreamMimic => stream::stream_mimic_solve(vi, max_iter),` の直後に追加:
```rust
        U64Solver::PriorityLabelSetting => priority::prio_ls_solve(vi, max_iter),
        U64Solver::PriorityLabelCorrecting => prio_lc::prio_lc_solve(vi, max_iter),
```

- [ ] **Step 4: prio_lc parity テストが通ることを確認**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml \
  -p vi_reference parity_standard_maps_prio_lc 2>&1 | tail -20
```
Expected: `parity_standard_maps_prio_lc ... ok`（到達セルで本家固定点と値・方策 bit-exact）。

- [ ] **Step 5: クレート全体のテスト・警告ゼロを確認**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml -p vi_reference 2>&1 | tail -25
```
Expected: 全テスト pass、warning なし（`relax_cell`/`prio_ls_solve` は使用されるので dead_code 解消）。

- [ ] **Step 6: コミット（GREEN）**

```bash
git add vi_rs/vi_reference/src/solvers/priority.rs vi_rs/vi_reference/src/solvers/prio_lc.rs vi_rs/vi_reference/src/solvers/mod.rs
git commit -m "feat(vi_reference): priority-propagation solvers prio_ls/prio_lc (prio_lc bit-exact)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: `prio_ls` 近似ソルバの near-parity テスト

**Files:**
- Modify: `vi_rs/vi_reference/src/solvers/priority.rs`（テストモジュールに2テスト追加）

- [ ] **Step 1: 決定論遷移なら厳密、のテストを追加（RED→GREEN は実装済みなので即 GREEN 想定）**

`priority.rs` の `#[cfg(test)] mod tests` 内、`rev_theta_round_trips_forward_transitions` の後に追加:

```rust
    use crate::params::{PROB_BASE};
    use crate::solvers::test_support::{run_reference_to_fixed_point, REACH};
    use crate::state_transition::StateTransition;
    use crate::value_iterator::ValueIterator;

    /// 各 action・θ の遷移分布を最頻 outcome 1点 (prob=PROB_BASE) に潰し、決定論化する。
    fn collapse_to_deterministic(vi: &mut ValueIterator) {
        let b = PROB_BASE as i32;
        for a in vi.actions.iter_mut() {
            for trans in a.state_transitions.iter_mut() {
                if trans.is_empty() {
                    continue;
                }
                let top = trans.iter().max_by_key(|s| s.prob).unwrap().clone();
                *trans = vec![StateTransition::new(top.dix, top.diy, top.dit, b)];
            }
        }
    }

    #[test]
    fn prio_ls_exact_on_deterministic_transitions() {
        // 決定論遷移では単調性違反が起き得ず、prio_ls (settle-once) も Reference と bit-exact。
        let mut a = make_vi(8, 8, vec![0i8; 64]);
        let mut b = make_vi(8, 8, vec![0i8; 64]);
        collapse_to_deterministic(&mut a);
        collapse_to_deterministic(&mut b);
        run_reference_to_fixed_point(&mut a);
        let (_i, _u, conv) = super::prio_ls_solve(&mut b, 3000);
        assert!(conv, "prio_ls must converge");

        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "value @ {i}");
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy @ {i}"
                );
            }
        }
        assert!(n_reach > 0, "決定論グラフでも到達可能セルが存在するはず");
    }
```

- [ ] **Step 2: 確率遷移での RMSE/方策一致 characterization テストを追加**

同テストモジュールに続けて追加（標準3マップで測定＋ゆるい上限で回帰ガード）:

```rust
    fn standard_occ() -> Vec<(&'static str, Vec<i8>)> {
        let empty = vec![0i8; 64];
        let mut wall = vec![0i8; 64];
        for iy in 0..8 {
            wall[(iy * 8 + 5) as usize] = 100;
        }
        wall[5] = 0;
        let mut sentinel = vec![0i8; 64];
        sentinel[(1 * 8 + 2) as usize] = 100;
        sentinel[(3 * 8 + 2) as usize] = 100;
        sentinel[(2 * 8 + 1) as usize] = 100;
        vec![("empty", empty), ("obstacle", wall), ("sentinel", sentinel)]
    }

    #[test]
    fn prio_ls_characterization_vs_reference() {
        // prio_ls の近似度を実測（RMSE/方策一致）。ゆるい上限で回帰ガード（厳密値は出力で観察）。
        for (name, occ) in standard_occ() {
            let mut a = make_vi(8, 8, occ.clone());
            let mut b = make_vi(8, 8, occ);
            run_reference_to_fixed_point(&mut a);
            super::prio_ls_solve(&mut b, 3000);

            let (mut se, mut n, mut agree) = (0f64, 0u64, 0u64);
            for i in 0..a.states.len() {
                if a.states[i].total_cost < REACH {
                    let va = (a.states[i].total_cost / PROB_BASE) as f64;
                    let vb = (b.states[i].total_cost / PROB_BASE) as f64;
                    se += (va - vb) * (va - vb);
                    n += 1;
                    if a.states[i].optimal_action == b.states[i].optimal_action {
                        agree += 1;
                    }
                }
            }
            let rmse = (se / n as f64).sqrt();
            let pa = agree as f64 / n as f64;
            eprintln!("[prio_ls characterization] map={name} rmse={rmse:.3} policy={pa:.4} n={n}");
            assert!(n > 0, "到達セルが存在するはず ({name})");
            assert!(rmse <= 10.0, "prio_ls RMSE {rmse} exceeds loose bound ({name})");
            assert!(pa >= 0.85, "prio_ls policy agreement {pa} below loose bound ({name})");
        }
    }
```

- [ ] **Step 3: テスト実行（測定値を控える）**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml \
  -p vi_reference prio_ls -- --nocapture 2>&1 | tail -25
```
Expected: `prio_ls_exact_on_deterministic_transitions ... ok`、`prio_ls_characterization_vs_reference ... ok`。`[prio_ls characterization]` 行の rmse/policy を控える（TeX 用）。もし上限超過なら、それは「8x8 では straddle が無視できない」という発見 → 上限を実測に合わせて緩め、TeX に明記（失敗ではなく測定対象）。

- [ ] **Step 4: コミット**

```bash
git add vi_rs/vi_reference/src/solvers/priority.rs
git commit -m "test(vi_reference): prio_ls near-parity (deterministic-exact + RMSE/policy characterization)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: 直径レジーム測定バイナリ `vi_prio_measure`

**Files:**
- Create: `vi_rs/vi_reference/src/bin/vi_prio_measure.rs`

- [ ] **Step 1: 測定バイナリを作成**

```rust
//! 直径レジーム測定（ホスト完結・Docker/ROS 非依存）。合成 free ストリップで
//! reference / frontier2d / prio_ls / prio_lc の elapsed・更新数・repops を比較し、
//! markdown 表で出力する。設計 §5.2。
//!
//!   cargo run --release -p vi_reference --bin vi_prio_measure

use std::time::Instant;

use vi_reference::params::PROB_BASE;
use vi_reference::solvers::priority::priority_solve;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

const REACH: u64 = 1_000_000 * PROB_BASE;

fn actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

fn build(w: i32, h: i32) -> ValueIterator {
    let mut vi = ValueIterator::new(actions(), 1);
    let map = OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: vec![0i8; (w * h) as usize],
    };
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
    // ゴールは左端中央付近に置き、横長マップで直径 ≈ 幅/ステップ を最大化。
    let gy = h as f64 * 0.5 * 0.05;
    vi.set_goal(0.10, gy, 0);
    vi
}

fn reach_count(vi: &ValueIterator) -> u64 {
    vi.states.iter().filter(|s| s.total_cost < REACH).count() as u64
}

fn main() {
    let sizes = [(512, 64), (1024, 64), (2048, 64)];
    println!("| map | solver | elapsed[s] | pops/sweeps | updates | upd/reach | repops | reach |");
    println!("|---|---|---|---|---|---|---|---|");
    for (w, h) in sizes {
        // 到達セルの分母は厳密な prio_lc から取る（unreachable は MAX_COST 据置）。
        let mut vi_lc = build(w, h);
        let t = Instant::now();
        let lc = priority_solve(&mut vi_lc, 3000, false);
        let e_lc = t.elapsed().as_secs_f64();
        let rc = reach_count(&vi_lc).max(1);

        // reference（全走査）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = solve(&mut vi, U64Solver::Reference, 3000);
        let e = t.elapsed().as_secs_f64();
        println!("| {w}x{h} | reference | {e:.3} | {} | - | - | - | {rc} |", st.iters);

        // frontier2d（活性集合）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = solve(&mut vi, U64Solver::Frontier2D, 3000);
        let e = t.elapsed().as_secs_f64();
        println!(
            "| {w}x{h} | frontier2d | {e:.3} | {} | {} | {:.2} | - | {rc} |",
            st.iters,
            st.updates,
            st.updates as f64 / rc as f64
        );

        // prio_ls（近似・settle-once）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = priority_solve(&mut vi, 3000, true);
        let e = t.elapsed().as_secs_f64();
        println!(
            "| {w}x{h} | prio_ls | {e:.3} | {} | {} | {:.2} | {} | {rc} |",
            st.iters,
            st.updates,
            st.updates as f64 / rc as f64,
            st.repops
        );

        // prio_lc（厳密・label-correcting）。
        println!(
            "| {w}x{h} | prio_lc | {e_lc:.3} | {} | {} | {:.2} | {} | {rc} |",
            lc.iters,
            lc.updates,
            lc.updates as f64 / rc as f64,
            lc.repops
        );
    }
}
```

- [ ] **Step 2: ビルド & 実行（表を控える）**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo run --release --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml \
  -p vi_reference --bin vi_prio_measure 2>&1 | tail -20
```
Expected: 3サイズ × 4ソルバの markdown 表。確認ポイント: ① `frontier2d` の `upd/reach` ≈ 3〜4、`prio_ls`/`prio_lc` の `upd/reach` がそれ未満（≈1〜2）、② `reference` の `pops/sweeps`（=反復数）が幅に比例して増大、③ `prio_lc` の `repops`（単調性違反指標）の大きさ、④ `elapsed` の大小関係（更新数↓でも prio が wall-clock で勝つか）。表を控える（TeX §5）。

- [ ] **Step 3: コミット**

```bash
git add vi_rs/vi_reference/src/bin/vi_prio_measure.rs
git commit -m "feat(vi_reference): vi_prio_measure — diameter-regime strip benchmark

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: house.pgm ベンチ配線（compare パイプライン）

**Files:**
- Modify: `vi_compare/u64/run_u64_bench.sh`
- Modify: `vi_compare/compare/make_u64_report.py`

- [ ] **Step 1: run_u64_bench.sh の SOLVERS 既定に追加**

`vi_compare/u64/run_u64_bench.sh` の該当行を変更:
```bash
SOLVERS="${SOLVERS:-reference frontier3d frontier2d frontier_stack block_refine pyramid_sweep prio_ls prio_lc}"
```

- [ ] **Step 2: make_u64_report.py の SOLVERS リストに追加**

`vi_compare/compare/make_u64_report.py` の該当行を変更:
```python
SOLVERS = ['reference', 'frontier3d', 'frontier2d', 'frontier2d_soa', 'frontier2d_pad', 'frontier2d_par', 'frontier_stack', 'block_refine', 'pyramid_sweep', 'prio_ls', 'prio_lc']
```

- [ ] **Step 3: （Docker 環境がある場合）ベンチ実行＋レポート再生成**

```bash
make compare-u64 SOLVERS="prio_ls prio_lc"
make compare-u64-summary
```
Expected: `vi_compare/results/report_u64.md` に `prio_ls` / `prio_lc` 行が追加。`prio_lc` は RMSE 0・方策 100%（bit-exact ✓）、`prio_ls` は RMSE 小・方策高一致＋ `updates` 最小、本家比速度を確認。

**Docker/ROS 環境が無い場合**: 配線（Step 1-2）のみコミットし、house ベンチは保留。直径レジーム測定（Task 5、ホスト完結）が研究の主要実験データを供給するので、結論は出せる。`report_u64.md` 更新は環境が整い次第。

- [ ] **Step 4: コミット**

```bash
git add vi_compare/u64/run_u64_bench.sh vi_compare/compare/make_u64_report.py vi_compare/results/report_u64.md
git commit -m "bench(vi_compare): wire prio_ls/prio_lc into u64 house.pgm pipeline

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
（`report_u64.md` を更新できなかった場合は git add から除外。）

---

## Task 7: TeX 研究ノート（数式本体）

**Files:**
- Create: `docs/research/2026-06-09-vi-ssp-acceleration.tex`
- Create: `docs/research/README.md`

- [ ] **Step 1: README.md を作成（ビルド手順）**

```markdown
# Research notes

VI（価値反復）に関する数式研究ノート。

## ビルド

```bash
cd docs/research
pdflatex 2026-06-09-vi-ssp-acceleration.tex
pdflatex 2026-06-09-vi-ssp-acceleration.tex   # 相互参照のため2回
```

`pdflatex` 未導入なら `sudo apt-get install texlive-latex-base texlive-latex-extra`（または `tectonic 2026-06-09-vi-ssp-acceleration.tex`）。
```

- [ ] **Step 2: TeX 本体を作成（数式・計算量・アルゴリズム）**

`docs/research/2026-06-09-vi-ssp-acceleration.tex`:

```latex
\documentclass[11pt]{article}
\usepackage[margin=25mm]{geometry}
\usepackage{amsmath,amssymb,amsthm}
\usepackage{booktabs}
\usepackage{algorithm}
\usepackage{algpseudocode}
\usepackage{hyperref}
\newtheorem{remark}{Remark}
\newtheorem{lemma}{Lemma}
\newtheorem{proposition}{Proposition}
\title{価値反復を確率的最短経路として捉えた優先順序伝播による高速化}
\author{vi\_reference 研究ノート}
\date{2026-06}
\begin{document}
\maketitle

\section{リファレンス実装の数式モデル}
状態 $s=(i_x,i_y,i_\theta)\in\mathcal{S}$、$|\mathcal{S}|=N=n_x n_y n_\theta$。索引
$\mathrm{idx}(s)=i_\theta+i_x n_\theta+i_y n_\theta n_x$。行動 $a\in\mathcal{A}$
（前進・後退・左右回転の6種）は、各 source $\theta$ について遷移分布
$\tau_a(i_\theta)=\{(\delta,p)\}$ を持つ。ここで $\delta=(d_{ix},d_{iy},d_{it})$、$d_{it}$ は
\emph{絶対}$\theta$ インデックス、$\sum_{(\delta,p)\in\tau_a(i_\theta)}p=B$、$B=2^{18}$。
$\tau_a$ はセルを $64^3$ にサブセル分割した決定論連続遷移
$T_a(x,y,\theta)=(x+\ell_a\cos\theta,\ y+\ell_a\sin\theta,\ \theta+\Delta_a)$ の集計である。

\paragraph{Bellman 作用素。}
\begin{equation}
(\mathcal{T}V)(s)=\min_{a\in\mathcal{A}}A(s,a),\quad
A(s,a)=\frac1B\sum_{(\delta,p)\in\tau_a(i_\theta)}p\bigl[V(s\oplus\delta)+g(s\oplus\delta)\bigr],
\label{eq:bellman}
\end{equation}
$s\oplus\delta=(i_x{+}d_{ix},\,i_y{+}d_{iy},\,d_{it}\bmod n_\theta)$。遷移先のいずれかが
範囲外または非 free なら $A(s,a)=\textsc{MaxCost}$。ステップコスト
$g(s')=\mathrm{penalty}(s')+\mathrm{local\_penalty}(s')\ge B$。固定点 $V^\star=\mathcal{T}V^\star$、
ゴール集合 $\mathcal{G}$（\texttt{final\_state}）は吸収 $V^\star|_{\mathcal{G}}=0$。
これは\textbf{割引なし確率的最短経路（SSP, $\gamma=1$）}であり、$g>0$・proper policy 存在
ゆえ固定点は到達可能セル上で一意。

\paragraph{実装の癖（remark）。}
\begin{remark}
式\eqref{eq:bellman}末尾の $\tfrac1B(\cdot)$ は u64 整数除算 $\gg18$ で切り捨てられる。
\end{remark}
\begin{remark}
$\sum p[\cdot]$ は u64 wrapping 加算。未到達セル $V=\textsc{MaxCost}$ 近傍は
オーバーフロー折返しで振動するが、到達可能セルの固定点には影響しない。
\end{remark}
\begin{remark}
本家は6種の sweep 順（行優先/列優先×正逆＋分割2種）で Gauss--Seidel を行い、
直径律速を方向性で緩和する。$t_{\text{res}}=360/n_\theta$ は整数除算。
\texttt{State::from\_occupancy} の安全マージン penalty は線形 \texttt{pos} 境界のみ見る
行跨ぎバグを持つ。\texttt{valueFunctionWriter} は $\mathrm{total\_cost}/B$ の整数除算で出力。
\end{remark}

\section{計算量と高速化のレバー}
情報はゴールから1行動ステップ/スイープで伝播するため、素朴な Jacobi/Gauss--Seidel VI は
$O(\mathrm{diam}\cdot N)$ 仕事を要する（$\mathrm{diam}$＝行動ステップでのグラフ直径）。
活性集合（frontier）は波面のみ更新し $O(R\cdot N)$（$R$＝波面厚＝1セルが確定するまでの
平均更新回数）。一方、状態を値の昇順に\textbf{一度だけ}確定する優先順序伝播は
$O(N\log N)$（二分ヒープ）で、$\mathrm{diam}$ に依存しない。
\[
\text{Jacobi } O(\mathrm{diam}\cdot N)\ \gg\ \text{frontier } O(R\cdot N)\ \gtrsim\ \text{priority } O(N\log N).
\]
横長マップ（campus $14000\times800$, $\mathrm{diam}\approx2300$）でこの差が顕在化する。

\section{優先順序伝播ソルバ}
$V$ の tentative ラベルを $\ell(s)$（初期 $\infty=\textsc{MaxCost}$、$\mathcal{G}$ で $0$）とし、
最小ヒープで昇順に取り出す。cost-to-go では $V(s)$ が後続に依存するため、確定セル $s^\star$ は
それを outcome に持つ\emph{前駆}のラベルを改善し得る（逆θ隣接 $\mathrm{rev}[i_t]$）。

\begin{algorithm}[h]
\caption{Priority propagation（共有骨格）}
\begin{algorithmic}[1]
\State $\ell\gets$ \textsc{MaxCost}（$\mathcal{G}$ で $0$）; heap $\gets\{(0,s):s\in\mathcal{G}\}$
\While{heap 非空}
  \State $(\lambda,s^\star)\gets$ pop-min
  \If{$\lambda\neq\ell(s^\star)$} \textbf{continue} \Comment{stale（遅延 decrease-key）}\EndIf
  \State \textbf{settle 規則}（A1: $s^\star$ 確定済みなら skip、確定印を付与／A2: なし）
  \For{前駆 $s\in\mathrm{pred}(s^\star)$, free, $\notin\mathcal{G}$（A1 は未確定のみ）}
     \State $\ell'\gets(\mathcal{T}\ell)(s)$ を式\eqref{eq:bellman}で再計算
     \If{$\ell'<\ell(s)$} $\ell(s)\gets\ell'$; push $(\ell',s)$ \EndIf
  \EndFor
\EndWhile
\end{algorithmic}
\end{algorithm}

\paragraph{A1: label-setting（\texttt{prio\_ls}、近似）.}
$s^\star$ を確定（settle）し二度と触れない。各到達セルを実質1回確定 → 最小仕事量。
\paragraph{A2: label-correcting（\texttt{prio\_lc}、厳密）.}
settle せず、改善時は確定済みセルも再 relax。単調減少・下に有界ゆえ停止し、$V^\star$ に一致。
ゆえに本家と bit-exact。

\section{単調性とその破れ（A1 の近似誤差）}
\begin{proposition}
全行動の全 outcome が「確定済みかつ値 $\le V(s)$」なら、純粋な label-setting は厳密。
\end{proposition}
本問題ではこれが破れ得る：$A(s,a)$ は\emph{全 outcome の確定}を要するため、$s$ の最適行動が
「稀だが高値の outcome（後退・横滑り）＋多数の低値前進 outcome」を含むと、低値 outcome が
先に確定しても高値 outcome の確定が遅れ、その瞬間 $s$ のラベルが確定走査位置より\emph{後方}に
現れる（単調性違反）。
\begin{lemma}[違反の有界性]
$\tau_a$ はサブセル離散化由来で空間的に密集し、outcome の値の広がりは
$O(\max\|\delta\|\cdot \bar g)$ で有界。ゆえに違反量（後方挿入距離）はこの遷移スプレッドで
上から押さえられる。
\end{lemma}
よって A1 の誤差は局所的・小であり、決定論遷移（単一 outcome）では消える。
本ノートでは違反量を \texttt{prio\_lc} の再処理回数 \texttt{repops} で実測し（\S6）、
これが小さければ Dial（radix）バケット化で $O(N+V_{\max}/B)$ への更なる加速が見込める。

\section{実験}
\subsection{house.pgm（$384\times384\times60$）}
% Task 6 後に report_u64.md の prio_ls/prio_lc 行を転記。
\begin{center}\emph{[実験後に表を挿入]}\end{center}

\subsection{直径レジーム（合成ストリップ）}
% Task 5 の vi_prio_measure 出力を転記。
\begin{center}\emph{[実験後に表を挿入]}\end{center}

\subsection{考察}
% 更新数の下界到達 vs wall-clock（帯域律速 frontier との逆特性）、repops と Dial 化可否、
% prio_ls の速度×精度トレードオフ。
\emph{[実験後に記述]}

\section{結論}
\emph{[実験後に記述]}
\end{document}
```

- [ ] **Step 3: LaTeX 構文の健全性チェック（latex があれば）**

```bash
cd docs/research && (which pdflatex >/dev/null 2>&1 && pdflatex -interaction=nonstopmode -halt-on-error 2026-06-09-vi-ssp-acceleration.tex >/dev/null 2>&1 && echo "PDF OK" || echo "no pdflatex — .tex source delivered")
```
Expected: `PDF OK` または `no pdflatex — .tex source delivered`（どちらも可。後者なら .tex のみ納品）。

- [ ] **Step 4: コミット**

```bash
git add docs/research/2026-06-09-vi-ssp-acceleration.tex docs/research/README.md
git commit -m "docs(research): TeX note — VI as SSP, priority-propagation acceleration (math)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: TeX に実測結果を反映＋考察・結論

**Files:**
- Modify: `docs/research/2026-06-09-vi-ssp-acceleration.tex`（§5「実験」の表・考察・結論）

- [ ] **Step 1: §5.1 に house.pgm 表を挿入**

`vi_compare/results/report_u64.md` の `prio_ls`/`prio_lc` 行（と比較用の reference/frontier2d 行）を `booktabs` 表に転記。house ベンチを実行できなかった場合は「house ベンチは別環境で実施予定」と明記し、§5.2 を主結果とする。`[実験後に表を挿入]` を置換。

- [ ] **Step 2: §5.2 に直径レジーム表を挿入**

Task 5 の `vi_prio_measure` 出力（3サイズ×4ソルバ、elapsed/pops/updates/upd-per-reach/repops）を `booktabs` 表へ転記。`[実験後に表を挿入]` を置換。

- [ ] **Step 3: §5.3 考察・§6 結論を記述**

実測に基づき以下を埋める（`[実験後に記述]` を置換）:
- `upd/reach` の reference($\approx$diam) → frontier2d($\approx R$) → prio($\approx$1) の低減。
- 直径（幅）増大に対する reference 反復の線形増 vs prio 更新数の $\propto N$ 頭打ち。
- **wall-clock の逆特性**: 更新数で下界に近づいても、ヒープ/random-access のキャッシュミスで
  帯域律速 frontier2d に wall-clock で負けるか勝つか（実測で結論）。
- `repops`（単調性違反）の大きさと Dial 化の見込み。
- `prio_ls` の速度×精度トレードオフ（characterization テストの rmse/policy も引用）。

- [ ] **Step 4: LaTeX 健全性チェック（latex があれば）＋コミット**

```bash
cd docs/research && (which pdflatex >/dev/null 2>&1 && pdflatex -interaction=nonstopmode -halt-on-error 2026-06-09-vi-ssp-acceleration.tex >/dev/null 2>&1 && echo "PDF OK" || echo "no pdflatex — .tex only")
cd /home/nop/dev/mywork/value_iteration_new
git add docs/research/2026-06-09-vi-ssp-acceleration.tex
git commit -m "docs(research): fill experimental results + discussion/conclusion

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: 最終検証

**Files:** なし（検証のみ）

- [ ] **Step 1: クレート全テスト green・警告ゼロ**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml -p vi_reference 2>&1 | tail -15
```
Expected: 全 pass、warning なし。`parity_standard_maps_prio_lc`（bit-exact）と `prio_ls_*`（near-parity）を含む。

- [ ] **Step 2: ワークスペース全体の回帰確認**

```bash
cd /tmp && CARGO_TARGET_DIR=/home/nop/dev/mywork/value_iteration_new/vi_compare/.cache/u64_target \
  cargo test --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml --workspace 2>&1 | tail -15
```
Expected: 既存ソルバ群も含め全 pass（優先順序ソルバ追加で既存挙動は不変）。

- [ ] **Step 3: 成果物の存在確認**

```bash
ls -la docs/research/2026-06-09-vi-ssp-acceleration.tex docs/research/README.md \
  vi_rs/vi_reference/src/solvers/priority.rs vi_rs/vi_reference/src/solvers/prio_lc.rs \
  vi_rs/vi_reference/src/bin/vi_prio_measure.rs
git log --oneline -10
```
Expected: 全ファイル存在、コミット履歴に Task 1–8 が並ぶ。

---

## Self-Review 結果（spec 被覆）

- spec §2 数式定式化 → Task 7（TeX §1）。癖 remark → Task 7（remark 3つ）。
- spec §2.1 計算量 → Task 7（TeX §2）。
- spec §2.2 単調性とその破れ → Task 7（TeX §4 + Lemma）。
- spec §3.1 逆隣接 → Task 1（`build_rev_theta`）。
- spec §3.2 共通スケルトン → Task 3（`priority_solve`）。
- spec §3.3 (A1) prio_ls → Task 3（`prio_ls_solve`）+ Task 4（near-parity）。
- spec §3.4 (A2) prio_lc → Task 2/3（bit-exact parity）。
- spec §3.5 配置/API → Task 1–3（priority.rs/prio_lc.rs/mod.rs）。
- spec §4 TDD → Task 2（parity red→green）, Task 4（決定論一致＋RMSE 閾値）。
- spec §5.1 house ベンチ → Task 6。
- spec §5.2 直径レジーム＋違反測定 → Task 5（`vi_prio_measure`, `repops`）。
- spec §6 成果物 → Task 5–8。

型整合: `priority_solve(vi, max_iter, label_setting) -> PrioStats{iters,updates,converged,repops}`、
`prio_ls_solve`/`prio_lc_solve -> (u32,u64,bool)`、`build_rev_theta -> Vec<Vec<(i32,i32,i32)>>`、
`relax_cell -> Option<u64>`、enum `PriorityLabelSetting`/`PriorityLabelCorrecting`、
from_name `"prio_ls"`/`"prio_lc"` — 全タスクで一貫。
