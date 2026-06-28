//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

// フロンティアには実績ある word 並列 Bitboard を再利用する（u16 frontier の高速化の源）。
// Bitboard は値の型に非依存なので u64 モデルでもそのまま使える。dilate は theta periodic。
pub(crate) use crate::bitboard::{Bitboard2D, Bitboard3D};

pub mod block;
pub mod coarse_theta;
pub mod frontier2d;
pub mod frontier2d_pad;
pub mod frontier2d_par;
pub mod frontier2d_fused;
pub mod frontier2d_sparse;
pub mod frontier2d_par_unsafe;
pub mod frontier2d_soa;
#[cfg(test)]
mod measure;
pub mod frontier3d;
pub mod pyramid;
pub mod stack;
pub mod stream;
pub mod tau;
pub mod topk;
pub mod priority;
pub mod prio_lc;

/// dilation 変位 `(mx, my, mt)` を `actions` の全遷移から算出する。`dit` は絶対 θ なので、
/// 各 (action, source theta `t`) について循環距離 `min(|dit-t|, nt-|dit-t|)` を取り `mt` とする。
/// これは「あるセルが変化したとき再評価が必要な前駆セル集合」の正しい上位集合を与える。
pub(crate) fn displacement(vi: &ValueIterator) -> (i32, i32, i32) {
    let nt = vi.cell_num_t;
    let (mut mx, mut my, mut mt) = (0i32, 0i32, 0i32);
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                mx = mx.max(st.dix.abs());
                my = my.max(st.diy.abs());
                let raw = (st.dit - t as i32).rem_euclid(nt);
                let circ = raw.min(nt - raw);
                mt = mt.max(circ);
            }
        }
    }
    (mx.max(1), my.max(1), mt)
}

/// 初期フロンティア種: `total_cost < MAX_COST` のセル（`set_goal` 後の `final_state` セル）。
pub(crate) fn seed_frontier(vi: &ValueIterator) -> Bitboard3D {
    let mut bb = Bitboard3D::new(vi.cell_num_x as u32, vi.cell_num_y as u32, vi.cell_num_t as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32, s.it as u32);
        }
    }
    bb
}

/// 初期フロンティア種 (2D): いずれかの θ で `total_cost < MAX_COST` の (ix,iy)。
pub(crate) fn seed_frontier_2d(vi: &ValueIterator) -> Bitboard2D {
    let mut bb = Bitboard2D::new(vi.cell_num_x as u32, vi.cell_num_y as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix as u32, s.iy as u32);
        }
    }
    bb
}

/// 3D フロンティア反復の共通ドライバ。frontier3d / tau / topk / coarse_theta が共有する
/// 「seed → (膨張 → 候補走査 → 減少セルを次フロンティアへ) を収束まで」という骨格を1箇所に
/// まとめる。差分は候補セルごとの処理 `update(vi, ix, iy, it)` のみ。
///
/// `update` は候補セル `(ix,iy,it)` を評価し、値を下げた（=次フロンティアへ伝播すべき）なら
/// `true` を返す。ドライバは `true` のセルだけを次フロンティアに入れ、`updates` を 1 加算する
/// （この「更新 ⟺ 伝播」は全 frontier3d 系ソルバで成り立つ不変条件）。
/// `(iters, updates, converged)` を返す（`converged` はフロンティアが空になったか）。
pub(crate) fn frontier3d_driver<F>(vi: &mut ValueIterator, max_iter: u32, mut update: F) -> (u32, u64, bool)
where
    F: FnMut(&mut ValueIterator, u32, u32, u32) -> bool,
{
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, mt) = displacement(vi);
    let (dx, dy, dt) = (mx as u32, my as u32, mt as u32);
    let mut frontier = seed_frontier(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy, dt);
        let mut new_frontier = Bitboard3D::new(nx as u32, ny as u32, nt as u32);
        for (ix, iy, it) in candidates.enumerate() {
            if update(vi, ix, iy, it) {
                updates += 1;
                new_frontier.set(ix, iy, it);
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, frontier.popcount() == 0)
}

/// 2D フロンティア反復の共通ドライバ。frontier2d / soa / pad が共有する「seed →
/// (空間膨張 → 候補 (ix,iy) ごとに全 θ 層を再評価 → 更新があれば次フロンティアへ) を収束まで」
/// の骨格をまとめる。差分は候補セルごとの処理 `cell(ix, iy)` のみ。
///
/// `cell` は候補セル `(ix,iy)` の全 θ 層を再評価し、**減少した θ 層の数**を返す（0 なら不変）。
/// ドライバは戻り値が 1 以上のセルだけを次フロンティアに入れ、その数を `updates` に加算する。
/// per-cell が読む状態 (vi / SoA 配列 / Padded) は呼び出し側がクロージャに閉じ込めるため、
/// ドライバ自身は `vi` を借用しない（seed / displacement は呼び出し側が事前計算して渡す）。
/// `(iters, updates, converged)` を返す。
pub(crate) fn frontier2d_driver<F>(
    nx: i32,
    ny: i32,
    seed: Bitboard2D,
    dx: u32,
    dy: u32,
    max_iter: u32,
    mut cell: F,
) -> (u32, u64, bool)
where
    F: FnMut(u32, u32) -> u64,
{
    let mut frontier = seed;
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(dx, dy);
        let mut new_frontier = Bitboard2D::new(nx as u32, ny as u32);
        for (ix, iy) in candidates.enumerate() {
            let u = cell(ix, iy);
            if u > 0 {
                updates += u;
                new_frontier.set(ix, iy);
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, frontier.popcount() == 0)
}

/// 収束後の最終 argmin パス（並列・読み取り専用）の共通実装。frontier2d_par /
/// frontier2d_fused（および各々を呼ぶ par_unsafe / sparse）が共有する「全 (ix,iy,it) を
/// 行バンド並列に走査し、free・非 final セルの optimal_action を argmin で確定する」骨格を
/// まとめる。差分は評価対象判定 `skip(pad_idx)` とアクションコスト `cost(buckets, pad_col)` の2点。
///
/// `pad_col = (ix+mx)·nt + (iy+my)·row_stride`（`Padded`/`Geom` の `pad_col` と同一式）。
/// `precomp[ai][it]` は `(action, source θ)` ごとの隣接 `(相対オフセット, prob)`。
/// 返り値はオリジナル座標 index の `Vec<Option<usize>>`。
#[allow(clippy::too_many_arguments)]
pub(crate) fn final_policy_parallel<S, C>(
    nx: i32,
    ny: i32,
    nt: i32,
    mx: i32,
    my: i32,
    row_stride: i64,
    precomp: &[Vec<Vec<(i64, u64)>>],
    nthreads: usize,
    skip: S,
    cost: C,
) -> Vec<Option<usize>>
where
    S: Fn(usize) -> bool + Sync,
    C: Fn(&[(i64, u64)], i64) -> u64 + Sync,
{
    let n = (nx * ny * nt) as usize;
    let rows: Vec<i32> = (0..ny).collect();
    let chunk = rows.len().div_ceil(nthreads).max(1);
    let skip = &skip;
    let cost = &cost;

    let parts: Vec<Vec<(usize, Option<usize>)>> = std::thread::scope(|scope| {
        let handles: Vec<_> = rows
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut out: Vec<(usize, Option<usize>)> = Vec::new();
                    for &iy in band {
                        for ix in 0..nx {
                            let pad_col =
                                (ix + mx) as i64 * nt as i64 + (iy + my) as i64 * row_stride;
                            let orig_col = (ix * nt + iy * (nt * nx)) as usize;
                            for it in 0..nt {
                                let pad_idx = (pad_col + it as i64) as usize;
                                if skip(pad_idx) {
                                    continue;
                                }
                                let mut min_cost = MAX_COST;
                                let mut min_action: Option<usize> = None;
                                for (ai, per_theta) in precomp.iter().enumerate() {
                                    let c = cost(&per_theta[it as usize], pad_col);
                                    if c < min_cost {
                                        min_cost = c;
                                        min_action = Some(ai);
                                    }
                                }
                                out.push((orig_col + it as usize, min_action));
                            }
                        }
                    }
                    out
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });

    let mut opt = vec![None; n];
    for part in parts {
        for (orig, action) in part {
            opt[orig] = action;
        }
    }
    opt
}

/// 到達可能とみなす total_cost 上限（compare.py の value>=1e6 境界と整合）。
pub(crate) const REACH_THRESH: u64 = 1_000_000u64 * crate::params::PROB_BASE;

/// u64 高速ソルバの種別。近似ソルバは no-op パラメータ（tau=0 / k=全 outcome / step=1）で
/// Frontier3D と等価（bit-exact）になり、移植の正しさを検証できる。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum U64Solver {
    Reference,
    Frontier3D,
    Frontier2D,
    Frontier2DSoA,
    Frontier2DPad,
    Frontier2DPar,
    Frontier2DParUnsafe,
    Frontier2DFused,
    Frontier2DSparse,
    FrontierStack,
    BlockRefine,
    PyramidSweep,
    Frontier3DTau { tau: u64 },
    Frontier3DTopK { k: u32 },
    Frontier3DCoarseTheta { step: u32 },
    StreamMimic,
    PriorityLabelSetting,
    PriorityLabelCorrecting,
}

impl U64Solver {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "reference" => U64Solver::Reference,
            "frontier3d" => U64Solver::Frontier3D,
            "frontier2d" => U64Solver::Frontier2D,
            "frontier2d_soa" => U64Solver::Frontier2DSoA,
            "frontier2d_pad" => U64Solver::Frontier2DPad,
            "frontier2d_par" => U64Solver::Frontier2DPar,
            "frontier2d_par_unsafe" => U64Solver::Frontier2DParUnsafe,
            "frontier2d_fused" => U64Solver::Frontier2DFused,
            "frontier2d_sparse" => U64Solver::Frontier2DSparse,
            "frontier_stack" => U64Solver::FrontierStack,
            "block_refine" => U64Solver::BlockRefine,
            "pyramid_sweep" => U64Solver::PyramidSweep,
            // 近似ソルバ: 既定は no-op（= Frontier3D 等価）。実用近似は param 指定で。
            "frontier3d_tau" => U64Solver::Frontier3DTau { tau: 0 },
            "frontier3d_topk" => U64Solver::Frontier3DTopK { k: u32::MAX },
            "frontier3d_coarse_theta" => U64Solver::Frontier3DCoarseTheta { step: 1 },
            "stream_mimic" => U64Solver::StreamMimic,
            "prio_ls" => U64Solver::PriorityLabelSetting,
            "prio_lc" => U64Solver::PriorityLabelCorrecting,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct U64SolveStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
}

/// Reference は全走査を strict 固定点（到達可能セルが不変）まで回す。
pub(crate) fn reference_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let mut iters = 0u32;
    let converged = loop {
        vi.value_iteration_worker(1, 0);
        iters += 1;
        let mut changed = false;
        for (i, s) in vi.states.iter().enumerate() {
            if s.total_cost < REACH_THRESH && s.total_cost != prev[i] {
                changed = true;
            }
            prev[i] = s.total_cost;
        }
        if !changed {
            break true;
        }
        if iters >= max_iter {
            break false;
        }
    };
    (iters, 0, converged)
}

/// セット済み `ValueIterator` を指定ソルバで収束まで解く。
pub fn solve(vi: &mut ValueIterator, solver: U64Solver, max_iter: u32) -> U64SolveStats {
    let (iters, updates, converged) = match solver {
        U64Solver::Reference => reference_solve(vi, max_iter),
        U64Solver::Frontier3D => frontier3d::frontier3d_solve(vi, max_iter),
        U64Solver::Frontier2D => frontier2d::frontier2d_solve(vi, max_iter),
        U64Solver::Frontier2DSoA => frontier2d_soa::frontier2d_soa_solve(vi, max_iter),
        U64Solver::Frontier2DPad => frontier2d_pad::frontier2d_pad_solve(vi, max_iter),
        U64Solver::Frontier2DPar => frontier2d_par::frontier2d_par_solve(vi, max_iter),
        U64Solver::Frontier2DFused => frontier2d_fused::frontier2d_fused_solve(vi, max_iter),
        U64Solver::Frontier2DSparse => frontier2d_sparse::frontier2d_sparse_solve(vi, max_iter),
        U64Solver::Frontier2DParUnsafe => {
            frontier2d_par_unsafe::frontier2d_par_unsafe_solve(vi, max_iter)
        }
        U64Solver::FrontierStack => stack::frontier_stack_solve(vi, max_iter),
        U64Solver::BlockRefine => block::block_refine_solve(vi, max_iter),
        U64Solver::PyramidSweep => pyramid::pyramid_sweep_solve(vi, max_iter),
        U64Solver::Frontier3DTau { tau } => tau::frontier3d_tau_solve(vi, tau, max_iter),
        U64Solver::Frontier3DTopK { k } => topk::frontier3d_topk_solve(vi, k, max_iter),
        U64Solver::Frontier3DCoarseTheta { step } => {
            coarse_theta::frontier3d_coarse_theta_solve(vi, step, max_iter)
        }
        U64Solver::StreamMimic => stream::stream_mimic_solve(vi, max_iter),
        U64Solver::PriorityLabelSetting => priority::prio_ls_solve(vi, max_iter),
        U64Solver::PriorityLabelCorrecting => prio_lc::prio_lc_solve(vi, max_iter),
    };
    U64SolveStats { iters, updates, converged }
}

/// フロンティア/ブロック系ソルバの parity テスト共有ヘルパ。
#[cfg(test)]
pub(crate) mod test_support {
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::params::PROB_BASE;
    use crate::value_iterator::ValueIterator;

    pub(crate) const REACH: u64 = 1_000_000u64 * PROB_BASE;

    pub(crate) fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }

    pub(crate) fn make_vi(w: i32, h: i32, occ: Vec<i8>) -> ValueIterator {
        let mut vi = ValueIterator::new(actions(), 1);
        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }

    /// Reference 全走査を strict 固定点（到達可能セルが変化しなくなる）まで回す。
    pub(crate) fn run_reference_to_fixed_point(vi: &mut ValueIterator) {
        let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        for _ in 0..2000 {
            vi.value_iteration_worker(1, 0);
            let mut changed = false;
            for (i, s) in vi.states.iter().enumerate() {
                if s.total_cost < REACH && s.total_cost != prev[i] {
                    changed = true;
                }
                prev[i] = s.total_cost;
            }
            if !changed {
                break;
            }
        }
    }

    /// `solve_fn` で解いた結果が Reference 固定点と到達可能セルで bit 一致することを assert。
    pub(crate) fn assert_parity<F>(w: i32, h: i32, occ: Vec<i8>, solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool),
    {
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);
        let (_i, _u, converged) = solve_fn(&mut b);
        assert!(converged, "solver must converge");
        let mut n_reach = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                n_reach += 1;
                assert_eq!(
                    a.states[i].total_cost, b.states[i].total_cost,
                    "total_cost mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i} (ix={},iy={},it={})",
                    a.states[i].ix, a.states[i].iy, a.states[i].it
                );
            }
        }
        assert!(n_reach > 0, "到達可能セルが存在するはず");
    }

    /// 標準の3マップ (empty / obstacle / sentinel) で parity を検証する共通テスト本体。
    pub(crate) fn parity_standard_maps<F>(solve_fn: F)
    where
        F: Fn(&mut ValueIterator) -> (u32, u64, bool) + Copy,
    {
        // empty 8x8
        assert_parity(8, 8, vec![0i8; 64], solve_fn);
        // obstacle: x=5 の縦壁 (隙間あり)
        let mut occ = vec![0i8; 64];
        for iy in 0..8 {
            occ[(iy * 8 + 5) as usize] = 100;
        }
        occ[5] = 0;
        assert_parity(8, 8, occ, solve_fn);
        // sentinel: goal(2,2) を3方向で囲む
        let mut occ = vec![0i8; 64];
        occ[(1 * 8 + 2) as usize] = 100;
        occ[(3 * 8 + 2) as usize] = 100;
        occ[(2 * 8 + 1) as usize] = 100;
        assert_parity(8, 8, occ, solve_fn);
    }
}

#[cfg(test)]
mod helper_tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::value_iterator::ValueIterator;

    fn small_vi() -> ValueIterator {
        let actions = vec![
            Action::new("forward", 0.3, 0.0, 0),
            Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2),
            Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4),
            Action::new("leftfw", 0.2, 20.0, 5),
        ];
        let mut vi = ValueIterator::new(actions, 1);
        let map = OccupancyGrid {
            width: 5,
            height: 5,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0i8; 25],
        };
        // theta_cell_num=60 (production と同じ)。粗いと goal の向き判定が成立しない。
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }

    #[test]
    fn displacement_is_bounded_and_positive() {
        let vi = small_vi();
        let (mx, my, mt) = displacement(&vi);
        assert!(mx >= 1 && my >= 1);
        assert!(mt >= 0 && mt < vi.cell_num_t);
    }

    #[test]
    fn seed_contains_goal_cells() {
        let vi = small_vi();
        let seed = seed_frontier(&vi);
        let n_final = vi.states.iter().filter(|s| s.total_cost < crate::params::MAX_COST).count();
        assert!(n_final > 0, "goal セルが存在するはず");
        assert_eq!(seed.popcount(), n_final as u64);
    }

    #[test]
    fn solve_reference_and_frontier3d_agree() {
        let mut a = small_vi();
        let mut b = small_vi();
        solve(&mut a, U64Solver::Reference, 2000);
        solve(&mut b, U64Solver::Frontier3D, 2000);
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH_THRESH {
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "@ {i}");
                assert_eq!(a.states[i].optimal_action, b.states[i].optimal_action, "@ {i}");
            }
        }
    }

    #[test]
    fn solver_from_str() {
        assert!(matches!(U64Solver::from_name("frontier3d"), Some(U64Solver::Frontier3D)));
        assert!(matches!(U64Solver::from_name("reference"), Some(U64Solver::Reference)));
        assert!(U64Solver::from_name("nope").is_none());
    }
}
