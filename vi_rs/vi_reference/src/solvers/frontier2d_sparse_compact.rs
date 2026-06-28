//! B5: `frontier2d_sparse` のアウトオブコア版 (`compact`)。メモリ制約下で巨大マップを解く。
//!
//! 設計（2026-06-27 合意・spec 省略・直接実装）:
//! - **健全性 = 値ウォーターマーク finalization**: 値を昇順で前進させ、しきい値 `T` 未満を毎波
//!   Gauss–Seidel で収束させる。`watermark = T − Δ_band` 以下に収束したセルは、その直接依存
//!   （窓内 outcome, 値は `v + 1セル分` 以内）が全て収束済み区間 `< T` に入るため v* 到達済み =
//!   以後不変 → final 化する。本家ダイナミクスは決定論 (`no_noise_state_transition`) + サブセル
//!   サンプリングなので outcome は隣接1セル幅のタイトクラスタ＝結合は浅く、`Δ_band` は控えめで
//!   足りる（parity テストで検証）。
//! - **メモリ = ブロックタイル + 値バンド**: パディング済みグリッドを行ブロックに分割。活性なのは
//!   値バンド `[T−Δ_band, T)` の列のみ（＝等コスト等高線近傍）。final 化された列は退避可能（S3）。
//!   常駐 ∝ 等高線 × バンド幅。
//!
//! 実装ステージ（TDD・parity 先行）: **S1** ブロックストレージ + flat 等価 solve（済）。**S2**
//! finalization（収束後一括, 済）。**S3(本コミット)** 値バンド閾値スキャン + during-solve
//! ウォーターマーク finalization。後続: ブロック退避（RAM 解放）→ ディスク mmap → 並列化 → CLI。

use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

use super::frontier2d_fused::{action_cost_fused, CpCells, Fused, Geom, UNREACHED};
use super::{seed_frontier_2d, Bitboard2D};

/// S1 のブロック行数（1 ブロック = この本数のパディング行）。複数ブロックに跨る gather を
/// 必ず刺激できるよう小さめにとる。後続ステージで 2D タイル/遅延確保へ拡張する。
const BLOCK_ROWS: usize = 8;

/// 値バンド幅 `Δ_band = COUPLE_SAFETY · max(mx,my) · max_pen` の結合深さ安全係数。本家は決定論
/// ダイナミクス + サブセルサンプリングで outcome が隣接1セル幅 → 結合は浅いので小さめで足りる。
/// 値が小さすぎると bit-exact が壊れる（parity テストが検出）ので、テストで妥当性を担保する。
const COUPLE_SAFETY: u64 = 4;

/// 行ブロック化した cp/pen/eval/fin ストア（S1: 全ブロック常駐・単一スレッド）。
///
/// フラットなパディング配列（`Geom` 座標系、`cp[it + (ix+mx)·nt + (iy+my)·row_stride]`）を
/// `chunk = BLOCK_ROWS·row_stride` ごとの連続ブロックへ分割する。各ブロックは独立した `Vec` を
/// 所有し（＝退避時にブロック単位で解放できる土台）、gather は flat 索引を `(block, offset)` に
/// 分解して読む。S1 は全ブロック常駐なので、検証済みの `Fused::build_direct` でフラットに構築して
/// からブロックへ切り分ける（遅延確保は S3 で導入）。
pub(crate) struct BlockStore {
    blocks: Vec<Block>,
    /// 1 ブロックの u64 数 = `BLOCK_ROWS · row_stride`。
    chunk: usize,
    /// 全 free セルの最大ペナルティ（値バンド幅算出用）。
    max_pen: u64,
}

struct Block {
    cp: Vec<u64>,
    pen: Vec<u64>,
    eval: Vec<bool>,
    /// 値ウォーターマークで final 化済みか。初期値は構造的 final（`!eval_ok` = ゴール/障害物/
    /// パディング）。
    fin: Vec<bool>,
    /// このブロック内の eval_ok セル数 / うち final 化済み数。`n_final == n_eval` で全 final。
    n_eval: usize,
    n_final: usize,
    /// 退避済み（cp/pen/eval/fin を解放）か。interior-final（自＋±halo ブロック全 final）で退避。
    /// 退避後は誰も読まない（窓内の読み手＝近傍はすべて final＝非活性）ことが健全性の根拠。
    evicted: bool,
}

impl CpCells for BlockStore {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        // 退避済みブロックは誰も読まないはず（interior-final 不変条件）。debug で検出。
        debug_assert!(!self.blocks[b].evicted, "read from evicted block {b}");
        self.blocks[b].cp[i - b * self.chunk]
    }
}

impl BlockStore {
    fn build(vi: &ValueIterator, g: &Geom) -> Self {
        // 検証済みビルダでフラットに構築（S1 は全常駐なので妥当）。
        let f = Fused::build_direct(vi, g);
        let max_pen = f.pen.iter().copied().max().unwrap_or(0);
        let chunk = BLOCK_ROWS * g.row_stride as usize;
        let to_blocks_u64 = |v: Vec<u64>| -> Vec<Vec<u64>> {
            v.chunks(chunk).map(|s| s.to_vec()).collect()
        };
        let cp_b = to_blocks_u64(f.cp);
        let pen_b = to_blocks_u64(f.pen);
        let ev_b: Vec<Vec<bool>> = f.eval_ok.chunks(chunk).map(|s| s.to_vec()).collect();
        let blocks = cp_b
            .into_iter()
            .zip(pen_b)
            .zip(ev_b)
            .map(|((cp, pen), eval)| {
                // 構造的 final: eval_ok でない（ゴール=値ピン留め / 障害物=恒久 UNREACHED /
                // パディング）セルは最初から不変なので final。
                let fin: Vec<bool> = eval.iter().map(|&e| !e).collect();
                let n_eval = eval.iter().filter(|&&e| e).count();
                Block { cp, pen, eval, fin, n_eval, n_final: 0, evicted: false }
            })
            .collect();
        Self { blocks, chunk, max_pen }
    }

    #[inline(always)]
    fn pen(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        self.blocks[b].pen[i - b * self.chunk]
    }

    #[inline(always)]
    fn eval_ok(&self, i: usize) -> bool {
        let b = i / self.chunk;
        self.blocks[b].eval[i - b * self.chunk]
    }

    #[inline(always)]
    fn set_cp(&mut self, i: usize, v: u64) {
        let b = i / self.chunk;
        self.blocks[b].cp[i - b * self.chunk] = v;
    }

    #[inline(always)]
    fn set_final(&mut self, i: usize) {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        let blk = &mut self.blocks[b];
        if !blk.fin[off] {
            blk.fin[off] = true;
            if blk.eval[off] {
                blk.n_final += 1;
            }
        }
    }

    /// ブロック `b` の eval_ok セルが全て final 化済みか（退避済みも「全 final」のまま）。
    #[inline(always)]
    fn block_full(&self, b: usize) -> bool {
        let blk = &self.blocks[b];
        blk.n_final == blk.n_eval
    }

    /// ブロック `b` を退避（cp/pen/eval/fin を解放）。`n_eval`/`n_final` は近傍判定用に残す。
    fn evict_block(&mut self, b: usize) {
        let blk = &mut self.blocks[b];
        blk.cp = Vec::new();
        blk.pen = Vec::new();
        blk.eval = Vec::new();
        blk.fin = Vec::new();
        blk.evicted = true;
    }

    fn nblocks(&self) -> usize {
        self.blocks.len()
    }
}

/// 値バンド幅 `Δ_band`。`COUPLE_SAFETY · max(mx,my) · max_pen`。
fn couple_margin(g: &Geom, max_pen: u64) -> u64 {
    let r = (g.mx.max(g.my)).max(1) as u64;
    COUPLE_SAFETY.saturating_mul(r).saturating_mul(max_pen.max(1))
}

/// 列 (ix,iy) の到達済みセル（eval_ok / 非 eval を問わず `cp != UNREACHED`）の値域 `(min, max)`。
/// 到達セルが無ければ `(MAX_COST, MAX_COST)`。
fn column_range(store: &BlockStore, g: &Geom, ix: i32, iy: i32) -> (u64, u64) {
    let pad_col = g.pad_col(ix, iy);
    let (mut mn, mut mx) = (MAX_COST, MAX_COST);
    let mut any = false;
    for it in 0..g.nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp = store.get(pad_idx);
        if cp != UNREACHED {
            let v = cp.wrapping_sub(store.pen(pad_idx));
            if !any {
                mn = v;
                mx = v;
                any = true;
            } else {
                mn = mn.min(v);
                mx = mx.max(v);
            }
        }
    }
    (mn, mx)
}

/// 列 (ix,iy) の全 θ を 1 回 Bellman 更新する。`(値を下げた θ があるか, 更新後 min, 更新後 max,
/// 減少 θ 数)` を返す。値域は到達済みセル基準（`column_range` と同義）。
fn relax_column(store: &mut BlockStore, g: &Geom, ix: i32, iy: i32) -> (bool, u64, u64, u64) {
    let pad_col = g.pad_col(ix, iy);
    let mut changed = false;
    let mut ups = 0u64;
    for it in 0..g.nt {
        let pad_idx = (pad_col + it as i64) as usize;
        if !store.eval_ok(pad_idx) {
            continue;
        }
        let cp_self = store.get(pad_idx);
        let pen_self = store.pen(pad_idx);
        let before = if cp_self == UNREACHED {
            MAX_COST
        } else {
            cp_self.wrapping_sub(pen_self)
        };
        let mut min_cost = MAX_COST;
        for per_theta in g.precomp.iter() {
            let c = action_cost_fused(store, &per_theta[it as usize], pad_col);
            if c < min_cost {
                min_cost = c;
            }
        }
        if min_cost < before {
            store.set_cp(pad_idx, min_cost.wrapping_add(pen_self));
            ups += 1;
            changed = true;
        }
    }
    let (mn, mx) = column_range(store, g, ix, iy);
    (changed, mn, mx, ups)
}

/// 列 (ix,iy) を final 化する。到達済みセルの (value, policy) を出力配列へ確定保存し（退避後に
/// 近傍が無く再計算できないため、ここで確定が必須）、eval_ok セルを final 化してその数を返す。
/// `out_*` は orig 索引 `it + ix·nt + iy·nt·nx`。policy 算出時は近傍ブロックがまだ常駐。
fn finalize_column(
    store: &mut BlockStore,
    g: &Geom,
    ix: i32,
    iy: i32,
    out_total: &mut [u64],
    out_action: &mut [i32],
) -> u64 {
    let pad_col = g.pad_col(ix, iy);
    let (nt, nx) = (g.nt, g.nx);
    let mut cnt = 0u64;
    for it in 0..nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp = store.get(pad_idx);
        if cp == UNREACHED {
            continue; // 未到達: 出力は初期 (MAX_COST, -1) のまま。
        }
        let pen = store.pen(pad_idx);
        let orig = (it + ix * nt + iy * nt * nx) as usize;
        out_total[orig] = cp.wrapping_sub(pen);
        if store.eval_ok(pad_idx) {
            let mut min_cost = MAX_COST;
            let mut min_action = -1i32;
            for (ai, per_theta) in g.precomp.iter().enumerate() {
                let c = action_cost_fused(store, &per_theta[it as usize], pad_col);
                if c < min_cost {
                    min_cost = c;
                    min_action = ai as i32;
                }
            }
            out_action[orig] = min_action;
            store.set_final(pad_idx);
            cnt += 1;
        }
        // 非 eval（ゴール）: out_action は初期 -1（None）のまま、out_total はピン留め値。
    }
    cnt
}

/// 詳細統計（テスト/退避判定で使う）。
pub(crate) struct CompactStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    /// final 化された eval_ok（本来更新対象）セル数。
    pub finalized: u64,
    /// 到達可能（eval_ok かつ `cp != UNREACHED`）セル数。
    pub reachable: u64,
    /// 退避後の常駐列数（＝非 final の到達列＝値バンド）のピーク。メモリ上限の指標。
    pub peak_resident_cols: u64,
    /// 到達した列数（値バンドの削減効果の比較基準）。
    pub reachable_cols: u64,
    /// 退避したブロック数（cp/pen/eval/fin を解放した数）。
    pub freed_blocks: u64,
    /// 総ブロック数。
    pub total_blocks: u64,
}

/// セット済み `ValueIterator` をブロックタイル・アウトオブコアで解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は本家と bit-exact。
pub fn frontier2d_sparse_compact_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let s = solve_compact(vi, max_iter, None);
    (s.iters, s.updates, s.converged)
}

/// 本体（S3: 値バンド閾値スキャン + during-solve ウォーターマーク finalization、単一スレッド）。
///
/// 値しきい値 `T` を `band` 刻みで上げ、各波で「到達済み・非 final・`col_min < T`」の列を Gauss–
/// Seidel 収束させる。波の収束後、`col_max ≤ T − band` の列（＝全 θ がウォーターマーク以下に収束）
/// を final 化する。final 化セルの直接依存（窓内、値 `≤ v + Δ_band` 相当）は収束済み区間に入るため
/// v* 到達済み。`band ≥ 結合深さ` なら到達可能セルの収束値・方策は本家と bit-exact（parity 検証）。
pub(crate) fn solve_compact(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
) -> CompactStats {
    let g = Geom::build(vi);
    let (nx, ny) = (g.nx, g.ny);
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let ncol = (nx * ny) as usize;
    let mut store = BlockStore::build(vi, &g);
    // band は値バンド幅（= メモリ予算つまみの土台）。None なら結合深さの安全側 auto 値。
    let band = band_override.unwrap_or_else(|| couple_margin(&g, store.max_pen));

    // 列ごとの現在値域（到達済みセル基準）と final フラグ。
    let mut col_min = vec![MAX_COST; ncol];
    let mut col_max = vec![MAX_COST; ncol];
    let mut col_final = vec![false; ncol];
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;

    // 初期値域: ゴールセル（値ピン留め）を持つ列が `col_min=0` となり最初の活性種になる。
    for iy in 0..ny {
        for ix in 0..nx {
            let (mn, mx) = column_range(&store, &g, ix, iy);
            let i = cidx(ix, iy);
            col_min[i] = mn;
            col_max[i] = mx;
        }
    }

    // 確定出力（store から切り離す。退避後もここに残るので write_back はこちらから読む）。
    // orig 索引 `it + ix·nt + iy·nt·nx`。S4 でこれを mmap 化してディスクへ。
    let nstates = (nx * ny * g.nt) as usize;
    let mut out_total = vec![MAX_COST; nstates];
    let mut out_action = vec![-1i32; nstates];

    // 退避ハロ（行ブロック）。窓は ±my 行 → ±ceil(my/BLOCK_ROWS) ブロックの読み手を持つ。
    let halo_blocks = (g.my as usize).div_ceil(BLOCK_ROWS);

    let mut iters = 0u32;
    let mut total_updates = 0u64;
    let mut finalized = 0u64;
    let mut peak_resident_cols = 0u64;
    let mut freed_blocks = 0u64;

    // 初期フロンティア: ゴール列を依存窓で膨張（ゴール隣接の未到達列を発見・relax する種）。
    let mut frontier: Vec<(u32, u32)> = seed_frontier_2d(vi).dilate(dx, dy).enumerate().collect();

    let mut t = band;
    let converged = 'outer: loop {
        // ── 波: バンド [.., t) を収束。relax は隣接（未到達含む）を発見するが、伝播（次フロン
        //    ティアへの膨張）は in-band（col_min < t）の列からのみ。バンド上の列は 1 回 relax で
        //    値を発見して以後アイドル（t が上がるまで deferred）。 ──
        loop {
            let mut changed = Bitboard2D::new(nx as u32, ny as u32);
            let mut any = false;
            for &(ixu, iyu) in &frontier {
                let (ix, iy) = (ixu as i32, iyu as i32);
                let i = cidx(ix, iy);
                if col_final[i] {
                    continue;
                }
                let (chg, mn, mx, ups) = relax_column(&mut store, &g, ix, iy);
                col_min[i] = mn;
                col_max[i] = mx;
                total_updates += ups;
                if chg {
                    any = true;
                }
                // in-band 列からのみ伝播（バンド上の列はゲートして churn を防ぐ）。
                if mn != MAX_COST && mn < t {
                    changed.set(ixu, iyu);
                }
            }
            iters += 1;
            if iters >= max_iter {
                break 'outer false;
            }
            if !any {
                break;
            }
            frontier = changed
                .dilate(dx, dy)
                .enumerate()
                .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
                .collect();
        }

        // ── ウォーターマーク finalize: col_max ≤ T − band（全 θ がウォーターマーク以下に収束） ──
        let wm = t.saturating_sub(band);
        for iy in 0..ny {
            for ix in 0..nx {
                let i = cidx(ix, iy);
                if !col_final[i] && col_max[i] != MAX_COST && col_max[i] <= wm {
                    finalized +=
                        finalize_column(&mut store, &g, ix, iy, &mut out_total, &mut out_action);
                    col_final[i] = true;
                }
            }
        }

        // ── 退避: interior-final（自 ± halo ブロックが全 final）のブロックを解放 ──
        let nb = store.nblocks();
        for b in 0..nb {
            if store.blocks[b].evicted {
                continue;
            }
            let lo = b.saturating_sub(halo_blocks);
            let hi = (b + halo_blocks).min(nb - 1);
            if (lo..=hi).all(|k| store.block_full(k)) {
                store.evict_block(b);
                freed_blocks += 1;
            }
        }

        // 退避後の常駐＝非 final の到達列（＝値バンド）。そのピークがメモリ上限の指標。
        let resident = (0..ncol)
            .filter(|&i| !col_final[i] && col_min[i] != MAX_COST)
            .count() as u64;
        peak_resident_cols = peak_resident_cols.max(resident);

        // ── 終了判定: 到達済み非 final 列が残っていない ──
        let mut remaining = false;
        for i in 0..ncol {
            if !col_final[i] && col_min[i] != MAX_COST {
                remaining = true;
                break;
            }
        }
        if !remaining {
            break true;
        }

        // T を次バンドへ進め、deferred（到達済み・非 final・col_min < 新 T）を膨張して再活性。
        t = t.saturating_add(band);
        let mut react = Bitboard2D::new(nx as u32, ny as u32);
        for iy in 0..ny {
            for ix in 0..nx {
                let i = cidx(ix, iy);
                if !col_final[i] && col_min[i] != MAX_COST && col_min[i] < t {
                    react.set(ix as u32, iy as u32);
                }
            }
        }
        frontier = react
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    };

    let total_blocks = store.nblocks() as u64;
    drop(store); // 退避済みでないブロックもここで全解放（出力は out_* に確定済み）。

    write_back_output(vi, &g, &out_total, &out_action);

    let reachable = vi
        .states
        .iter()
        .filter(|s| s.free && !s.final_state && s.total_cost < MAX_COST)
        .count() as u64;
    let reachable_cols = (0..ncol).filter(|&i| col_max[i] != MAX_COST).count() as u64;

    CompactStats {
        iters,
        updates: total_updates,
        converged,
        finalized,
        reachable,
        peak_resident_cols,
        reachable_cols,
        freed_blocks,
        total_blocks,
    }
}

/// 確定出力配列（finalize 時に保存済み）から states へ値・方策を書き戻す。退避でブロックが解放
/// されていても出力は残るので、store には触れない。orig 索引 `it + ix·nt + iy·nt·nx`。
fn write_back_output(vi: &mut ValueIterator, g: &Geom, out_total: &[u64], out_action: &[i32]) {
    let (nt, nx) = (g.nt, g.nx);
    for s in vi.states.iter_mut() {
        let orig = (s.it + s.ix * nt + s.iy * nt * nx) as usize;
        s.total_cost = out_total[orig];
        s.optimal_action = if out_action[orig] < 0 {
            None
        } else {
            Some(out_action[orig] as usize)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// S1/S3 parity ゲート: 標準3マップ (empty/obstacle/sentinel) で reference 固定点と bit-exact。
    /// 値バンド finalization（band ≥ 結合深さ）が収束値・方策を壊さないことを担保。
    #[test]
    fn parity_standard_maps_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            frontier2d_sparse_compact_solve(vi, 4000)
        });
    }

    /// 多数の行ブロックに跨る大きめ空マップで paged gather + 値バンドを刺激する。
    #[test]
    fn parity_larger_empty_compact() {
        crate::solvers::test_support::assert_parity(32, 24, vec![0i8; 32 * 24], |vi| {
            frontier2d_sparse_compact_solve(vi, 4000)
        });
    }

    /// S2: 収束後、到達可能な eval_ok セルは全て final 化されるべき。
    #[test]
    fn finalizes_all_reachable_cells() {
        let mut vi = crate::solvers::test_support::make_vi(8, 8, vec![0i8; 64]);
        let stats = solve_compact(&mut vi, 4000, None);
        assert!(stats.converged, "solver must converge");
        assert!(stats.reachable > 0, "到達可能セルが存在するはず");
        assert_eq!(
            stats.finalized, stats.reachable,
            "全到達可能セルが final 化されるべき (finalized={}, reachable={})",
            stats.finalized, stats.reachable
        );
    }

    /// S3: 小さい値バンドでも bit-exact を保ちつつ、退避後の常駐列を到達列より小さく抑える。
    /// 横長マップ（値が大きく広がる）で reference 固定点と一致＋常駐 < 到達を確認。
    #[test]
    fn value_band_bounds_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (64, 8);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let band = 3 * PROB_BASE; // 小バンド（結合深さは浅い前提）。
        let stats = solve_compact(&mut b, 8000, Some(band));
        assert!(stats.converged, "must converge");

        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH && a.states[i].total_cost != b.states[i].total_cost {
                mism += 1;
            }
        }
        assert_eq!(mism, 0, "bit-exact mismatch (band={band})");
        assert!(
            stats.peak_resident_cols < stats.reachable_cols,
            "band should bound resident (peak={}, reachable_cols={}, band={})",
            stats.peak_resident_cols, stats.reachable_cols, band
        );
    }

    /// S3: finalize 列のブロックを退避（RAM 解放）しても、出力配列経由で bit-exact。退避が実際に
    /// 起きること（空振り防止）も確認。縦長マップ＋小バンドで行ブロックが下から順に退避する。
    #[test]
    fn evicts_finalized_blocks_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (8, 48);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let stats = solve_compact(&mut b, 8000, Some(3 * PROB_BASE));
        assert!(stats.converged, "must converge");

        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH && a.states[i].total_cost != b.states[i].total_cost {
                mism += 1;
            }
        }
        assert_eq!(mism, 0, "bit-exact mismatch after eviction");
        assert!(
            stats.freed_blocks > 0,
            "ブロック退避が起きるべき (freed={}, total={})",
            stats.freed_blocks, stats.total_blocks
        );
    }
}
