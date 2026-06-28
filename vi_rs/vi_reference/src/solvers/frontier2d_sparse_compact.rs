//! B5: `frontier2d_sparse` のアウトオブコア版 (`compact`)。メモリ制約下で巨大マップを解く。
//!
//! 設計（2026-06-27 合意・spec 省略・直接実装）:
//! - **健全性 = 値ウォーターマーク finalization**: 値を昇順で前進させ、しきい値 `T` 未満を毎波
//!   Gauss–Seidel で収束させる。`watermark = T − Δ_band` 以下に収束したセルは、その直接依存
//!   （窓内 outcome, 値は `v + 1セル分` 以内）が全て収束済み区間 `< T` に入るため v* 到達済み =
//!   以後不変 → final 化する。本家ダイナミクスは決定論 (`no_noise_state_transition`) + サブセル
//!   サンプリングなので outcome は隣接1セル幅のタイトクラスタ＝結合は浅く、`Δ_band` は控えめで
//!   足りる（parity テストで検証）。
//! - **メモリ = ブロックタイル + 値バンド + 遅延確保 + 退避**: パディング済みグリッドを行ブロックに
//!   分割。ブロックは frontier 到達時に states から**遅延確保**し、interior-final（自±halo が全
//!   final）になったら cp/pen/eval を**退避（解放）**。常駐ブロック ∝ 等高線×バンド（総面積でない）。
//!   出力(value/policy)は finalize 時に別配列へ確定保存し store から切り離す。
//!
//! 実装ステージ（TDD・parity 先行）: S1 ブロックストレージ。S2 finalization。S3 値バンド閾値
//! スキャン + ウォーターマーク finalization + ブロック退避。**S3b(本コミット)** 遅延確保で peak
//! RAM を band に抑える。後続: ディスク mmap → 並列化 → CLI。

use crate::params::MAX_COST;
use crate::state::State;
use crate::value_iterator::ValueIterator;

use super::frontier2d_fused::{action_cost_fused, CpCells, Geom, UNREACHED};
use super::{seed_frontier_2d, Bitboard2D};

/// 1 ブロック = この本数のパディング行。複数ブロックに跨る gather を刺激しつつ、退避粒度を保つ。
const BLOCK_ROWS: usize = 8;

/// 値バンド幅 `Δ_band = COUPLE_SAFETY · max(mx,my) · max_pen` の結合深さ安全係数。本家は決定論
/// ダイナミクス + サブセルサンプリングで outcome が隣接1セル幅 → 結合は浅いので小さめで足りる。
/// 値が小さすぎると bit-exact が壊れる（parity テストが検出）ので、テストで妥当性を担保する。
const COUPLE_SAFETY: u64 = 4;

/// 1 ブロック分の cp/pen/eval/fin。退避時に `BlockStore` 側で `None` に落として解放する。
struct Block {
    cp: Vec<u64>,
    pen: Vec<u64>,
    eval: Vec<bool>,
    /// final 化済みか。構造的 final（`!eval_ok`）は構築時 true。
    fin: Vec<bool>,
}

/// 行ブロック化＋遅延確保＋退避するストア。
///
/// パディング済みフラット座標（`Geom`、`cp[it + (ix+mx)·nt + (iy+my)·row_stride]`）を
/// `chunk = BLOCK_ROWS·row_stride` ごとの行ブロックへ分割。`blocks[b]` は frontier 到達時に
/// `ensure_block` で states から遅延構築し、interior-final になったら `evict_block` で `None` に
/// 落として解放する。ブロック単位 metadata（`n_eval`/`n_final`/`evicted`）は確保/退避をまたいで
/// 残すため並列 Vec で保持する。gather は flat 索引を `(block, offset)` に分解して読む。
pub(crate) struct BlockStore {
    blocks: Vec<Option<Block>>,
    /// ブロックごとの eval_ok セル数（構築前に states スキャンで確定）。
    n_eval: Vec<usize>,
    /// ブロックごとの final 化済み eval_ok セル数。`n_final == n_eval` で全 final。
    n_final: Vec<usize>,
    /// 退避済みフラグ（退避後も全 final 判定に使うため残す）。
    evicted: Vec<bool>,
    chunk: usize,
    n_pad: usize,
    // 遅延構築の座標復元用。
    row_stride: usize,
    nt: i32,
    nx: i32,
    ny: i32,
    mx: i32,
    my: i32,
    /// 全 free セルの最大ペナルティ（値バンド幅算出用）。
    max_pen: u64,
}

impl CpCells for BlockStore {
    #[inline(always)]
    fn get(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        debug_assert!(!self.evicted[b], "read from evicted block {b}");
        self.blocks[b].as_ref().expect("block not allocated").cp[i - b * self.chunk]
    }
}

impl BlockStore {
    /// states を 1 回スキャンして `n_eval`/`max_pen` だけ確定する（cp は materialize しない）。
    fn new(g: &Geom, states: &[State]) -> Self {
        let chunk = BLOCK_ROWS * g.row_stride as usize;
        let n_pad = g.n_pad();
        let nblk = n_pad.div_ceil(chunk);
        let mut n_eval = vec![0usize; nblk];
        let mut max_pen = 0u64;
        for s in states {
            let p = s.penalty.wrapping_add(s.local_penalty);
            if p > max_pen {
                max_pen = p;
            }
            if s.free && !s.final_state {
                let pad_idx = (s.it as i64 + g.pad_col(s.ix, s.iy)) as usize;
                n_eval[pad_idx / chunk] += 1;
            }
        }
        Self {
            blocks: (0..nblk).map(|_| None).collect(),
            n_final: vec![0; nblk],
            evicted: vec![false; nblk],
            n_eval,
            chunk,
            n_pad,
            row_stride: g.row_stride as usize,
            nt: g.nt,
            nx: g.nx,
            ny: g.ny,
            mx: g.mx,
            my: g.my,
            max_pen,
        }
    }

    /// ブロック `b` を states から遅延構築する（既に確保済み/退避済みなら no-op）。`Fused::build_direct`
    /// と同一規約: `pen = penalty +ʷ local_penalty`、`cp = total_cost +ʷ pen`（free かつ未到達でない
    /// とき。さもなくば UNREACHED）、`eval = free && !final_state`、構造的 `fin = !eval`。
    fn ensure_block(&mut self, b: usize, states: &[State]) {
        if self.blocks[b].is_some() || self.evicted[b] {
            return;
        }
        let start = b * self.chunk;
        let len = self.chunk.min(self.n_pad - start);
        let mut cp = vec![UNREACHED; len];
        let mut pen = vec![0u64; len];
        let mut eval = vec![false; len];
        let mut fin = vec![true; len]; // パディング/非 eval は構造的 final。
        let (nt, nx, ny, mx, my) = (self.nt, self.nx, self.ny, self.mx, self.my);
        for (o, (cpc, (penc, (evc, finc)))) in cp
            .iter_mut()
            .zip(pen.iter_mut().zip(eval.iter_mut().zip(fin.iter_mut())))
            .enumerate()
        {
            let pad_idx = start + o;
            let iy_pad = pad_idx / self.row_stride;
            let rem = pad_idx % self.row_stride;
            let ix_pad = rem / nt as usize;
            let it = rem % nt as usize;
            let ix = ix_pad as i32 - mx;
            let iy = iy_pad as i32 - my;
            if ix >= 0 && ix < nx && iy >= 0 && iy < ny {
                let orig =
                    it + ix as usize * nt as usize + iy as usize * nx as usize * nt as usize;
                let s = &states[orig];
                let p = s.penalty.wrapping_add(s.local_penalty);
                *penc = p;
                let is_eval = s.free && !s.final_state;
                *evc = is_eval;
                *finc = !is_eval;
                if s.free && s.total_cost != MAX_COST {
                    *cpc = s.total_cost.wrapping_add(p);
                }
            }
        }
        self.blocks[b] = Some(Block { cp, pen, eval, fin });
    }

    #[inline(always)]
    fn pen(&self, i: usize) -> u64 {
        let b = i / self.chunk;
        self.blocks[b].as_ref().expect("block not allocated").pen[i - b * self.chunk]
    }

    #[inline(always)]
    fn eval_ok(&self, i: usize) -> bool {
        let b = i / self.chunk;
        self.blocks[b].as_ref().expect("block not allocated").eval[i - b * self.chunk]
    }

    #[inline(always)]
    fn set_cp(&mut self, i: usize, v: u64) {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        self.blocks[b].as_mut().expect("block not allocated").cp[off] = v;
    }

    #[inline(always)]
    fn set_final(&mut self, i: usize) {
        let b = i / self.chunk;
        let off = i - b * self.chunk;
        let blk = self.blocks[b].as_mut().expect("block not allocated");
        if !blk.fin[off] {
            blk.fin[off] = true;
            let is_eval = blk.eval[off];
            if is_eval {
                self.n_final[b] += 1;
            }
        }
    }

    /// ブロック `b` の eval_ok セルが全て final 化済みか（退避済みも「全 final」のまま）。
    #[inline(always)]
    fn block_full(&self, b: usize) -> bool {
        self.n_final[b] == self.n_eval[b]
    }

    /// ブロック `b` を退避（cp/pen/eval/fin を解放）。metadata は近傍判定用に残す。
    fn evict_block(&mut self, b: usize) {
        self.blocks[b] = None;
        self.evicted[b] = true;
    }

    fn nblocks(&self) -> usize {
        self.blocks.len()
    }

    /// 現在常駐している（確保済み・未退避の）ブロック数。
    fn resident_blocks(&self) -> usize {
        self.blocks.iter().filter(|b| b.is_some()).count()
    }
}

/// 値バンド幅 `Δ_band`。`COUPLE_SAFETY · max(mx,my) · max_pen`。
fn couple_margin(g: &Geom, max_pen: u64) -> u64 {
    let r = (g.mx.max(g.my)).max(1) as u64;
    COUPLE_SAFETY.saturating_mul(r).saturating_mul(max_pen.max(1))
}

/// 列 (ix,iy) の窓（±my 行）に重なる行ブロックを全て確保する。relax/finalize で近傍 gather する前に
/// 呼び、退避済み以外の窓ブロックを常駐させる（退避済みは interior-final なので読まれない）。
fn ensure_window(store: &mut BlockStore, ix: i32, iy: i32, states: &[State]) {
    let _ = ix; // 行ブロックは全 x を含むので x 窓はブロック境界を跨がない。
    let nb = store.nblocks();
    // 列のセルはパディング行 iy+my、窓は ±my → パディング行 [iy, iy+2my]、ブロック = 行/BLOCK_ROWS。
    let b_lo = iy as usize / BLOCK_ROWS;
    let b_hi = (((iy + 2 * store.my) as usize) / BLOCK_ROWS).min(nb - 1);
    for b in b_lo..=b_hi {
        store.ensure_block(b, states);
    }
}

/// 列 (ix,iy) の到達済みセル（`cp != UNREACHED`）の値域 `(min, max)`。無ければ `(MAX, MAX)`。
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

/// 列 (ix,iy) の全 θ を 1 回 Bellman 更新する。窓ブロックを確保してから relax。
/// `(値を下げた θ があるか, 更新後 min, 更新後 max, 減少 θ 数)`。
fn relax_column(
    store: &mut BlockStore,
    g: &Geom,
    ix: i32,
    iy: i32,
    states: &[State],
) -> (bool, u64, u64, u64) {
    ensure_window(store, ix, iy, states);
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
/// 近傍が無く再計算できないため必須）、eval_ok セルを final 化してその数を返す。policy 算出時は
/// 近傍ブロックがまだ常駐（退避は finalize の後）。`out_*` は orig 索引 `it + ix·nt + iy·nt·nx`。
fn finalize_column(
    store: &mut BlockStore,
    g: &Geom,
    ix: i32,
    iy: i32,
    states: &[State],
    out_total: &mut [u64],
    out_action: &mut [i32],
) -> u64 {
    ensure_window(store, ix, iy, states);
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

/// 詳細統計（テスト/退避判定/ベンチで使う）。
pub struct CompactStats {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    /// final 化された eval_ok セル数。
    pub finalized: u64,
    /// 到達可能（eval_ok かつ `cp != UNREACHED`）セル数。
    pub reachable: u64,
    /// 退避後の常駐列数（＝非 final の到達列＝値バンド）のピーク。
    pub peak_resident_cols: u64,
    /// 到達した列数（値バンドの削減効果の比較基準）。
    pub reachable_cols: u64,
    /// 退避したブロック数。
    pub freed_blocks: u64,
    /// 常駐ブロック数（確保済み・未退避）のピーク。遅延確保＋退避が peak RAM を抑える指標。
    pub peak_resident_blocks: u64,
    /// 総ブロック数。
    pub total_blocks: u64,
}

/// セット済み `ValueIterator` をブロックタイル・アウトオブコアで解く。
/// `(iters, updates, converged)`。到達可能セルの収束値・方策は本家と bit-exact。
pub fn frontier2d_sparse_compact_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let s = solve_compact(vi, max_iter, None);
    (s.iters, s.updates, s.converged)
}

/// 本体（値バンド閾値スキャン + during-solve ウォーターマーク finalization + 遅延確保 + 退避、
/// 単一スレッド）。`band` 可変（`--mem-budget` の土台）。`band ≥ 結合深さ` なら本家と bit-exact。
pub fn solve_compact(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
) -> CompactStats {
    let g = Geom::build(vi);
    let (nx, ny) = (g.nx, g.ny);
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let ncol = (nx * ny) as usize;
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;

    // 確定出力（store から切り離す。退避後もここに残るので write_back はこちらから読む）。
    // orig 索引 `it + ix·nt + iy·nt·nx`。S4 でこれを mmap 化してディスクへ。
    let nstates = (nx * ny * g.nt) as usize;
    let mut out_total = vec![MAX_COST; nstates];
    let mut out_action = vec![-1i32; nstates];

    let halo_blocks = (g.my as usize).div_ceil(BLOCK_ROWS);

    let mut iters = 0u32;
    let mut total_updates = 0u64;
    let mut finalized = 0u64;
    let mut peak_resident_cols = 0u64;
    let mut peak_resident_blocks = 0u64;
    let mut freed_blocks = 0u64;

    // 初期フロンティア（ゴール列を依存窓で膨張）。vi の借用はここで終わる。
    let mut frontier: Vec<(u32, u32)> = seed_frontier_2d(vi).dilate(dx, dy).enumerate().collect();

    // states を借用して遅延構築のソースにする（write_back は loop 後なので競合しない）。
    let states: &[State] = &vi.states;
    let mut store = BlockStore::new(&g, states);
    let band = band_override.unwrap_or_else(|| couple_margin(&g, store.max_pen));

    // 列ごとの現在値域と final フラグ。初期値域は states スキャンで（ブロックを構築せず）作る。
    let mut col_min = vec![MAX_COST; ncol];
    let mut col_max = vec![MAX_COST; ncol];
    let mut col_final = vec![false; ncol];
    for s in states {
        if s.free && s.total_cost < MAX_COST {
            let i = cidx(s.ix, s.iy);
            let v = s.total_cost; // value = cp − pen = total_cost
            if col_min[i] == MAX_COST {
                col_min[i] = v;
                col_max[i] = v;
            } else {
                col_min[i] = col_min[i].min(v);
                col_max[i] = col_max[i].max(v);
            }
        }
    }

    let mut t = band;
    let converged = 'outer: loop {
        // ── 波: バンド [.., t) を収束。relax は隣接（未到達含む）を発見、伝播は in-band 列のみ。 ──
        loop {
            let mut changed = Bitboard2D::new(nx as u32, ny as u32);
            let mut any = false;
            for &(ixu, iyu) in &frontier {
                let (ix, iy) = (ixu as i32, iyu as i32);
                let i = cidx(ix, iy);
                if col_final[i] {
                    continue;
                }
                let (chg, mn, mx, ups) = relax_column(&mut store, &g, ix, iy, states);
                col_min[i] = mn;
                col_max[i] = mx;
                total_updates += ups;
                if chg {
                    any = true;
                }
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

        // ── ウォーターマーク finalize: col_max ≤ T − band ──
        let wm = t.saturating_sub(band);
        for iy in 0..ny {
            for ix in 0..nx {
                let i = cidx(ix, iy);
                if !col_final[i] && col_max[i] != MAX_COST && col_max[i] <= wm {
                    finalized += finalize_column(
                        &mut store,
                        &g,
                        ix,
                        iy,
                        states,
                        &mut out_total,
                        &mut out_action,
                    );
                    col_final[i] = true;
                }
            }
        }

        // ── 退避: interior-final（自 ± halo ブロックが全 final）のブロックを解放 ──
        let nb = store.nblocks();
        for b in 0..nb {
            if store.evicted[b] {
                continue;
            }
            let lo = b.saturating_sub(halo_blocks);
            let hi = (b + halo_blocks).min(nb - 1);
            if (lo..=hi).all(|k| store.block_full(k)) {
                // 確保済みなら解放（未確保＝touch されていないブロックは退避不要）。
                if store.blocks[b].is_some() {
                    store.evict_block(b);
                    freed_blocks += 1;
                } else {
                    store.evicted[b] = true; // touch されず全 final（=パディング/空）→ 退避済み扱い。
                }
            }
        }

        peak_resident_blocks = peak_resident_blocks.max(store.resident_blocks() as u64);
        let resident_cols = (0..ncol)
            .filter(|&i| !col_final[i] && col_min[i] != MAX_COST)
            .count() as u64;
        peak_resident_cols = peak_resident_cols.max(resident_cols);

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
    drop(store); // 残常駐ブロックも解放（出力は out_* に確定済み）。states 借用もここで終わる。

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
        peak_resident_blocks,
        total_blocks,
    }
}

/// 確定出力配列（finalize 時に保存済み）から states へ値・方策を書き戻す。退避でブロックが解放
/// されていても出力は残るので store には触れない。orig 索引 `it + ix·nt + iy·nt·nx`。
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

    /// parity ゲート: 標準3マップ (empty/obstacle/sentinel) で reference 固定点と bit-exact。
    #[test]
    fn parity_standard_maps_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            frontier2d_sparse_compact_solve(vi, 4000)
        });
    }

    /// 多数の行ブロックに跨る大きめ空マップで遅延 gather + 値バンドを刺激する。
    #[test]
    fn parity_larger_empty_compact() {
        crate::solvers::test_support::assert_parity(32, 24, vec![0i8; 32 * 24], |vi| {
            frontier2d_sparse_compact_solve(vi, 4000)
        });
    }

    /// 収束後、到達可能な eval_ok セルは全て final 化されるべき。
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

    /// 小さい値バンドでも bit-exact を保ちつつ、退避後の常駐列を到達列より小さく抑える。
    #[test]
    fn value_band_bounds_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (64, 8);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let band = 3 * PROB_BASE;
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

    /// 遅延確保＋退避が peak RAM（常駐ブロック）を band に抑え、かつ出力経由で bit-exact。
    #[test]
    fn lazy_alloc_and_eviction_bound_peak_and_exact() {
        use crate::params::PROB_BASE;
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        // 縦長マップ＋小バンドで、band がブロックの一部だけを覆い peak < total を顕在化させる。
        let (w, h) = (8, 128);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let stats = solve_compact(&mut b, 20000, Some(PROB_BASE));
        assert!(stats.converged, "must converge");

        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH && a.states[i].total_cost != b.states[i].total_cost {
                mism += 1;
            }
        }
        assert_eq!(mism, 0, "bit-exact mismatch after lazy alloc + eviction");
        assert!(stats.freed_blocks > 0, "ブロック退避が起きるべき");
        // 遅延確保＋退避で常駐ブロックのピークが総ブロックを下回る（= peak RAM 削減）。
        assert!(
            stats.peak_resident_blocks < stats.total_blocks,
            "peak 常駐ブロックが総ブロックを下回るべき (peak={}, total={})",
            stats.peak_resident_blocks, stats.total_blocks
        );
    }
}
