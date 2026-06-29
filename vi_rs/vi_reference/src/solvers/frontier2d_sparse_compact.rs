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
use super::frontier2d_par::n_threads;
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

/// 並列 Jacobi の 1 列分 compute 結果。`updates` は減少した θ の `(pad_idx, 新 cp)`。
/// `mn`/`mx` は更新後の列値域、`changed` は減少 θ があったか。
struct ColResult {
    ixu: u32,
    iyu: u32,
    updates: Vec<(usize, u64)>,
    mn: u64,
    mx: u64,
    changed: bool,
}

/// 列 (ix,iy) の全 θ を Bellman 更新するが**書き込まない**（read-only スナップショット読み）。
/// 並列フェーズで使う Jacobi 版 `relax_column`：隣接は更新前の値で評価され、減少分は `updates`
/// に集めて直列フェーズで一括 apply する。固定点が一意（更新順序非依存）なので最終値は G-S 版と
/// bit-exact。窓ブロックは呼び出し前に `ensure_window` で確保済みである前提。`mn`/`mx` は更新後の
/// 値域（`column_range` と同じく到達済み = `value != MAX_COST` のセルのみ集計）。
fn compute_column_jacobi(store: &BlockStore, g: &Geom, ix: i32, iy: i32) -> ColResult {
    let pad_col = g.pad_col(ix, iy);
    let mut updates: Vec<(usize, u64)> = Vec::new();
    let mut changed = false;
    let (mut mn, mut mx) = (MAX_COST, MAX_COST);
    let mut first = true;
    for it in 0..g.nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp_self = store.get(pad_idx);
        let pen_self = store.pen(pad_idx);
        let cur_v = if cp_self == UNREACHED {
            MAX_COST
        } else {
            cp_self.wrapping_sub(pen_self)
        };
        let new_v = if store.eval_ok(pad_idx) {
            let mut min_cost = MAX_COST;
            for per_theta in g.precomp.iter() {
                let c = action_cost_fused(store, &per_theta[it as usize], pad_col);
                if c < min_cost {
                    min_cost = c;
                }
            }
            if min_cost < cur_v {
                updates.push((pad_idx, min_cost.wrapping_add(pen_self)));
                changed = true;
                min_cost
            } else {
                cur_v
            }
        } else {
            cur_v // 非 eval（ゴール/障害）: 値は不変。
        };
        if new_v != MAX_COST {
            if first {
                mn = new_v;
                mx = new_v;
                first = false;
            } else {
                mn = mn.min(new_v);
                mx = mx.max(new_v);
            }
        }
    }
    ColResult { ixu: ix as u32, iyu: iy as u32, updates, mn, mx, changed }
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
    sink: &mut dyn CompactSink,
) -> u64 {
    ensure_window(store, ix, iy, states);
    let pad_col = g.pad_col(ix, iy);
    let (nt, nx) = (g.nt, g.nx);
    let ntu = nt as usize;
    // 列の全 θ を一旦バッファに作り、sink へ 1 回で書く（ディスク sink でも列連続 write になる）。
    let mut buf_v = vec![MAX_COST; ntu];
    let mut buf_a = vec![-1i32; ntu];
    let mut cnt = 0u64;
    for it in 0..nt {
        let pad_idx = (pad_col + it as i64) as usize;
        let cp = store.get(pad_idx);
        if cp == UNREACHED {
            continue; // 未到達: バッファは初期 (MAX_COST, -1) のまま。
        }
        let pen = store.pen(pad_idx);
        buf_v[it as usize] = cp.wrapping_sub(pen);
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
            buf_a[it as usize] = min_action;
            store.set_final(pad_idx);
            cnt += 1;
        }
        // 非 eval（ゴール）: action は -1（None）、value はピン留め値。
    }
    let base = (ix * nt + iy * nt * nx) as usize; // orig(it=0)
    sink.write_column(base, &buf_v, &buf_a);
    cnt
}

/// 確定出力（value, policy）の格納先を抽象化する sink。finalize 時に列単位で書き込み、
/// write_back 時に orig 単位で読む。RAM 実装（`RamSink`）と、呼び出し側のディスク mmap 実装
/// （`vi_bench::MmapSink`）を差し替えられる。これで出力の O(total) RAM をディスクへ外せる
/// （巨大マップ対応）。vi_reference は依存軽量なので mmap 実装はここには置かない。
pub trait CompactSink {
    /// 連続 orig 範囲 `[base, base+values.len())` へ value/action を書く（列の全 θ を 1 回で）。
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]);
    /// orig セルの (value, action) を読む。action<0 は None。
    fn read(&self, orig: usize) -> (u64, i32);
}

/// 既定の RAM 実装（2 本の `Vec`）。未書き込みセル（到達不能/パディング）は `(MAX_COST, -1)`。
pub struct RamSink {
    total: Vec<u64>,
    action: Vec<i32>,
}

impl RamSink {
    pub fn new(nstates: usize) -> Self {
        Self { total: vec![MAX_COST; nstates], action: vec![-1; nstates] }
    }
}

impl CompactSink for RamSink {
    #[inline]
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
        self.total[base..base + values.len()].copy_from_slice(values);
        self.action[base..base + actions.len()].copy_from_slice(actions);
    }
    #[inline]
    fn read(&self, orig: usize) -> (u64, i32) {
        (self.total[orig], self.action[orig])
    }
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

/// 波内バンドを直列 Gauss–Seidel で収束させる。frontier から始め、減少 θ があった in-band 列の
/// 依存窓を膨張して次フロンティアにし、丸ごと無変化になったら `true`。`max_iter` 到達で `false`。
/// store/col_min/col_max/reached/live/iters/total_updates を更新する（波ループ本体から切り出し）。
#[allow(clippy::too_many_arguments)]
fn converge_band_serial(
    store: &mut BlockStore,
    g: &Geom,
    states: &[State],
    nx: i32,
    ny: i32,
    dx: u32,
    dy: u32,
    t: u64,
    max_iter: u32,
    frontier: &mut Vec<(u32, u32)>,
    col_min: &mut [u64],
    col_max: &mut [u64],
    col_final: &[bool],
    reached: &mut [bool],
    live: &mut Vec<usize>,
    iters: &mut u32,
    total_updates: &mut u64,
) -> bool {
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;
    loop {
        let mut changed = Bitboard2D::new(nx as u32, ny as u32);
        let mut any = false;
        for &(ixu, iyu) in frontier.iter() {
            let (ix, iy) = (ixu as i32, iyu as i32);
            let i = cidx(ix, iy);
            if col_final[i] {
                continue;
            }
            let (chg, mn, mx, ups) = relax_column(store, g, ix, iy, states);
            col_min[i] = mn;
            col_max[i] = mx;
            if mn != MAX_COST && !reached[i] {
                reached[i] = true;
                live.push(i);
            }
            *total_updates += ups;
            if chg {
                any = true;
            }
            if mn != MAX_COST && mn < t {
                changed.set(ixu, iyu);
            }
        }
        *iters += 1;
        if *iters >= max_iter {
            return false;
        }
        if !any {
            return true;
        }
        *frontier = changed
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    }
}

/// 波内バンドを**決定的並列 Jacobi** で収束させる（`converge_band_serial` と同じ事後状態・収束値、
/// bit-exact）。`frontier2d_par` と同じ三相: ① 直列で frontier 全列の窓ブロックを `ensure_window`
/// 確保（compute は read-only なのでここで確保しておく）② 各ワーカーが `&store` を**読み取り専用**
/// で参照し担当列チャンクの新値を計算（Jacobi スナップショット読み = スレッド数・分割非依存）
/// ③ join 後に直列で cp を書き戻し col_min/col_max/live/次フロンティアを構築。固定点が一意なので
/// 並列でも最終値は G-S と bit-exact。unsafe 不使用（`&BlockStore` は内部可変性なしで `Sync`）。
#[allow(clippy::too_many_arguments)]
fn converge_band_parallel(
    store: &mut BlockStore,
    g: &Geom,
    states: &[State],
    nthreads: usize,
    nx: i32,
    ny: i32,
    dx: u32,
    dy: u32,
    t: u64,
    max_iter: u32,
    frontier: &mut Vec<(u32, u32)>,
    col_min: &mut [u64],
    col_max: &mut [u64],
    col_final: &[bool],
    reached: &mut [bool],
    live: &mut Vec<usize>,
    iters: &mut u32,
    total_updates: &mut u64,
) -> bool {
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;
    loop {
        // ① 直列: frontier 全列の窓ブロックを確保（並列 compute 中は store を書けないため）。
        for &(ixu, iyu) in frontier.iter() {
            ensure_window(store, ixu as i32, iyu as i32, states);
        }
        // ② 並列 compute（read-only &store・決定的 Jacobi）。各列は 1 チャンク = 1 スレッドが担当
        //    し pad_idx は列ごとに排他なので、書き戻し（③）に競合はない。
        let chunk = frontier.len().div_ceil(nthreads).max(1);
        let results: Vec<Vec<ColResult>> = {
            let store_ref: &BlockStore = store;
            std::thread::scope(|scope| {
                let handles: Vec<_> = frontier
                    .chunks(chunk)
                    .map(|part| {
                        scope.spawn(move || {
                            let mut out: Vec<ColResult> = Vec::new();
                            for &(ixu, iyu) in part {
                                let i = (iyu as i32 * nx + ixu as i32) as usize;
                                if col_final[i] {
                                    continue;
                                }
                                out.push(compute_column_jacobi(
                                    store_ref, g, ixu as i32, iyu as i32,
                                ));
                            }
                            out
                        })
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().unwrap()).collect()
            })
        };
        // ③ 直列 apply: cp 書き戻し + col_min/col_max/live/次フロンティア構築。
        let mut changed = Bitboard2D::new(nx as u32, ny as u32);
        let mut any = false;
        for part in &results {
            for r in part {
                let i = cidx(r.ixu as i32, r.iyu as i32);
                col_min[i] = r.mn;
                col_max[i] = r.mx;
                if r.mn != MAX_COST && !reached[i] {
                    reached[i] = true;
                    live.push(i);
                }
                for &(pad_idx, new_cp) in &r.updates {
                    store.set_cp(pad_idx, new_cp);
                    *total_updates += 1;
                }
                if r.changed {
                    any = true;
                }
                if r.mn != MAX_COST && r.mn < t {
                    changed.set(r.ixu, r.iyu);
                }
            }
        }
        *iters += 1;
        if *iters >= max_iter {
            return false;
        }
        if !any {
            return true;
        }
        *frontier = changed
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    }
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
    // 既定は RAM 出力（states 数 = nx·ny·nt）。
    let mut sink = RamSink::new(vi.states.len());
    solve_compact_into(vi, max_iter, band_override, &mut sink)
}

/// `solve_compact` の出力 sink 差し替え版。確定出力（value/policy）を `sink`（RAM or ディスク
/// mmap）へ書き、write_back も sink から読む。これで出力の O(total) RAM をディスクへ外せる。
/// スレッド数は `n_threads()`（環境変数 `VI_THREADS` で上書き可）。
pub fn solve_compact_into(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
) -> CompactStats {
    solve_compact_into_nthreads(vi, max_iter, band_override, sink, n_threads())
}

/// `solve_compact_into` のスレッド数明示版。`nthreads == 1` は直列 G-S、`>= 2` は波内 relax を
/// 決定的並列 Jacobi（`converge_band_parallel`）で回す。固定点は一意なので結果はスレッド数に依らず
/// bit-exact（テストが 1/4 両方を固定して検証する）。
pub(crate) fn solve_compact_into_nthreads(
    vi: &mut ValueIterator,
    max_iter: u32,
    band_override: Option<u64>,
    sink: &mut dyn CompactSink,
    nthreads: usize,
) -> CompactStats {
    let g = Geom::build(vi);
    let (nx, ny) = (g.nx, g.ny);
    let (dx, dy) = (g.mx as u32, g.my as u32);
    let ncol = (nx * ny) as usize;
    let cidx = |ix: i32, iy: i32| (iy * nx + ix) as usize;

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
    // `live` = 到達済み（`col_min != MAX`）かつ非 final の列インデックス集合。波ごとの
    // finalize/再活性/終了判定/常駐ピークは全列 O(ncol) 走査ではなく `live` の反復で行う
    // （バンド ≪ 全グリッドなので走査が O(band) になる）。`reached` で二重 push を防ぐ。
    let mut col_min = vec![MAX_COST; ncol];
    let mut col_max = vec![MAX_COST; ncol];
    let mut col_final = vec![false; ncol];
    let mut reached = vec![false; ncol];
    let mut live: Vec<usize> = Vec::new();
    for s in states {
        if s.free && s.total_cost < MAX_COST {
            let i = cidx(s.ix, s.iy);
            let v = s.total_cost; // value = cp − pen = total_cost
            if !reached[i] {
                reached[i] = true;
                live.push(i);
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
        // ── 波: バンド [.., t) を収束。relax は隣接（未到達含む）を発見、伝播は in-band 列のみ。
        //    nthreads==1 は直列 G-S、>=2 は並列 Jacobi。max_iter 到達なら 'outer を false で抜ける。 ──
        let band_ok = if nthreads >= 2 {
            converge_band_parallel(
                &mut store, &g, states, nthreads, nx, ny, dx, dy, t, max_iter, &mut frontier,
                &mut col_min, &mut col_max, &col_final, &mut reached, &mut live, &mut iters,
                &mut total_updates,
            )
        } else {
            converge_band_serial(
                &mut store, &g, states, nx, ny, dx, dy, t, max_iter, &mut frontier, &mut col_min,
                &mut col_max, &col_final, &mut reached, &mut live, &mut iters, &mut total_updates,
            )
        };
        if !band_ok {
            break 'outer false;
        }

        // ── ウォーターマーク finalize: col_max ≤ T − band。live のみ走査。 ──
        // finalize した列が属する行ブロック（列 (ix,iy) → ブロック (iy+my)/BLOCK_ROWS）を
        // 記録し、退避はその近傍ブロックだけ調べる（O(nblocks) 全走査を回避）。
        let wm = t.saturating_sub(band);
        let mut touched_blocks: Vec<usize> = Vec::new();
        for &i in &live {
            if col_max[i] <= wm {
                let ix = (i % nx as usize) as i32;
                let iy = (i / nx as usize) as i32;
                finalized += finalize_column(&mut store, &g, ix, iy, states, sink);
                col_final[i] = true;
                touched_blocks.push((iy + store.my) as usize / BLOCK_ROWS);
            }
        }
        live.retain(|&i| !col_final[i]);

        // ── 退避: finalize 列の近傍ブロック（±halo）のうち interior-final のものを解放。 ──
        // ブロック b が新たに退避可能になるのは b±halo の最後の非 full ブロックが full 化した瞬間 =
        // b±halo 内の列が finalize した波。その列の b_col は b±halo に入る ⟺ b は b_col±halo に入る
        // ので、touched_blocks±halo を候補にすれば取りこぼさない（健全性は元の全走査と同一）。
        if !touched_blocks.is_empty() {
            let nb = store.nblocks();
            touched_blocks.sort_unstable();
            touched_blocks.dedup();
            let mut cand: Vec<usize> = Vec::new();
            for &b in &touched_blocks {
                let lo = b.saturating_sub(halo_blocks);
                let hi = (b + halo_blocks).min(nb - 1);
                cand.extend(lo..=hi);
            }
            cand.sort_unstable();
            cand.dedup();
            for b in cand {
                if store.evicted[b] || store.blocks[b].is_none() {
                    continue; // 退避済み or 未確保（空/パディング）は対象外。
                }
                let lo = b.saturating_sub(halo_blocks);
                let hi = (b + halo_blocks).min(nb - 1);
                if (lo..=hi).all(|k| store.block_full(k)) {
                    store.evict_block(b);
                    freed_blocks += 1;
                }
            }
        }

        peak_resident_blocks = peak_resident_blocks.max(store.resident_blocks() as u64);
        peak_resident_cols = peak_resident_cols.max(live.len() as u64);

        // ── 終了判定: 到達済み非 final 列（= live）が残っていない ──
        if live.is_empty() {
            break true;
        }

        // T を次バンドへ進め、deferred（live かつ col_min < 新 T）を膨張して再活性。
        t = t.saturating_add(band);
        let mut react = Bitboard2D::new(nx as u32, ny as u32);
        for &i in &live {
            if col_min[i] < t {
                let ix = (i % nx as usize) as u32;
                let iy = (i / nx as usize) as u32;
                react.set(ix, iy);
            }
        }
        frontier = react
            .dilate(dx, dy)
            .enumerate()
            .filter(|&(ixu, iyu)| !col_final[cidx(ixu as i32, iyu as i32)])
            .collect();
    };

    let total_blocks = store.nblocks() as u64;
    drop(store); // 残常駐ブロックも解放（出力は sink に確定済み）。states 借用もここで終わる。

    write_back_sink(vi, &g, sink);

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

/// 確定出力 sink（finalize 時に保存済み）から states へ値・方策を書き戻す。退避でブロックが解放
/// されていても sink に残るので store には触れない。orig 索引 `it + ix·nt + iy·nt·nx`。
fn write_back_sink(vi: &mut ValueIterator, g: &Geom, sink: &dyn CompactSink) {
    let (nt, nx) = (g.nt, g.nx);
    for s in vi.states.iter_mut() {
        let orig = (s.it + s.ix * nt + s.iy * nt * nx) as usize;
        let (v, a) = sink.read(orig);
        s.total_cost = v;
        s.optimal_action = if a < 0 { None } else { Some(a as usize) };
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

    /// 直列 G-S パス（nthreads==1）を機械のコア数に依らず固定して value+policy parity を検証。
    #[test]
    fn parity_serial_nthreads1_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            let mut sink = RamSink::new(vi.states.len());
            let s = solve_compact_into_nthreads(vi, 4000, None, &mut sink, 1);
            (s.iters, s.updates, s.converged)
        });
    }

    /// 並列 Jacobi パス（nthreads==4）を固定して value+policy parity を検証。固定点は一意なので
    /// スレッド数・チャンク分割に依らず本家と bit-exact（決定的 Jacobi）。
    #[test]
    fn parity_parallel_nthreads4_compact() {
        crate::solvers::test_support::parity_standard_maps(|vi| {
            let mut sink = RamSink::new(vi.states.len());
            let s = solve_compact_into_nthreads(vi, 4000, None, &mut sink, 4);
            (s.iters, s.updates, s.converged)
        });
    }

    /// 複数行ブロックに跨る大きめ空マップ × 並列 Jacobi（nthreads==4）で、チャンク境界を跨ぐ
    /// スナップショット読み（隣接列が別スレッド担当）と退避を同時に刺激し bit-exact を確認。
    #[test]
    fn parity_parallel_larger_nthreads4_compact() {
        use crate::solvers::test_support::{make_vi, run_reference_to_fixed_point, REACH};
        let (w, h) = (32, 24);
        let occ = vec![0i8; (w * h) as usize];
        let mut a = make_vi(w, h, occ.clone());
        let mut b = make_vi(w, h, occ);
        run_reference_to_fixed_point(&mut a);

        let mut sink = RamSink::new(b.states.len());
        let s = solve_compact_into_nthreads(&mut b, 4000, None, &mut sink, 4);
        assert!(s.converged, "parallel Jacobi must converge");
        let mut mism = 0u64;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                if a.states[i].total_cost != b.states[i].total_cost {
                    mism += 1;
                }
                assert_eq!(
                    a.states[i].optimal_action, b.states[i].optimal_action,
                    "policy mismatch @ state {i}"
                );
            }
        }
        assert_eq!(mism, 0, "parallel Jacobi value must be bit-exact with reference");
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
