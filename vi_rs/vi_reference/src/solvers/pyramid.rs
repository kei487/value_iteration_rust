//! PyramidSweep の u64 版（bit-exact 安全版）。
//!
//! `vi_algorithm/src/block/pyramid.rs` は 2×2 空間ピラミッドで粗レベルの value を細レベルへ
//! prolongate（上位サンプリング）する。しかし確率的 Bellman backup は floor を含み固定点が
//! **非一意**になり得る（粗値が過小評価だと VI は min ベースで増加できず誤った固定点に陥る）。
//! 本 u64 版はこの非一意性を避け、**値の prolongation を行わない**。代わりに粗→細の
//! **ブロックスケジュール**で BlockRefine を回す: 値は常に MAX_COST から単調降下し、最終（細）
//! レベルを収束まで回すため、結果は BlockRefine = Reference = 本家と bit-exact。粗レベルは
//! 事前伝播による加速のみを担う（coarse-to-fine の精神は保持）。

use crate::solvers::block::block_refine_sized;
use crate::value_iterator::ValueIterator;

const FINEST: i32 = 8; // 最終レベルのブロックサイズ (= BlockRefine 既定)
const COARSE_BUDGET: u32 = 4; // 粗レベルの事前伝播の反復上限
const LOCAL_SWEEPS: u32 = 2;

/// セット済み `ValueIterator` を PyramidSweep（bit-exact 安全版）で収束まで解く。
pub fn pyramid_sweep_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let max_dim = vi.cell_num_x.max(vi.cell_num_y);

    // 粗→細のブロックサイズ列。FINEST より大きく map に意味のある粗サイズを降順に並べ、
    // 最後に FINEST。小さい map では粗レベルが無く FINEST のみ（= BlockRefine）。
    let mut sizes: Vec<i32> = [64, 32, 16]
        .into_iter()
        .filter(|&s| s > FINEST && s < max_dim)
        .collect();
    sizes.push(FINEST);

    let mut total_iters = 0u32;
    let mut total_updates = 0u64;
    let mut converged = false;
    let n = sizes.len();
    for (i, &block) in sizes.iter().enumerate() {
        let is_finest = i == n - 1;
        let budget = if is_finest {
            max_iter.saturating_sub(total_iters)
        } else {
            COARSE_BUDGET
        };
        let (it, up, conv) = block_refine_sized(vi, block, LOCAL_SWEEPS, budget);
        total_iters = total_iters.saturating_add(it);
        total_updates += up;
        converged = conv; // 最終レベルの収束が全体の収束
        if total_iters >= max_iter {
            break;
        }
    }
    (total_iters, total_updates, converged)
}

#[cfg(test)]
mod tests {
    use super::pyramid_sweep_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_pyramid_sweep() {
        parity_standard_maps(|vi| pyramid_sweep_solve(vi, 2000));
    }
}
