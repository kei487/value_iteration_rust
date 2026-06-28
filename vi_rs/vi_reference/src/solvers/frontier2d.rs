//! Frontier2D の u64 版。`vi_algorithm/src/frontier/f2d.rs` を本家 u64 モデルへ移植。
//! 空間 2D フロンティア: 活性 (ix,iy) が現れたら全 θ 層を再評価する。dilation は空間のみで
//! 安い代わりに per-cell 仕事量が N_THETA 倍。収束値・方策は Reference = 本家と bit-exact。

use crate::solvers::{displacement, frontier2d_driver, seed_frontier_2d};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み `ValueIterator` を Frontier2D で収束まで解く。`(iters, updates, converged)` を返す。
///
/// 反復骨格は [`frontier2d_driver`] が担う。候補 (ix,iy) の全 θ 層を `value_iteration_raw` で
/// 更新し、減少した θ 層数を返す（1 以上なら次フロンティアへ）。
pub fn frontier2d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, _mt) = displacement(vi);
    let seed = seed_frontier_2d(vi);
    frontier2d_driver(nx, ny, seed, mx as u32, my as u32, max_iter, |ix, iy| {
        let mut updates = 0u64;
        for it in 0..nt {
            let idx = vi.to_index(ix as i32, iy as i32, it) as usize;
            let before = vi.states[idx].total_cost;
            value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
            if vi.states[idx].total_cost < before {
                updates += 1;
            }
        }
        updates
    })
}

#[cfg(test)]
mod tests {
    use super::frontier2d_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_frontier2d() {
        parity_standard_maps(|vi| frontier2d_solve(vi, 2000));
    }
}
