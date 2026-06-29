//! Frontier3DTau の u64 版。`vi_algorithm/src/frontier/tau.rs` を本家 u64 モデルへ移植。
//! `tau == 0` では Frontier3D と等価（bit-exact）。`tau > 0` は per-cell の減少量が `tau` を
//! 超えるときのみ更新・伝播する近似（小さな改善を捨てて高速化、bit-exact ではない）。

use crate::solvers::{frontier3d::frontier3d_solve, frontier3d_driver};
use crate::value_iterator::{min_action_cost, ValueIterator};

/// セット済み `ValueIterator` を Frontier3DTau で収束まで解く。`(iters, updates, converged)`。
pub fn frontier3d_tau_solve(vi: &mut ValueIterator, tau: u64, max_iter: u32) -> (u32, u64, bool) {
    if tau == 0 {
        // tau=0 は Frontier3D と完全等価（policy 追跡まで一致させるため委譲）。
        return frontier3d_solve(vi, max_iter);
    }
    frontier3d_driver(vi, max_iter, |vi, ix, iy, it| {
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
        let old = vi.states[idx].total_cost;
        if let Some((min_cost, min_a)) = min_action_cost(&vi.states, &vi.actions, idx, nx, ny, nt) {
            // 減少が tau を超えるときのみ更新・伝播。
            if old.saturating_sub(min_cost) > tau {
                vi.states[idx].total_cost = min_cost;
                vi.states[idx].optimal_action = min_a;
                return true;
            }
        }
        false
    })
}

#[cfg(test)]
mod tests {
    use super::frontier3d_tau_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn tau0_parity_standard_maps() {
        // tau=0 は Frontier3D 等価 → Reference と bit-exact。
        parity_standard_maps(|vi| frontier3d_tau_solve(vi, 0, 2000));
    }
}
