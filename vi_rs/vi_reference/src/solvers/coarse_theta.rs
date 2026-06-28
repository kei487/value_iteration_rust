//! Frontier3DCoarseTheta の u64 版。`vi_algorithm/src/frontier/coarse_theta.rs` を本家 u64
//! モデルへ移植。粗い θ（`step` ごと）だけを先に伝播させてから全 θ を refine する coarse-to-fine。
//!
//! `step <= 1` は Frontier3D と等価（bit-exact）。`step > 1` は coarse pass（θ%step==0 のセルのみ
//! 更新）で値を上から下げ、その後 Frontier3D で全 θ を収束まで refine する。coarse pass は妥当な
//! Bellman 更新の部分集合なので値は固定点以上に留まり、refine が真の固定点へ収束する → 本家と
//! bit-exact（u16 版の refine 上限による近似と異なり、ここでは完全収束させる）。

use crate::solvers::{frontier3d::frontier3d_solve, frontier3d_driver};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

const COARSE_BUDGET: u32 = 64; // coarse pass の反復上限

/// セット済み `ValueIterator` を Frontier3DCoarseTheta で収束まで解く。`(iters, updates, converged)`。
pub fn frontier3d_coarse_theta_solve(
    vi: &mut ValueIterator,
    step: u32,
    max_iter: u32,
) -> (u32, u64, bool) {
    if step <= 1 {
        return frontier3d_solve(vi, max_iter);
    }
    let step_i = step as i32;

    // ── coarse pass: θ%step==0 のセルのみ更新（値を上から下げる事前伝播） ──
    let (citers, cupd, _) = frontier3d_driver(vi, COARSE_BUDGET, |vi, ix, iy, it| {
        if (it as i32) % step_i != 0 {
            return false; // 粗い θ のみ更新
        }
        let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
        let idx = vi.to_index(ix as i32, iy as i32, it as i32) as usize;
        let before = vi.states[idx].total_cost;
        value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
        vi.states[idx].total_cost < before
    });

    // ── refine: 全 θ を Frontier3D で収束まで（上からの収束 → 本家と bit-exact） ──
    let (riters, rupd, conv) = frontier3d_solve(vi, max_iter.saturating_sub(citers));
    (citers + riters, cupd + rupd, conv)
}

#[cfg(test)]
mod tests {
    use super::frontier3d_coarse_theta_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn coarse_theta_parity_standard_maps() {
        // step>1 でも refine が完全収束するので Reference と bit-exact。
        parity_standard_maps(|vi| frontier3d_coarse_theta_solve(vi, 4, 2000));
    }
}
