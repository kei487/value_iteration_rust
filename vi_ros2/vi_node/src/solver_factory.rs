//! Maps the `solver: string` ROS parameter to a `vi_reference::solvers::U64Solver`.
//!
//! u16 時代は `Box<dyn Solver>` を返していたが、u64 (本家忠実) 移行で `U64Solver`
//! (Copy な enum) を返す。近似ソルバは no-op パラメータ（tau=0 / k=全 outcome /
//! step=1）= Frontier3D 等価で渡す（本家と bit-exact）。

use anyhow::{anyhow, Result};
use vi_reference::solvers::U64Solver;

pub fn make_solver(name: &str) -> Result<U64Solver> {
    Ok(match name {
        "reference" => U64Solver::Reference,
        "frontier3d" => U64Solver::Frontier3D,
        "frontier3d_topk" => U64Solver::Frontier3DTopK { k: u32::MAX },
        "frontier3d_tau" => U64Solver::Frontier3DTau { tau: 0 },
        "frontier3d_coarse_theta" => U64Solver::Frontier3DCoarseTheta { step: 1 },
        "frontier2d" => U64Solver::Frontier2D,
        "frontier_stack" => U64Solver::FrontierStack,
        "block_refine" => U64Solver::BlockRefine,
        // 旧 ROS パラメータ名 "pyramid" を PyramidSweep にマップ（"pyramid_sweep" も許容）。
        "pyramid" | "pyramid_sweep" => U64Solver::PyramidSweep,
        "stream_mimic" => U64Solver::StreamMimic,
        other => {
            return Err(anyhow!(
                "unknown solver: {other}. Supported: reference | frontier3d | frontier3d_topk | \
                 frontier3d_tau | frontier3d_coarse_theta | frontier2d | frontier_stack | \
                 block_refine | pyramid | stream_mimic"
            ))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_solvers_resolve() {
        for name in [
            "reference",
            "frontier3d",
            "frontier3d_topk",
            "frontier3d_tau",
            "frontier3d_coarse_theta",
            "frontier2d",
            "frontier_stack",
            "block_refine",
            "pyramid",
            "pyramid_sweep",
            "stream_mimic",
        ] {
            make_solver(name).unwrap_or_else(|_| panic!("solver `{name}` must resolve"));
        }
    }

    #[test]
    fn pyramid_alias_maps_to_pyramid_sweep() {
        assert_eq!(make_solver("pyramid").unwrap(), U64Solver::PyramidSweep);
        assert_eq!(make_solver("pyramid_sweep").unwrap(), U64Solver::PyramidSweep);
    }

    #[test]
    fn unknown_solver_errors_with_listing() {
        let err = match make_solver("does_not_exist") {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected Err for unknown solver"),
        };
        assert!(err.contains("does_not_exist"));
        assert!(err.contains("Supported"));
    }
}
