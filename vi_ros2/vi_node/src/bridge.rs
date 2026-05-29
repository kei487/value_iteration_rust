//! ROS-free conversion layer between ROS message views and vi_rs types.
//!
//! Bridge functions take "view" structs (plain borrowed POD) rather than
//! ROS message types. `main.rs` is responsible for pulling fields out of
//! `nav_msgs::msg::OccupancyGrid` / `geometry_msgs::msg::PoseStamped`
//! and constructing these views. Keeping this module ROS-free means
//! `cargo test -p vi_node --lib` runs without ROS installed.

use ndarray::Array2;
use vi_core::{Penalty, PENALTY_OBSTACLE};

#[derive(Debug, Clone, Copy)]
pub struct OccupancyGridView<'a> {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: &'a [i8],
}

#[derive(Debug, Clone, Copy)]
pub struct PoseView {
    pub x: f64,
    pub y: f64,
    pub yaw_rad: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct PenaltyParams {
    pub safety_radius_m: f64,
    pub safety_radius_penalty: u16,
    /// Behavior for OccupancyGrid cells with value `-1` (unknown).
    /// vi_node uses `Obstacle` by default — matches the conservative
    /// reading of value_iteration when no cost map is provided.
    pub unknown_as_obstacle: bool,
}

/// `OccupancyGrid` (data values in `-1` or `0..=100`) → `Array2<Penalty>`
/// indexed as `[iy, ix]`.
///
/// - `data[iy * width + ix] == 100` (or `-1` when `unknown_as_obstacle`)
///   → `PENALTY_OBSTACLE`.
/// - free cells start at `0`, then any free cell within
///   `safety_radius_m` (chessboard distance in cells) of an obstacle
///   is set to `safety_radius_penalty` unless it is already obstacle.
pub fn occupancy_to_penalty(
    grid: &OccupancyGridView,
    params: &PenaltyParams,
) -> Array2<Penalty> {
    let w = grid.width as usize;
    let h = grid.height as usize;
    assert_eq!(grid.data.len(), w * h, "OccupancyGrid data length mismatch");
    let mut p = Array2::<Penalty>::zeros((h, w));
    let radius_cells = (params.safety_radius_m / grid.resolution).ceil() as i32;

    // First pass: obstacles.
    for iy in 0..h {
        for ix in 0..w {
            let v = grid.data[iy * w + ix];
            let obs = v >= 100 || (v < 0 && params.unknown_as_obstacle);
            if obs {
                p[[iy, ix]] = PENALTY_OBSTACLE;
            }
        }
    }

    // Second pass: dilation.
    if radius_cells > 0 && params.safety_radius_penalty > 0 {
        let r = radius_cells;
        for iy in 0..h {
            for ix in 0..w {
                if p[[iy, ix]] == PENALTY_OBSTACLE { continue; }
                let mut near_obs = false;
                'scan: for dy in -r..=r {
                    for dx in -r..=r {
                        let ny = iy as i32 + dy;
                        let nx = ix as i32 + dx;
                        if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 { continue; }
                        if p[[ny as usize, nx as usize]] == PENALTY_OBSTACLE {
                            near_obs = true;
                            break 'scan;
                        }
                    }
                }
                if near_obs {
                    p[[iy, ix]] = params.safety_radius_penalty;
                }
            }
        }
    }

    p
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(w: u32, h: u32, res: f64, data: Vec<i8>) -> (Vec<i8>, OccupancyGridView<'static>) {
        // Leak the data to obtain a 'static slice for ergonomic test setup.
        let leaked: &'static [i8] = Box::leak(data.clone().into_boxed_slice());
        (data, OccupancyGridView {
            width: w, height: h, resolution: res,
            origin_x: 0.0, origin_y: 0.0,
            data: leaked,
        })
    }

    fn params(rad: f64, pen: u16, unk_as_obs: bool) -> PenaltyParams {
        PenaltyParams {
            safety_radius_m: rad,
            safety_radius_penalty: pen,
            unknown_as_obstacle: unk_as_obs,
        }
    }

    #[test]
    fn all_free_yields_all_zero() {
        let (_, g) = grid(4, 3, 0.05, vec![0; 12]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert!(p.iter().all(|&v| v == 0));
    }

    #[test]
    fn obstacle_value_100_is_marked() {
        let mut data = vec![0i8; 9];
        data[4] = 100;
        let (_, g) = grid(3, 3, 0.05, data);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert_eq!(p[[1, 1]], PENALTY_OBSTACLE);
    }

    #[test]
    fn unknown_treated_as_obstacle_when_flag_set() {
        let (_, g) = grid(2, 1, 0.05, vec![-1, 0]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, true));
        assert_eq!(p[[0, 0]], PENALTY_OBSTACLE);
        assert_eq!(p[[0, 1]], 0);
    }

    #[test]
    fn unknown_treated_as_free_when_flag_unset() {
        let (_, g) = grid(2, 1, 0.05, vec![-1, 0]);
        let p = occupancy_to_penalty(&g, &params(0.0, 0, false));
        assert_eq!(p[[0, 0]], 0);
    }

    #[test]
    fn safety_radius_one_cell_dilation() {
        // 5x5, single obstacle in the center. radius=0.05m, res=0.05m → 1 cell.
        let mut data = vec![0i8; 25];
        data[12] = 100;
        let (_, g) = grid(5, 5, 0.05, data);
        let p = occupancy_to_penalty(&g, &params(0.05, 42, true));
        assert_eq!(p[[2, 2]], PENALTY_OBSTACLE);
        // Immediate neighbours (chessboard 1) get the dilation value.
        for (iy, ix) in [(1,1),(1,2),(1,3),(2,1),(2,3),(3,1),(3,2),(3,3)] {
            assert_eq!(p[[iy, ix]], 42, "dilated cell ({iy},{ix}) must be 42");
        }
        // Distance-2 cells stay 0.
        assert_eq!(p[[0, 0]], 0);
        assert_eq!(p[[4, 4]], 0);
    }
}
