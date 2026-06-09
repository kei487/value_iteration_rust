//! Oracle-equivalence tests for the u64 node path (library-internal so they run
//! under `cargo test --lib` — an integration test in `tests/` would force cargo
//! to compile the rclrs `vi_node` binary, which only links via colcon, not plain
//! cargo).
//!
//! Two properties, both bit-exact on reachable cells:
//!   1. The incremental worker (`spawn_sweep` running `solve` in bounded chunks)
//!      reaches the SAME fixed point as a single monolithic `solve(Reference)`.
//!   2. A `ValueIterator` built through the bridge's occupancy conversion equals
//!      one built from a directly-authored `OccupancyGrid` (same 0/100 data).

#![cfg(test)]

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

use ndarray::Array3;

use crate::bridge::{occupancy_view_to_vi_grid, OccupancyGridView};
use crate::sweep_thread::{spawn_sweep, DumpData};
use vi_reference::params::PROB_BASE;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

const REACH_STEPS: f64 = 1_000_000.0; // total_cost/PROB_BASE 境界 (compare.py value>=1e6)
const BIG_BUDGET: u32 = 1_000_000;
const W: i32 = 16;
const H: i32 = 16;
const RES: f64 = 0.05;

fn actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

/// 16x16 with a vertical wall at x=8 (gap at the goal row y=8), goal at centre.
fn wall_occupancy() -> Vec<i8> {
    let (w, h) = (W as usize, H as usize);
    let mut occ = vec![0i8; w * h];
    let (gx, gy) = (w / 2, h / 2);
    for y in 0..h {
        if y == gy {
            continue;
        }
        occ[y * w + gx] = 100;
    }
    occ[gy * w + gx] = 0;
    occ
}

/// total_cost/PROB_BASE as f64 over (H, W, N_THETA).
fn value_f64(vi: &ValueIterator) -> Array3<f64> {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut v = Array3::<f64>::zeros((ny as usize, nx as usize, nt as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                v[[iy as usize, ix as usize, it as usize]] = (s.total_cost / PROB_BASE) as f64;
            }
        }
    }
    v
}

fn build_via_bridge(occ: Vec<i8>) -> ValueIterator {
    let view = OccupancyGridView {
        width: W as u32,
        height: H as u32,
        resolution: RES,
        origin_x: 0.0,
        origin_y: 0.0,
        data: &occ[..],
    };
    let grid = occupancy_view_to_vi_grid(&view, true);
    let mut vi = ValueIterator::new(actions(), 1);
    vi.set_map_with_occupancy_grid(&grid, 60, 0.2, 30.0, 0.1, 15);
    let g = W as f64 * RES / 2.0;
    vi.set_goal(g, g, 0);
    vi
}

#[test]
fn worker_matches_monolithic_reference() {
    // Worker path: incremental bounded solve via spawn_sweep, capture final value.
    let slot: Arc<Mutex<Option<DumpData>>> = Arc::new(Mutex::new(None));
    let cancel = Arc::new(AtomicBool::new(false));
    let h = spawn_sweep(
        build_via_bridge(wall_occupancy()),
        U64Solver::Reference,
        BIG_BUDGET,
        cancel,
        Some(Arc::clone(&slot)),
    );
    let stats = h.join.join().expect("worker panicked");
    assert!(stats.converged, "worker must converge");
    let worker_value = slot.lock().unwrap().take().expect("dump").value;

    // Direct path: monolithic solve.
    let mut vi = build_via_bridge(wall_occupancy());
    let s = solve(&mut vi, U64Solver::Reference, BIG_BUDGET);
    assert!(s.converged, "direct solve must converge");
    let direct_value = value_f64(&vi);

    assert_eq!(worker_value.shape(), direct_value.shape());
    let mut n_reach = 0u64;
    for (w, d) in worker_value.iter().zip(direct_value.iter()) {
        if *d < REACH_STEPS {
            n_reach += 1;
            assert_eq!(w, d, "incremental worker must match monolithic reference bit-exactly");
        }
    }
    assert!(n_reach > 0, "reachable cells must exist (VI actually ran)");
}

#[test]
fn bridge_grid_matches_direct_grid() {
    // Bridge conversion of (0/100/-1) occupancy must equal a directly-authored
    // 0/100 OccupancyGrid: same penalty / cost / free flags after set_map.
    let occ = wall_occupancy();
    let a = build_via_bridge(occ.clone());

    let grid = OccupancyGrid {
        width: W,
        height: H,
        resolution: RES,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };
    let mut b = ValueIterator::new(actions(), 1);
    b.set_map_with_occupancy_grid(&grid, 60, 0.2, 30.0, 0.1, 15);
    let g = W as f64 * RES / 2.0;
    b.set_goal(g, g, 0);

    assert_eq!(a.states.len(), b.states.len());
    for (sa, sb) in a.states.iter().zip(b.states.iter()) {
        assert_eq!(sa.free, sb.free, "free flag mismatch @ ({},{},{})", sa.ix, sa.iy, sa.it);
        assert_eq!(sa.penalty, sb.penalty, "penalty mismatch");
        assert_eq!(sa.total_cost, sb.total_cost, "cost mismatch");
        assert_eq!(sa.final_state, sb.final_state, "final_state mismatch");
    }
}
