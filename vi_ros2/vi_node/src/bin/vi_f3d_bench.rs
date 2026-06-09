//! 比較ベンチ用ハーネス: vi_reference の **Frontier3D** (u64) ソルバを、vi_compare
//! パイプラインと同一の入力 (map_server 意味論の OccupancyGrid)・ゴール・パラメータで
//! 走らせ、`value_f3d.npy` / `policy_f3d.npy` (float64, 形状 (H, W, N_THETA)) と
//! `timing_f3d.json` を出力する。
//!
//! 設計意図:
//!   * ROS を経由しない直接ハーネスにすることで vi_node の feedback ポンプ等のオーバーヘッドを
//!     排除し、ros1 / ref と公平に速度比較できる。
//!   * `ValueIterator` の構築は vi_node の `bridge`（`occupancy_view_to_vi_grid`）を **そのまま
//!     再利用** する。よって本ハーネスは vi_node (solver=frontier3d) と同一の本家 u64 モデルで
//!     あり、Frontier3D の収束固定点は Reference (=本家) と bit 一致する (f3d ≡ ros2 のクロス
//!     チェックが成立する)。
//!
//! 使い方 (位置引数, f3d_bench.py が組み立てる。vi_ref_bench と同一レイアウト):
//!   vi_f3d_bench <occ_raw> <width> <height> <resolution> <origin_x> <origin_y>
//!                <goal_x> <goal_y> <goal_yaw_deg>
//!                <theta_cell_num> <safety_radius> <safety_radius_penalty>
//!                <goal_margin_radius> <goal_margin_theta>
//!                <max_sweeps> <delta_threshold> <out_dir>

use std::fs::File;
use std::io::Write;
use std::time::Instant;

use vi_node::bridge::{occupancy_view_to_vi_grid, OccupancyGridView};
use vi_node::npy::write_f64;
use vi_node::sweep_thread::compute_policy;

use vi_reference::params::PROB_BASE;
use vi_reference::solvers::{solve, U64Solver};
use vi_reference::{Action, ValueIterator};

fn arg<T: std::str::FromStr>(args: &[String], i: usize, name: &str) -> T
where
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    args.get(i)
        .unwrap_or_else(|| panic!("missing arg {i} ({name})"))
        .parse::<T>()
        .unwrap_or_else(|e| panic!("bad arg {i} ({name}): {e}"))
}

fn default_actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 18 {
        eprintln!(
            "usage: {} <occ_raw> <width> <height> <resolution> <origin_x> <origin_y> \
             <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius> \
             <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> \
             <max_sweeps> <delta_threshold> <out_dir>",
            args.first().map(String::as_str).unwrap_or("vi_f3d_bench")
        );
        std::process::exit(2);
    }

    let occ_raw: String = arg(&args, 1, "occ_raw");
    let width: i32 = arg(&args, 2, "width");
    let height: i32 = arg(&args, 3, "height");
    let resolution: f64 = arg(&args, 4, "resolution");
    let origin_x: f64 = arg(&args, 5, "origin_x");
    let origin_y: f64 = arg(&args, 6, "origin_y");
    let goal_x: f64 = arg(&args, 7, "goal_x");
    let goal_y: f64 = arg(&args, 8, "goal_y");
    let goal_yaw_deg: f64 = arg(&args, 9, "goal_yaw_deg");
    let theta_cell_num: i32 = arg(&args, 10, "theta_cell_num");
    let safety_radius: f64 = arg(&args, 11, "safety_radius");
    let safety_radius_penalty: f64 = arg(&args, 12, "safety_radius_penalty");
    let goal_margin_radius: f64 = arg(&args, 13, "goal_margin_radius");
    let goal_margin_theta: i32 = arg(&args, 14, "goal_margin_theta");
    let max_sweeps: u32 = arg(&args, 15, "max_sweeps");
    let delta_threshold: f64 = arg(&args, 16, "delta_threshold");
    let out_dir: String = arg(&args, 17, "out_dir");

    // occupancy (raw i8, len=width*height, row-major, ros2 bench_client の to_occupancy と同一)
    let raw = std::fs::read(&occ_raw).expect("read occ_raw");
    let n = (width as usize) * (height as usize);
    assert_eq!(raw.len(), n, "occ_raw size {} != width*height {}", raw.len(), n);
    let data: Vec<i8> = raw.iter().map(|&b| b as i8).collect();

    let view = OccupancyGridView {
        width: width as u32,
        height: height as u32,
        resolution,
        origin_x,
        origin_y,
        data: &data[..],
    };

    // ── ValueIterator 構築: vi_node main.rs と同一手順 (bridge を再利用) ───────────────
    let grid = occupancy_view_to_vi_grid(&view, true); // unknown -> obstacle (node default)
    let mut vi = ValueIterator::new(default_actions(), 1);
    vi.set_map_with_occupancy_grid(
        &grid,
        theta_cell_num,
        safety_radius,
        safety_radius_penalty,
        goal_margin_radius,
        goal_margin_theta,
    );
    // 本家 executeVi: int t = (int)(yaw_rad*180/M_PI) = (int)goal_yaw_deg。
    vi.set_goal(goal_x, goal_y, goal_yaw_deg as i32);

    // ── 求解: Frontier3D をフロンティアが空になる固定点まで ─────────────────────────
    let t0 = Instant::now();
    let stats = solve(&mut vi, U64Solver::Frontier3D, max_sweeps);
    let elapsed = t0.elapsed().as_secs_f64();

    // value = total_cost/PROB_BASE (本家 valueFunctionWriter int-division), policy = action id。
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut value = ndarray::Array3::<f64>::zeros((ny as usize, nx as usize, nt as usize));
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                value[[iy as usize, ix as usize, it as usize]] = (s.total_cost / PROB_BASE) as f64;
            }
        }
    }
    let policy = compute_policy(&vi);

    std::fs::create_dir_all(&out_dir).expect("mkdir out_dir");
    write_f64(&format!("{out_dir}/value_f3d.npy"), &value).expect("write value_f3d");
    write_f64(&format!("{out_dir}/policy_f3d.npy"), &policy).expect("write policy_f3d");

    let timing = format!(
        "{{\n  \"elapsed_sec\": {},\n  \"sweeps\": {},\n  \"updates\": {},\n  \"converged\": {},\n  \"thread_num\": 1,\n  \"delta_threshold\": {},\n  \"side\": \"f3d\"\n}}\n",
        elapsed,
        stats.iters,
        stats.updates,
        if stats.converged { "true" } else { "false" },
        delta_threshold
    );
    File::create(format!("{out_dir}/timing_f3d.json"))
        .and_then(|mut f| f.write_all(timing.as_bytes()))
        .expect("write timing_f3d");

    eprintln!(
        "[vi_f3d_bench] iters={} updates={} converged={} elapsed={:.3}s shape=[{}, {}, {}]",
        stats.iters, stats.updates, stats.converged, elapsed, ny, nx, nt
    );
}
