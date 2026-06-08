//! 本家 `ValueIterator` 忠実移植 (フルパイプライン)。

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::action::Action;
use crate::msg::{OccupancyGrid, Quaternion};
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT, RESOLUTION_T_BIT, RESOLUTION_XY_BIT};
use crate::state::State;
use crate::state_transition::StateTransition;
use crate::sweep_status::SweepWorkerStatus;

pub struct ValueIterator {
    pub states: Vec<State>,
    pub actions: Vec<Action>,
    pub sweep_orders: Vec<Vec<i32>>,
    pub thread_status: BTreeMap<i32, SweepWorkerStatus>,
    pub status: String,

    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_margin_radius: f64,
    pub goal_t: i32,
    pub goal_margin_theta: i32,
    pub thread_num: i32,

    pub xy_resolution: f64,
    pub t_resolution: f64,
    pub cell_num_x: i32,
    pub cell_num_y: i32,
    pub cell_num_t: i32,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub map_origin_quat: Quaternion,
}

impl ValueIterator {
    /// 本家 `ValueIterator(std::vector<Action> &actions, int thread_num)`。
    pub fn new(actions: Vec<Action>, thread_num: i32) -> Self {
        Self {
            states: Vec::new(),
            actions,
            sweep_orders: Vec::new(),
            thread_status: BTreeMap::new(),
            status: "init".to_string(),
            goal_x: 0.0,
            goal_y: 0.0,
            goal_margin_radius: 0.0,
            goal_t: 0,
            goal_margin_theta: 0,
            thread_num,
            xy_resolution: 0.0,
            t_resolution: 0.0,
            cell_num_x: 0,
            cell_num_y: 0,
            cell_num_t: 0,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            map_origin_quat: Quaternion::default(),
        }
    }

    /// 本家 `toIndex(ix,iy,it) = it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_)`。
    pub fn to_index(&self, ix: i32, iy: i32, it: i32) -> i32 {
        to_index_raw(ix, iy, it, self.cell_num_x, self.cell_num_t)
    }

    /// 本家 `inMapArea`。
    pub fn in_map_area(&self, ix: i32, iy: i32) -> bool {
        ix >= 0 && ix < self.cell_num_x && iy >= 0 && iy < self.cell_num_y
    }

    /// 本家 `setStateTransition`。θ ごとに 1 スレッドで遷移生成 (書き込み先が
    /// θ 独立なので結果は決定的)。各 action の `state_transitions[it]` を埋める。
    pub(crate) fn set_state_transition(&mut self) {
        let cell_num_t = self.cell_num_t;
        let xy_resolution = self.xy_resolution;
        let t_resolution = self.t_resolution;

        for a in self.actions.iter_mut() {
            a.state_transitions = vec![Vec::new(); cell_num_t as usize];
        }

        let action_params: Vec<(f64, f64)> =
            self.actions.iter().map(|a| (a.delta_fw, a.delta_rot)).collect();

        // per_theta[it][a] を θ 並列で計算。
        let per_theta: Vec<Vec<Vec<StateTransition>>> = std::thread::scope(|scope| {
            let ap = &action_params;
            let handles: Vec<_> = (0..cell_num_t)
                .map(|it| {
                    scope.spawn(move || {
                        ap.iter()
                            .map(|&(fw, rot)| {
                                compute_theta_transitions(fw, rot, it, xy_resolution, t_resolution)
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        for (it, per_action) in per_theta.into_iter().enumerate() {
            for (a, list) in per_action.into_iter().enumerate() {
                self.actions[a].state_transitions[it] = list;
            }
        }
    }

    /// 本家 `setMapWithOccupancyGrid`。
    pub fn set_map_with_occupancy_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        safety_radius_penalty: f64,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.cell_num_t = theta_cell_num;
        self.goal_margin_radius = goal_margin_radius;
        self.goal_margin_theta = goal_margin_theta;
        self.cell_num_x = map.width;
        self.cell_num_y = map.height;
        self.xy_resolution = map.resolution;
        // ★整数除算後に f64 化 (本家 `t_resolution_ = 360/cell_num_t_;`)。
        self.t_resolution = (360 / self.cell_num_t) as f64;
        self.map_origin_x = map.origin_x;
        self.map_origin_y = map.origin_y;
        self.map_origin_quat = map.origin_quat.clone();

        self.set_state(map, safety_radius, safety_radius_penalty);
        self.set_state_transition();
        self.set_sweep_orders();
    }

    /// 本家 `setState`。
    fn set_state(&mut self, map: &OccupancyGrid, safety_radius: f64, safety_radius_penalty: f64) {
        self.states.clear();
        let margin = (safety_radius / self.xy_resolution).ceil() as i32;
        for y in 0..self.cell_num_y {
            for x in 0..self.cell_num_x {
                for t in 0..self.cell_num_t {
                    self.states.push(State::from_occupancy(
                        x,
                        y,
                        t,
                        map,
                        margin,
                        safety_radius_penalty,
                        self.cell_num_x,
                    ));
                }
            }
        }
    }

    /// 本家 `setMapWithCostGrid`。`margin` は本家にあるが未使用。
    pub fn set_map_with_cost_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        _safety_radius_penalty: f64,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.cell_num_t = theta_cell_num;
        self.goal_margin_radius = goal_margin_radius;
        self.goal_margin_theta = goal_margin_theta;
        self.cell_num_x = map.width;
        self.cell_num_y = map.height;
        self.xy_resolution = map.resolution;
        self.t_resolution = (360 / self.cell_num_t) as f64;
        self.map_origin_x = map.origin_x;
        self.map_origin_y = map.origin_y;
        self.map_origin_quat = map.origin_quat.clone();

        self.states.clear();
        let _margin = (safety_radius / self.xy_resolution).ceil() as i32; // 本家にあるが未使用
        for y in 0..self.cell_num_y {
            for x in 0..self.cell_num_x {
                // 本家 `(unsigned int)(map.data[x + cell_num_x_*y] & 0xFF)`。
                let cost = (map.data[(x + self.cell_num_x * y) as usize] as u8) as u32;
                for t in 0..self.cell_num_t {
                    self.states.push(State::from_cost(x, y, t, cost));
                }
            }
        }
        self.set_state_transition();
        self.set_sweep_orders();
    }

    /// 本家 `setSweepOrders`。6 種の走査順を生成。既に生成済みなら何もしない。
    /// ★[4]=[0]全体+[1]後半、[5]=[1]前半 というアンバランス/重複を逐語再現。
    pub(crate) fn set_sweep_orders(&mut self) {
        if !self.sweep_orders.is_empty() {
            return;
        }
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);

        // [0]: y, x, t 順
        let mut o0 = Vec::new();
        for y in 0..ny {
            for x in 0..nx {
                for t in 0..nt {
                    o0.push(self.to_index(x, y, t));
                }
            }
        }
        // [1]: x, y, t 順
        let mut o1 = Vec::new();
        for x in 0..nx {
            for y in 0..ny {
                for t in 0..nt {
                    o1.push(self.to_index(x, y, t));
                }
            }
        }
        let o2: Vec<i32> = o0.iter().rev().cloned().collect();
        let o3: Vec<i32> = o1.iter().rev().cloned().collect();
        self.sweep_orders.push(o0); // 0
        self.sweep_orders.push(o1); // 1
        self.sweep_orders.push(o2); // 2
        self.sweep_orders.push(o3); // 3

        // [4],[5]: 本家 `for(i=0;i<2;i++){ push(前半[i]); [4].append(後半[i]); }`
        let half = self.sweep_orders[0].len() / 2;
        // i=0
        let o0_first: Vec<i32> = self.sweep_orders[0][..half].to_vec();
        self.sweep_orders.push(o0_first); // index 4 = [0]前半
        let o0_second: Vec<i32> = self.sweep_orders[0][half..].to_vec();
        self.sweep_orders[4].extend(o0_second); // [4] = [0]全体
        // i=1
        let o1_first: Vec<i32> = self.sweep_orders[1][..half].to_vec();
        self.sweep_orders.push(o1_first); // index 5 = [1]前半
        let o1_second: Vec<i32> = self.sweep_orders[1][half..].to_vec();
        self.sweep_orders[4].extend(o1_second); // [4] = [0]全体 + [1]後半
    }

    /// 本家 `actionCost`。
    pub fn action_cost(&self, s: &State, a: &Action) -> u64 {
        action_cost_raw(
            &self.states,
            a,
            s,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
        )
    }

    /// 本家 `valueIteration` (states[idx] を更新)。
    pub fn value_iteration_at(&mut self, idx: usize) -> u64 {
        value_iteration_raw(
            &mut self.states,
            &self.actions,
            idx,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
        )
    }

    /// 本家 `setGoal`。goal_t を [0,360) に正規化し、final_state を再計算。
    pub fn set_goal(&mut self, goal_x: f64, goal_y: f64, goal_t: i32) {
        let mut gt = goal_t;
        while gt < 0 {
            gt += 360;
        }
        while gt >= 360 {
            gt -= 360;
        }
        self.goal_x = goal_x;
        self.goal_y = goal_y;
        self.goal_t = gt;

        self.thread_status.clear();
        self.set_state_values();
        self.status = "calculating".to_string();
    }

    /// 本家 `setStateValues`。距離 + 向き判定で final_state を決め、値を初期化。
    fn set_state_values(&mut self) {
        let (xy_res, ox, oy) = (self.xy_resolution, self.map_origin_x, self.map_origin_y);
        let (gx, gy, gt, gm) = (self.goal_x, self.goal_y, self.goal_t, self.goal_margin_theta);
        let r2 = self.goal_margin_radius * self.goal_margin_radius;
        let t_res = self.t_resolution;

        for s in self.states.iter_mut() {
            // 距離判定
            let x0 = s.ix as f64 * xy_res + ox;
            let y0 = s.iy as f64 * xy_res + oy;
            let r0 = (x0 - gx) * (x0 - gx) + (y0 - gy) * (y0 - gy);
            let x1 = x0 + xy_res;
            let y1 = y0 + xy_res;
            let r1 = (x1 - gx) * (x1 - gx) + (y1 - gy) * (y1 - gy);
            s.final_state = r0 < r2 && r1 < r2 && s.free;

            // 向き判定 (t0/t1 は f64→i32 切り捨て)
            let t0 = (s.it as f64 * t_res) as i32;
            let t1 = ((s.it + 1) as f64 * t_res) as i32;
            let goal_t_2 = if gt > 180 { gt - 360 } else { gt + 360 };
            let ok = (gt - gm <= t0 && t1 <= gt + gm) || (goal_t_2 - gm <= t0 && t1 <= goal_t_2 + gm);
            s.final_state = s.final_state && ok;
        }

        for s in self.states.iter_mut() {
            s.total_cost = if s.final_state { 0 } else { MAX_COST };
            s.local_penalty = 0;
            s.optimal_action = None;
        }
    }
}

// ── コア free 関数 (単スレッド経路とマルチスレッド経路で共有) ──

#[inline]
pub(crate) fn to_index_raw(ix: i32, iy: i32, it: i32, cell_num_x: i32, cell_num_t: i32) -> i32 {
    it + ix * cell_num_t + iy * (cell_num_t * cell_num_x)
}

/// 本家 `cellDelta`。`it` は絶対インデックス (負正規化しない)。
pub(crate) fn cell_delta(
    x: f64,
    y: f64,
    t: f64,
    xy_resolution: f64,
    t_resolution: f64,
) -> (i32, i32, i32) {
    let mut ix = (x.abs() / xy_resolution).floor() as i32;
    if x < 0.0 {
        ix = -ix - 1;
    }
    let mut iy = (y.abs() / xy_resolution).floor() as i32;
    if y < 0.0 {
        iy = -iy - 1;
    }
    let it = (t / t_resolution).floor() as i32;
    (ix, iy, it)
}

/// 本家 `noNoiseStateTransition`。`to_t` は負方向しか正規化しない (>=360 は残す)。
pub(crate) fn no_noise_state_transition(
    delta_fw: f64,
    delta_rot: f64,
    from_x: f64,
    from_y: f64,
    from_t: f64,
) -> (f64, f64, f64) {
    let ang = from_t / 180.0 * PI;
    let to_x = from_x + delta_fw * ang.cos();
    let to_y = from_y + delta_fw * ang.sin();
    let mut to_t = from_t + delta_rot;
    while to_t < 0.0 {
        to_t += 360.0;
    }
    (to_x, to_y, to_t)
}

/// 本家 `setStateTransitionWorkerSub` の 1 (action, theta) 分。
/// サブセルサンプリングで遷移先バケットを集計する。`dit` は絶対 θ。
pub(crate) fn compute_theta_transitions(
    delta_fw: f64,
    delta_rot: f64,
    it: i32,
    xy_resolution: f64,
    t_resolution: f64,
) -> Vec<StateTransition> {
    let theta_origin = it as f64 * t_resolution;
    let xy_sample_num = 1i32 << RESOLUTION_XY_BIT; // 64
    let t_sample_num = 1i32 << RESOLUTION_T_BIT; // 64
    let xy_step = xy_resolution / xy_sample_num as f64;
    let t_step = t_resolution / t_sample_num as f64;

    let mut out: Vec<StateTransition> = Vec::new();

    // 本家 `for(double o=0.5*step; o<limit; o+=step)` の f64 累積を忠実再現。
    let mut oy = 0.5 * xy_step;
    while oy < xy_resolution {
        let mut ox = 0.5 * xy_step;
        while ox < xy_resolution {
            let mut ot = 0.5 * t_step;
            while ot < t_resolution {
                let (dx, dy, dt) =
                    no_noise_state_transition(delta_fw, delta_rot, ox, oy, ot + theta_origin);
                let (dix, diy, dit) = cell_delta(dx, dy, dt, xy_resolution, t_resolution);

                let mut exist = false;
                for s in out.iter_mut() {
                    if s.dix == dix && s.diy == diy && s.dit == dit {
                        s.prob += 1;
                        exist = true;
                        break;
                    }
                }
                if !exist {
                    out.push(StateTransition::new(dix, diy, dit, 1));
                }
                ot += t_step;
            }
            ox += xy_step;
        }
        oy += xy_step;
    }
    out
}

/// 本家 `actionCost`。★u64 オーバーフロー折り返しを `wrapping_*` で再現。
/// `dit` は絶対 θ なので `(dit + nt) % nt` で wrap (s.it は足さない)。
pub(crate) fn action_cost_raw(
    states: &[State],
    a: &Action,
    s: &State,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
) -> u64 {
    let mut cost: u64 = 0;
    for tran in &a.state_transitions[s.it as usize] {
        let ix = s.ix + tran.dix;
        if ix < 0 || ix >= cell_num_x {
            return MAX_COST;
        }
        let iy = s.iy + tran.diy;
        if iy < 0 || iy >= cell_num_y {
            return MAX_COST;
        }
        let it = (tran.dit + cell_num_t) % cell_num_t;
        let after = &states[to_index_raw(ix, iy, it, cell_num_x, cell_num_t) as usize];
        if !after.free {
            return MAX_COST;
        }
        cost = cost.wrapping_add(
            after
                .total_cost
                .wrapping_add(after.penalty)
                .wrapping_add(after.local_penalty)
                .wrapping_mul(tran.prob as u64),
        );
    }
    cost >> PROB_BASE_BIT
}

/// 本家 `valueIteration`。free でない/final_state なら 0 を返し更新しない。
pub(crate) fn value_iteration_raw(
    states: &mut [State],
    actions: &[Action],
    idx: usize,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
) -> u64 {
    if !states[idx].free || states[idx].final_state {
        return 0;
    }
    let mut min_cost: u64 = MAX_COST;
    let mut min_action: Option<usize> = None;
    {
        let s = &states[idx];
        for (ai, a) in actions.iter().enumerate() {
            let c = action_cost_raw(states, a, s, cell_num_x, cell_num_y, cell_num_t);
            if c < min_cost {
                min_cost = c;
                min_action = Some(ai);
            }
        }
    }
    let old = states[idx].total_cost;
    let delta = (min_cost as i64) - (old as i64);
    states[idx].total_cost = min_cost;
    states[idx].optimal_action = min_action;
    delta.unsigned_abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_index_layout() {
        // cell_num_x=4, cell_num_t=60。
        assert_eq!(to_index_raw(0, 0, 0, 4, 60), 0);
        assert_eq!(to_index_raw(0, 0, 5, 4, 60), 5);
        assert_eq!(to_index_raw(1, 0, 0, 4, 60), 60);
        assert_eq!(to_index_raw(0, 1, 0, 4, 60), 240);
    }

    #[test]
    fn cell_delta_negative_correction() {
        // xy_res=0.05。x=-0.01 → |x|/res=0.2 → floor 0 → x<0 → -0-1 = -1。
        let (ix, _, _) = cell_delta(-0.01, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix, -1);
        // x=0.06 → 1.2 → floor 1。
        let (ix2, _, _) = cell_delta(0.06, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix2, 1);
    }

    #[test]
    fn cell_delta_theta_absolute_not_normalized() {
        // t=366, t_res=6 → floor(61) = 61 (絶対、wrap しない)。
        let (_, _, it) = cell_delta(0.0, 0.0, 366.0, 0.05, 6.0);
        assert_eq!(it, 61);
    }

    #[test]
    fn no_noise_negative_theta_normalized_once() {
        // from_t=10, rot=-20 → to_t=-10 → +360 = 350。
        let (_, _, to_t) = no_noise_state_transition(0.0, -20.0, 0.0, 0.0, 10.0);
        assert!((to_t - 350.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_over_360_not_normalized() {
        // from_t=350, rot=20 → to_t=370 (>=360 は残す)。
        let (_, _, to_t) = no_noise_state_transition(0.0, 20.0, 0.0, 0.0, 350.0);
        assert!((to_t - 370.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_forward_uses_cos_sin() {
        // fw=0.3, from_t=0 → to_x=0.3, to_y=0。
        let (to_x, to_y, _) = no_noise_state_transition(0.3, 0.0, 0.0, 0.0, 0.0);
        assert!((to_x - 0.3).abs() < 1e-9);
        assert!(to_y.abs() < 1e-9);
    }

    fn free_grid(w: i32, h: i32) -> OccupancyGrid {
        OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0; (w * h) as usize],
        }
    }

    #[test]
    fn set_map_occupancy_populates_states_and_transitions() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(3, 2);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);

        assert_eq!(vi.cell_num_x, 3);
        assert_eq!(vi.cell_num_y, 2);
        assert_eq!(vi.cell_num_t, 60);
        assert_eq!(vi.t_resolution, 6.0);
        assert_eq!(vi.states.len(), 3 * 2 * 60);
        // 各 action の θ ごとに遷移が生成されている。
        assert_eq!(vi.actions[0].state_transitions.len(), 60);
        let total: i64 = vi.actions[0].state_transitions[0]
            .iter()
            .map(|s| s.prob as i64)
            .sum();
        assert_eq!(total, super::PROB_BASE as i64);
    }

    #[test]
    fn set_map_cost_grid_free_and_obstacle() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let mut map = free_grid(2, 1);
        map.data = vec![0, 255i32 as i8]; // 1 つ目 free(cost0), 2 つ目 255
        vi.set_map_with_cost_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // index (x=0): free。 (x=1): not free。
        let s0 = &vi.states[vi.to_index(0, 0, 0) as usize];
        let s1 = &vi.states[vi.to_index(1, 0, 0) as usize];
        assert!(s0.free);
        assert!(!s1.free);
    }

    #[test]
    fn sweep_orders_structure() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10); // 小さい cell_num_t=4
        let total = (2 * 2 * 4) as usize;
        assert_eq!(vi.sweep_orders.len(), 6);
        assert_eq!(vi.sweep_orders[0].len(), total);
        assert_eq!(vi.sweep_orders[1].len(), total);
        // [2],[3] は逆順
        let rev0: Vec<i32> = vi.sweep_orders[0].iter().rev().cloned().collect();
        assert_eq!(vi.sweep_orders[2], rev0);
        // ★[4] = [0]全体 + [1]後半 (size = total + (total - half))
        let half = total / 2;
        assert_eq!(vi.sweep_orders[4].len(), total + (total - half));
        assert_eq!(&vi.sweep_orders[4][..total], &vi.sweep_orders[0][..]);
        assert_eq!(&vi.sweep_orders[4][total..], &vi.sweep_orders[1][half..]);
        // ★[5] = [1]前半
        assert_eq!(vi.sweep_orders[5], vi.sweep_orders[1][..half].to_vec());
    }

    #[test]
    fn sweep_orders_idempotent() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10);
        let len_before = vi.sweep_orders.len();
        vi.set_sweep_orders(); // 2 回目は no-op
        assert_eq!(vi.sweep_orders.len(), len_before);
    }

    #[test]
    fn prob_sum_equals_prob_base() {
        // 任意 action・θで prob 総和 = 64^3 = 262144 = PROB_BASE。
        let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
        let total: i64 = list.iter().map(|s| s.prob as i64).sum();
        assert_eq!(total, super::PROB_BASE as i64);
    }

    #[test]
    fn forward_theta0_moves_in_x() {
        // 前進 fw=0.3, θ=0, res=0.05 → 主に dix≈6, diy=0, dit=0。
        let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
        // 最頻バケット (prob 最大) を確認。
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.diy, 0);
        assert_eq!(top.dit, 0, "θ=0 の前進は絶対 θ=0");
        assert!(top.dix >= 5 && top.dix <= 6, "dix was {}", top.dix);
    }

    #[test]
    fn rotation_dit_is_absolute_theta() {
        // 左回転 rot=+20, θ=0, t_res=6 → to_t≈20 → dit≈3 (絶対)、dix=diy=0。
        let list = compute_theta_transitions(0.0, 20.0, 0, 0.05, 6.0);
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.dix, 0);
        assert_eq!(top.diy, 0);
        assert_eq!(top.dit, 3, "rot+20 → 絶対 θ index 3");
    }

    #[test]
    fn rotation_dit_absolute_at_theta30() {
        // θ=30 (index 5, t_res=6 → θ_origin=30°), 左回転 +20 → to_t≈50 → dit≈8 (絶対)。
        let list = compute_theta_transitions(0.0, 20.0, 5, 0.05, 6.0);
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.dit, 8, "θ_origin30 + rot20 = 50° → index 8 (絶対)");
    }

    fn mk_state(ix: i32, iy: i32, it: i32, free: bool, total: u64, penalty: u64) -> State {
        State {
            total_cost: total,
            penalty,
            local_penalty: 0,
            ix,
            iy,
            it,
            free,
            final_state: false,
            optimal_action: None,
        }
    }

    fn single_action(dix: i32, diy: i32, dit: i32, nt: usize) -> Action {
        let mut a = Action::new("a", 0.0, 0.0, 0);
        a.state_transitions = vec![Vec::new(); nt];
        for it in 0..nt {
            a.state_transitions[it].push(StateTransition::new(dix, diy, dit, super::PROB_BASE as i32));
        }
        a
    }

    #[test]
    fn action_cost_deterministic_neighbor() {
        // 2x1 マップ、θ=0。dix=+1 で隣 (free, total=5*PROB_BASE, penalty=PROB_BASE)。
        // cost = (5*PB + PB)*PB >>18 = 6*PB。
        let nt = 1usize;
        let nx = 2;
        let ny = 1;
        let states = vec![
            mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE),
            mk_state(1, 0, 0, true, 5 * super::PROB_BASE, super::PROB_BASE),
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, nx, ny, nt as i32);
        assert_eq!(c, 6 * super::PROB_BASE);
    }

    #[test]
    fn action_cost_out_of_map_returns_max() {
        let nt = 1usize;
        let states = vec![mk_state(0, 0, 0, true, 0, 0)];
        let a = single_action(-1, 0, 0, nt); // dix=-1 → 範囲外
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 1, 1, nt as i32);
        assert_eq!(c, super::MAX_COST);
    }

    #[test]
    fn action_cost_obstacle_neighbor_returns_max() {
        let nt = 1usize;
        let states = vec![
            mk_state(0, 0, 0, true, 0, 0),
            mk_state(1, 0, 0, false, 0, 0), // not free
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 2, 1, nt as i32);
        assert_eq!(c, super::MAX_COST);
    }

    #[test]
    fn action_cost_overflow_wraps() {
        // 未到達 free 隣接 (total=MAX_COST) → MAX_COST*PROB_BASE が u64 を折り返す。
        // 期待値: (MAX_COST + PROB_BASE) を PROB_BASE 倍して wrap し >>18。
        let nt = 1usize;
        let penalty = super::PROB_BASE;
        let states = vec![
            mk_state(0, 0, 0, true, super::MAX_COST, penalty),
            mk_state(1, 0, 0, true, super::MAX_COST, penalty),
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 2, 1, nt as i32);
        // 手計算: term = (MAX_COST + PROB_BASE) wrapping_mul PROB_BASE; result = term >> 18。
        let term = (super::MAX_COST.wrapping_add(penalty)).wrapping_mul(super::PROB_BASE);
        let expected = term >> super::PROB_BASE_BIT;
        assert_eq!(c, expected);
        // 折り返しにより MAX_COST 未満になることを確認 (固有挙動)。
        assert!(c < super::MAX_COST, "overflow wrap should yield value < MAX_COST, got {c}");
    }

    #[test]
    fn value_iteration_picks_min_and_records_action() {
        // 3x1 マップ θ=0。中央 (idx=1) から action0:dix=+1(隣 total=9), action1:dix=-1(隣 total=4)。
        let nt = 1usize;
        let nx = 3;
        let ny = 1;
        let mut states = vec![
            mk_state(0, 0, 0, true, 4 * super::PROB_BASE, super::PROB_BASE),
            mk_state(1, 0, 0, true, super::MAX_COST, super::PROB_BASE),
            mk_state(2, 0, 0, true, 9 * super::PROB_BASE, super::PROB_BASE),
        ];
        let a0 = single_action(1, 0, 0, nt); // → 右 (total=9)
        let a1 = single_action(-1, 0, 0, nt); // → 左 (total=4)
        let actions = vec![a0, a1];
        let mid = 1usize;
        let d = super::value_iteration_raw(&mut states, &actions, mid, nx, ny, nt as i32);
        // 左 (4*PB + PB)*PB >>18 = 5*PB。右 = 10*PB。min = 5*PB、action1。
        assert_eq!(states[mid].total_cost, 5 * super::PROB_BASE);
        assert_eq!(states[mid].optimal_action, Some(1));
        // delta = |5*PB - MAX_COST|
        assert_eq!(d, super::MAX_COST - 5 * super::PROB_BASE);
    }

    #[test]
    fn value_iteration_skips_final_and_obstacle() {
        let nt = 1usize;
        let mut s_final = mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE);
        s_final.final_state = true;
        let mut states = vec![s_final];
        let actions: Vec<Action> = vec![single_action(1, 0, 0, nt)];
        let d = super::value_iteration_raw(&mut states, &actions, 0, 1, 1, nt as i32);
        assert_eq!(d, 0);
        assert_eq!(states[0].total_cost, super::MAX_COST); // 未更新
    }

    #[test]
    fn set_goal_normalizes_theta() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(3, 3);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.0, 0.0, -10);
        assert_eq!(vi.goal_t, 350);
        vi.set_goal(0.0, 0.0, 370);
        assert_eq!(vi.goal_t, 10);
        assert_eq!(vi.status, "calculating");
    }

    #[test]
    fn set_state_values_pins_goal_cell() {
        // goal をグリッド角 (0.5,0.5) に置く。final_state は「セルの両角がゴール半径内」
        // を要求するため、角を共有する 4 セルの遠い角 (距離 √2*0.05≈0.0707m) を包む
        // R=0.08 を使う。margin_theta=360 で全θ許容。
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(20, 20); // res=0.05 → 範囲 1.0m
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.08, 360);
        vi.set_goal(0.5, 0.5, 0); // セル角 (10,10)=(0.5,0.5)
        // (ix=10,iy=10): 左下角=ゴール(r0=0)、右上角 r1=0.005 < 0.08^2=0.0064 → final。
        let idx = vi.to_index(10, 10, 0) as usize;
        assert!(vi.states[idx].final_state);
        assert_eq!(vi.states[idx].total_cost, 0);
        // 遠方セル (0,0) は距離 ≫ R → final でない。
        let far = vi.to_index(0, 0, 0) as usize;
        assert!(!vi.states[far].final_state);
        assert_eq!(vi.states[far].total_cost, super::MAX_COST);
    }
}
