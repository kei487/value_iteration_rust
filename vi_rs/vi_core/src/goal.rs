use crate::params::N_THETA;
use ndarray::Array3;

pub struct GoalSpec {
    pub xy_resolution: f64,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_theta_deg: f64,
    pub goal_radius_m: f64,
    pub goal_margin_theta_deg: f64,
}

pub fn make_goal_mask(map_x: u32, map_y: u32, spec: &GoalSpec) -> Array3<bool> {
    let mx = map_x as usize;
    let my = map_y as usize;
    let mut mask = Array3::from_elem((my, mx, N_THETA), false);
    let t_resolution = 360.0 / N_THETA as f64;
    let r2_thresh = spec.goal_radius_m * spec.goal_radius_m;

    for iy in 0..my {
        for ix in 0..mx {
            let x0 = ix as f64 * spec.xy_resolution + spec.map_origin_x;
            let y0 = iy as f64 * spec.xy_resolution + spec.map_origin_y;
            let x1 = x0 + spec.xy_resolution;
            let y1 = y0 + spec.xy_resolution;

            let r0 = (x0 - spec.goal_x).powi(2) + (y0 - spec.goal_y).powi(2);
            let r1 = (x1 - spec.goal_x).powi(2) + (y1 - spec.goal_y).powi(2);
            if !(r0 < r2_thresh && r1 < r2_thresh) {
                continue;
            }

            // WHY: wrapped_goal is always the 360-offset counterpart, per MATLAB lines 35-40.
            // It never equals goal_theta_deg itself — it's the wrap-around alias.
            let wrapped_goal = if spec.goal_theta_deg > 180.0 {
                spec.goal_theta_deg - 360.0
            } else {
                spec.goal_theta_deg + 360.0
            };
            let margin = spec.goal_margin_theta_deg;

            for it in 0..N_THETA {
                let t0 = it as f64 * t_resolution;
                let t1 = (it + 1) as f64 * t_resolution;
                let in_theta = (spec.goal_theta_deg - margin <= t0 && t1 <= spec.goal_theta_deg + margin)
                    || (wrapped_goal - margin <= t0 && t1 <= wrapped_goal + margin);
                if in_theta {
                    mask[[iy, ix, it]] = true;
                }
            }
        }
    }
    mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_mask_for_distant_goal() {
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 10.0,
            goal_y: 10.0,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let mask = make_goal_mask(4, 4, &spec);
        assert!(mask.iter().all(|&v| !v));
    }

    #[test]
    fn goal_at_center_marks_some_cells() {
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.20,
            goal_y: 0.20,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let mask = make_goal_mask(8, 8, &spec);
        assert!(mask.iter().any(|&v| v));
    }

    #[test]
    fn mask_cells_are_inside_disk() {
        let spec = GoalSpec {
            xy_resolution: 0.05,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            goal_x: 0.20,
            goal_y: 0.20,
            goal_theta_deg: 90.0,
            goal_radius_m: 0.30,
            goal_margin_theta_deg: 15.0,
        };
        let map_x: u32 = 8;
        let map_y: u32 = 8;
        let mask = make_goal_mask(map_x, map_y, &spec);
        let r2_thresh = spec.goal_radius_m * spec.goal_radius_m;

        for iy in 0..map_y as usize {
            for ix in 0..map_x as usize {
                for it in 0..N_THETA {
                    if mask[[iy, ix, it]] {
                        let x0 = ix as f64 * spec.xy_resolution + spec.map_origin_x;
                        let y0 = iy as f64 * spec.xy_resolution + spec.map_origin_y;
                        let x1 = x0 + spec.xy_resolution;
                        let y1 = y0 + spec.xy_resolution;
                        let r0 = (x0 - spec.goal_x).powi(2) + (y0 - spec.goal_y).powi(2);
                        let r1 = (x1 - spec.goal_x).powi(2) + (y1 - spec.goal_y).powi(2);
                        assert!(r0 < r2_thresh, "r0={r0} not < {r2_thresh} at ix={ix} iy={iy} it={it}");
                        assert!(r1 < r2_thresh, "r1={r1} not < {r2_thresh} at ix={ix} iy={iy} it={it}");
                    }
                }
            }
        }
    }
}
