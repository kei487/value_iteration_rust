use crate::types::{Penalty, Value};
use crate::params::{MAX_VALUE, PENALTY_OBSTACLE, PENALTY_GOAL, STEP_COST};

/// Compute traversal cost for one neighbor.
///
/// Matches `fpga/hls/stream/src/compute_row.cpp:cost_of()` and
/// `vi_matlab/src/fpga/stream/cost_of.m`.
///
/// Load-bearing invariant: `PENALTY_GOAL` is treated as 0 when read as a
/// neighbor's penalty — this keeps the goal cell's value pinned at 0.
#[inline]
pub fn cost_of(nv: Value, np_raw: Penalty) -> Value {
    if nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE {
        return MAX_VALUE;
    }
    let np: u32 = if np_raw == PENALTY_GOAL { 0 } else { np_raw as u32 };
    let s = nv as u32 + np + STEP_COST;
    if s >= MAX_VALUE as u32 { MAX_VALUE - 1 } else { s as Value }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{MAX_VALUE, PENALTY_OBSTACLE, PENALTY_GOAL};

    /// PENALTY_GOAL is substituted to 0: cost_of(5, PENALTY_GOAL) == 5 + 0 + 1 == 6
    #[test]
    fn penalty_goal_substituted_to_zero() {
        assert_eq!(cost_of(5, PENALTY_GOAL), 6);
    }

    /// PENALTY_OBSTACLE returns MAX_VALUE
    #[test]
    fn penalty_obstacle_returns_max_value() {
        assert_eq!(cost_of(5, PENALTY_OBSTACLE), MAX_VALUE);
    }

    /// MAX_VALUE neighbor returns MAX_VALUE
    #[test]
    fn max_value_neighbor_returns_max_value() {
        assert_eq!(cost_of(MAX_VALUE, 0), MAX_VALUE);
    }

    /// Overflow clamp returns MAX_VALUE - 1
    #[test]
    fn overflow_clamp_returns_max_value_minus_one() {
        // MAX_VALUE - 2 + 10 + 1 = MAX_VALUE + 9 => clamp to MAX_VALUE - 1
        assert_eq!(cost_of(MAX_VALUE - 2, 10), MAX_VALUE - 1);
    }

    /// Normal sum: cost_of(10, 5) == 16
    #[test]
    fn normal_sum() {
        assert_eq!(cost_of(10, 5), 16);
    }
}
