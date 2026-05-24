use ndarray::{Array2, Array3};
use vi_core::{ActionIdx, Penalty, TransitionModel, Value};

#[derive(Clone, Debug)]
pub struct MapDims {
    pub map_x: u32,
    pub map_y: u32,
}

pub struct VIContext {
    pub dims: MapDims,
    pub value: Array3<Value>,
    pub penalty: Array2<Penalty>,
    pub goal_mask: Array3<bool>,
    pub transitions: TransitionModel,
}

impl VIContext {
    /// Clone with a fresh independent `value` table (other fields shared logically by clone).
    /// Used by benchmarks to run multiple solvers from the same initial state.
    pub fn clone_value(&self) -> Self {
        VIContext {
            dims: self.dims.clone(),
            value: self.value.clone(),
            penalty: self.penalty.clone(),
            goal_mask: self.goal_mask.clone(),
            transitions: self.transitions.clone(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Budget {
    /// Reference / block / pyramid sweep count.
    Sweeps(u32),
    /// Frontier iteration count.
    Iterations(u32),
}

#[derive(Debug)]
pub struct SolveStats {
    pub iters_or_sweeps: u32,
    pub updates: u64,
    pub final_delta: Value,
    pub converged: bool,
    pub extra: Option<SolveExtra>,
}

#[derive(Debug)]
pub enum SolveExtra {
    PyramidPerLevel(Vec<PyramidLevelStat>),
    ActionTable(Array3<ActionIdx>),
}

#[derive(Clone, Copy, Debug)]
pub struct PyramidLevelStat {
    pub level: u32,
    pub map_x: u32,
    pub map_y: u32,
    pub scale: u32,
    pub sweeps: u32,
    pub changed_states: u64,
    pub visited_states: u64,
    pub final_delta: Value,
}

pub trait Solver: Send + Sync {
    fn name(&self) -> &'static str;
    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats;
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::{Array2, Array3};
    use vi_core::params::N_THETA;

    fn make_test_ctx(map_x: u32, map_y: u32) -> VIContext {
        let mx = map_x as usize;
        let my = map_y as usize;
        VIContext {
            dims: MapDims { map_x, map_y },
            value: Array3::zeros((my, mx, N_THETA)),
            penalty: Array2::zeros((my, mx)),
            goal_mask: Array3::from_elem((my, mx, N_THETA), false),
            transitions: TransitionModel::default(),
        }
    }

    #[test]
    fn clone_value_independence() {
        let ctx = make_test_ctx(3, 3);
        let mut cloned = ctx.clone_value();
        cloned.value[[0, 0, 0]] = 42;
        assert_eq!(ctx.value[[0, 0, 0]], 0, "original must not be affected by mutation of clone");
        assert_eq!(cloned.value[[0, 0, 0]], 42);
    }
}
