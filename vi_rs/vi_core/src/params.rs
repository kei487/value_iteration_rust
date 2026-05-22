use crate::types::Value;

pub const N_ACTIONS: usize = 6;
pub const N_THETA: usize = 60;
pub const MAX_VALUE: Value = 0xFFFF;
pub const PENALTY_OBSTACLE: u16 = 0xFFFF;
pub const PENALTY_GOAL: u16 = 0xFFFE;
pub const STEP_COST: u32 = 1;
pub const PROB_BASE: u32 = 262_144;
pub const MAX_OUTCOMES: usize = 10;
pub const TRANS_WORD_STRIDE: usize = 21;
pub const TRANS_TABLE_SIZE: usize = 7_560;

pub const ACTION_FW: [f64; N_ACTIONS] = [0.3, -0.2, 0.0, 0.2, 0.0, 0.2];
pub const ACTION_ROT: [f64; N_ACTIONS] = [0.0, 0.0, -20.0, -20.0, 20.0, 20.0];
