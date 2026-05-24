//! Shared compute kernels used by all Solver variants.

pub mod bellman;

pub use bellman::bellman_backup;
