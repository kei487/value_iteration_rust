//! Shared compute kernels used by all Solver variants.

pub mod bellman;
pub mod norm;

pub use bellman::bellman_backup;
pub use norm::bellman_backup_norm;
