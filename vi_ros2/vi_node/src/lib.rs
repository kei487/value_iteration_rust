//! vi_node library — modules consumed by main.rs and unit tests.
//!
//! Modules listed here are stubbed in this task and filled in by
//! Tasks 4–8. Keeping them as `pub mod` from the start lets us run
//! `cargo test -p vi_node --lib` after each task without churn.

pub mod bridge;
pub mod npy;
pub mod solver_factory;
pub mod sweep_thread;

// Oracle-equivalence tests live in the library (run via `cargo test --lib`) so
// they don't force cargo to build the rclrs `vi_node` binary (which links only
// via colcon). See `oracle.rs`.
#[cfg(test)]
mod oracle;
