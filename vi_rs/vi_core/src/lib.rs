//! 16bit HLS データ契約のミラー: 型 (`types`) とアルゴリズム定数 (`params`)。
//!
//! u64 モデルの本体は `vi_reference` が独自に持つ。ここは `vi_bench`/`vi_ros2` が
//! HLS 契約値 (N_THETA, アクション表, Penalty センチネル) を参照・検証するための
//! 軽量な定数・型のみを公開する。コスト関数・遷移表・goal マスクの旧 u16 実装は
//! 唯一の利用者だった vi_fixtures とともに撤去した。
pub mod params;
pub mod types;

pub use types::{Value, Penalty, Offset, ThetaIdx, ActionIdx};
pub use params::{MAX_VALUE, N_THETA, N_ACTIONS, PROB_BASE,
                 PENALTY_OBSTACLE, PENALTY_GOAL, STEP_COST,
                 RESOLUTION_XY_BIT, RESOLUTION_T_BIT,
                 ACTION_FW, ACTION_ROT};
