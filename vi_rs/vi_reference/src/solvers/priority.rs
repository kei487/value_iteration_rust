//! VI を SSP と捉えた優先順序伝播ソルバの共有基盤。本家 per-cell 更新
//! `value_iteration_raw` を「値の昇順」に呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と一致（厳密版 prio_lc）。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-ssp-priority-acceleration-design.md`

use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// 逆θ写像。`rev[it']` = 確定セル `(.., it')` の前駆を列挙する `(dix, diy, t_src)` 列。
/// 全 (action, source θ `t`, 遷移 `δ`) を走査し、着地 θ `it' = (dit + nt) % nt` をキーに
/// `(dix, diy, t)` を積む。前駆は `(ix' - dix, iy' - diy, t)`。重複は dedup（過剰列挙抑制）。
pub(crate) fn build_rev_theta(vi: &ValueIterator) -> Vec<Vec<(i32, i32, i32)>> {
    let nt = vi.cell_num_t;
    let mut rev: Vec<Vec<(i32, i32, i32)>> = vec![Vec::new(); nt as usize];
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                let itp = (((st.dit % nt) + nt) % nt) as usize;
                rev[itp].push((st.dix, st.diy, t as i32));
            }
        }
    }
    for bucket in rev.iter_mut() {
        bucket.sort_unstable();
        bucket.dedup();
    }
    rev
}

/// セル `idx` を本家 Bellman で再評価・書込。改善（厳密減少）したら新ラベルを返す。
#[inline]
pub(crate) fn relax_cell(
    vi: &mut ValueIterator,
    idx: usize,
    nx: i32,
    ny: i32,
    nt: i32,
) -> Option<u64> {
    let before = vi.states[idx].total_cost;
    value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
    let after = vi.states[idx].total_cost;
    if after < before {
        Some(after)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::test_support::make_vi;

    #[test]
    fn rev_theta_round_trips_forward_transitions() {
        // 全 (action, θ, 遷移) について、着地θのバケットに (dix,diy,t) が含まれること。
        let vi = make_vi(8, 8, vec![0i8; 64]);
        let rev = build_rev_theta(&vi);
        let nt = vi.cell_num_t;
        assert_eq!(rev.len(), nt as usize);
        for a in &vi.actions {
            for (t, trans) in a.state_transitions.iter().enumerate() {
                for st in trans {
                    let itp = (((st.dit % nt) + nt) % nt) as usize;
                    assert!(
                        rev[itp].contains(&(st.dix, st.diy, t as i32)),
                        "rev[{itp}] must contain ({},{},{})",
                        st.dix,
                        st.diy,
                        t
                    );
                }
            }
        }
        // dedup 済み（各バケットは昇順ユニーク）。
        for bucket in &rev {
            let mut sorted = bucket.clone();
            sorted.sort_unstable();
            sorted.dedup();
            assert_eq!(&sorted, bucket);
        }
    }
}
