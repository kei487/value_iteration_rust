//! (A2) Priority Label-Correcting — 厳密・本家と bit-exact。
//! `priority_solve(label_setting=false)` の薄ラッパ。

use crate::value_iterator::ValueIterator;

pub fn prio_lc_solve(_vi: &mut ValueIterator, _max_iter: u32) -> (u32, u64, bool) {
    unimplemented!("Task 3 で実装")
}

#[cfg(test)]
mod tests {
    use super::prio_lc_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_prio_lc() {
        // empty / obstacle / sentinel の3マップで到達セル bit-exact（値＋方策）。
        parity_standard_maps(|vi| prio_lc_solve(vi, 3000));
    }
}
