#!/usr/bin/env python3
"""bench_map の `frontier2d_sparse_compact` ディスク出力を value/policy 図にする。

`bench_map --solver frontier2d_sparse_compact --compact-out-dir <dir> [--dump-value <dir>/value_field.f32]`
が出す以下を読む（索引は本家 toIndex と同じ `orig = it + ix*nt + iy*nt*nx`）:

  value_field.f32    : --dump-value の出力。header i32(ow), i32(oh) + ow*oh の f32
                       (θ最小の value[秒], 到達不能=NaN)。value 図はこれだけで描ける。
  compact_value.bin  : nstates × u64 LE。値は PROB_BASE 単位 (262144 = 1 秒)、
                       1e9*PROB_BASE = 到達不能。
  compact_action.bin : nstates × i32 LE。-1 = None(ゴール/到達不能)、0..5 = action 索引
                       (0 forward / 1 back / 2 right / 3 rightfw / 4 left / 5 leftfw)。

policy パネル(右)は生 bin が要る。無ければ value パネルのみ描く。grid 次元 ow×oh は
value_field.f32 のヘッダから取り、θ数 nt は --nt(既定 60)。

使い方:
  python3 render_compact.py <out_dir> [--nt 60] [--out <out_dir>/compact_viz.png] [--no-policy]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, LogNorm

PROB_BASE = 262144  # 18bit 固定小数 = 1 秒
ACTION_NAMES = ["forward", "back", "right", "rightfw", "left", "leftfw"]


def load_value_field(path):
    """value_field.f32 (header i32 ow, i32 oh + ow*oh f32) を (oh, ow) で返す。"""
    with open(path, "rb") as fh:
        ow, oh = (int(v) for v in np.fromfile(fh, dtype="<i4", count=2))
        vf = np.fromfile(fh, dtype="<f4", count=ow * oh).reshape(oh, ow)
    return vf, ow, oh


def best_theta_policy(out_dir, ow, oh, nt):
    """compact_value/action.bin から各セルの最良 θ の action (-1..5) を (oh, ow) で返す。"""
    val = np.fromfile(f"{out_dir}/compact_value.bin", dtype="<u8").reshape(oh, ow, nt)
    act = np.fromfile(f"{out_dir}/compact_action.bin", dtype="<i4").reshape(oh, ow, nt)
    best = val.argmin(axis=2)
    iy, ix = np.indices((oh, ow))
    return act[iy, ix, best]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir", help="bench_map --compact-out-dir に渡したディレクトリ")
    ap.add_argument("--nt", type=int, default=60, help="θ セル数 (既定 60)")
    ap.add_argument("--out", default=None, help="出力 PNG (既定 <out_dir>/compact_viz.png)")
    ap.add_argument("--no-policy", action="store_true", help="value パネルのみ (bin を読まない)")
    ap.add_argument("--title", default=None, help="図タイトル")
    args = ap.parse_args()

    vf_path = f"{args.out_dir}/value_field.f32"
    if not os.path.exists(vf_path):
        sys.exit(f"error: {vf_path} がありません。bench_map に --dump-value {vf_path} を付けて実行してください。")
    vf, ow, oh = load_value_field(vf_path)
    reach = np.isfinite(vf)
    print(f"grid {ow}x{oh}x{args.nt}  reachable cells {int(reach.sum())}  Vmax={np.nanmax(vf):.1f}s")

    have_policy = (not args.no_policy) and all(
        os.path.exists(f"{args.out_dir}/{n}") for n in ("compact_value.bin", "compact_action.bin")
    )
    pol = best_theta_policy(args.out_dir, ow, oh, args.nt) if have_policy else None

    ncol = 2 if pol is not None else 1
    fig, axes = plt.subplots(1, ncol, figsize=(9.5 * ncol, 7), constrained_layout=True, squeeze=False)
    a1 = axes[0][0]

    # value: ダイナミックレンジが広い (1s〜1e6s, 壁沿い safety-penalty) ので対数表示。
    vcmap = plt.cm.turbo.copy()
    vcmap.set_over("magenta")  # penalty 壁 (V>1e5s)
    vcmap.set_bad("0.15")      # 到達不能
    im = a1.imshow(np.ma.masked_invalid(vf), origin="lower", cmap=vcmap, norm=LogNorm(vmin=1.0, vmax=1e5))
    a1.set_facecolor("0.15")
    a1.set_title("value  V*  (min over θ)  [s, log]   (magenta = penalty wall >1e5s)")
    fig.colorbar(im, ax=a1, shrink=0.82, label="seconds-to-goal (log)", extend="max")

    if pol is not None:
        a2 = axes[0][1]
        pol = np.where(reach, pol, -1)
        cmap = ListedColormap(["0.15", "#e6194b", "#42d4f4", "#3cb44b", "#4363d8", "#f58231", "#911eb4"])
        im2 = a2.imshow(np.where(pol < 0, 0, pol + 1), origin="lower", cmap=cmap, vmin=0, vmax=6)
        a2.set_facecolor("0.15")
        a2.set_title("optimal action  (policy at best θ)")
        cb = fig.colorbar(im2, ax=a2, shrink=0.82, ticks=[i + 0.5 for i in range(7)])
        cb.ax.set_yticklabels(["(none)"] + ACTION_NAMES)

    for row in axes:
        for a in row:
            a.set_xlabel("x cell")
            a.set_ylabel("y cell")
    title = args.title or f"compact mapped output · {ow}×{oh}×{args.nt} = {ow * oh * args.nt / 1e6:.0f}M states"
    fig.suptitle(title, fontsize=13)

    out_png = args.out or f"{args.out_dir}/compact_viz.png"
    fig.savefig(out_png, dpi=110)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
