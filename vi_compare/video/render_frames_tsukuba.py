#!/usr/bin/env python3
"""tsukuba フル (4417×2367×60 = 627M 状態) の ROS1 vs vi_rs sparse 並列スイープ動画。

render_frames_tsudanuma.py (津田沼) の tsukuba 版。違い:
  - グリッド 4417×2367 (0.15 m/cell, x3 pool)、origin (-553.84, -60.609)。津田沼と同解像度。
  - goal world(20.5, -1.0, 0deg) → pooled cell (3828, iy=397)。goal_margin_theta=15・
    goal_radius=0.30 (津田沼と同設定)。0.15 m では goal mask=28 セルあり孤立しない
    (0.25 m 版で margin=180 にしていた回避策は不要)。
  - vi_rs sparse は ~6.9 s で厳密収束、ROS1 はこの規模では収束せず TIMEOUT(600 s)
    まで未収束。フェーズ構成: intro → real-time → TIMELAPSE ×40 → end card。
  - フレームは 41.8 MB/枚 なので全ロードせず逐次読み (アクセスは単調)。

タイムライン: vi_rs はスナップショット時刻 (ダンプ時間除外済) のクリーン計測を
そのまま使う (last ≈ 6.9 s)。ROS1 は収束しないので計測ランの生 wall-clock を使う。
"""
import os
import glob
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.environ.get('VI_ROOT', '/work')
RES = f'{ROOT}/vi_compare/results/tsukuba'
OUT = f'{RES}/video_frames'
PGM = f'{RES}/map_tsukuba_pooled.pgm'
NX, NY = 4417, 2367
FPS = 30
LAPSE = 40.0            # phase2 の倍率
GARBAGE = 1.0e6
INTRO_SEC, END_SEC = 3.0, 4.0
GX, GY = 3828, 397      # goal pooled cell (bottom-up iy), world(20.5,-1.0)/res0.15/origin(-553.84,-60.609)


class Side:
    """snap_NNNNN.bin 列の逐次リーダ (単調アクセス前提、1 枚キャッシュ)。"""

    def __init__(self, d, scale=1.0):
        self.files = sorted(glob.glob(f'{d}/snap_*.bin'))
        ts, rounds = [], []
        with open(f'{d}/times.csv') as f:
            next(f)
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 3:
                    continue
                i, t, r = parts[0], parts[1], parts[2]
                ts.append(float(t) * scale)
                rounds.append(int(r))
        n = min(len(self.files), len(ts))
        self.files, self.ts, self.rounds = self.files[:n], np.array(ts[:n]), rounds[:n]
        self.cache_i = -1
        self.cache = None

    def at(self, t):
        """時刻 t 直前のフレーム (無ければ None) と round。"""
        i = int(np.searchsorted(self.ts, t, side='right')) - 1
        if i < 0:
            return None, 0
        if i != self.cache_i:
            self.cache = np.fromfile(self.files[i], dtype='<f4').reshape(NY, NX)
            self.cache_i = i
        return self.cache, self.rounds[i]

    def last(self):
        return self.at(self.ts[-1] + 1)[0]


def main():
    os.makedirs(OUT, exist_ok=True)
    with open(PGM, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        pgm = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # tsukuba pooled: obstacle=0, free=255。obstacle を壁色に。
    occ_wall = np.flipud(pgm < 128)

    L = Side(f'{RES}/frames_ros1')
    R = Side(f'{RES}/frames_sparse')
    T_SPARSE = float(R.ts[-1])          # vi_rs clean wall-clock (収束時刻)
    T_PHASE1 = T_SPARSE + 1.1           # real-time 区間: 収束を見せてから timelapse へ
    # ROS1 の VI 計算 wall-clock (snapshotWorker 時計; セットアップ除外, vi_rs と同条件)。
    T_END = float(np.ceil(L.ts[-1]))
    meta = json.load(open(f'{RES}/snap_run/ros1_m16.json'))
    resid_s = (meta.get('last_max_delta') or 0.0) / 262144.0
    n_round_r = R.rounds[-1]
    print(f'L(ROS1) frames={len(L.files)} (last t={L.ts[-1]:.0f}s) R(vi_rs) frames={len(R.files)} '
          f'T_SPARSE={T_SPARSE:.2f}s resid={resid_s:.0f}s rounds={n_round_r}')

    final_r = R.last().copy()
    fin = np.isfinite(final_r)
    # tsukuba は safety-penalty セル (V>1e5 s) が混じるので、通常走行域 (<5000 s) の
    # P99 でスケールし penalty 域は赤に飽和させる。
    vals = final_r[fin]
    drive = vals[vals < 5000.0]
    vmax = float(np.percentile(drive, 99.0)) if drive.size else float(np.percentile(vals, 90.0))
    print(f'vmax={vmax:.1f}s  reachable={int(fin.sum())}')
    cmap = plt.get_cmap('turbo')

    wall_rgb = np.float32((0.55, 0.55, 0.58))

    def render_field(a):
        img = np.zeros((NY, NX, 3), dtype=np.float32)
        img[:] = (0.06, 0.06, 0.09)
        img[occ_wall] = wall_rgb
        if a is not None:
            reached = np.isfinite(a) & (a < GARBAGE)
            v = np.sqrt(np.clip(a[reached] / vmax, 0, 1))
            img[reached] = cmap(v)[:, :3].astype(np.float32)
        return img

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='#101014')
    axL = fig.add_axes([0.025, 0.10, 0.46, 0.72])
    axR = fig.add_axes([0.515, 0.10, 0.46, 0.72])
    for ax in (axL, axR):
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#444')
    imL = axL.imshow(render_field(None), origin='lower', interpolation='nearest')
    imR = axR.imshow(render_field(None), origin='lower', interpolation='nearest')
    for ax in (axL, axR):
        ax.plot(GX, GY, marker='*', ms=16, mfc='white', mec='black', mew=1.0, zorder=5)

    fig.text(0.5, 0.955, 'Value Iteration sweep — ROS1 (C++) vs vi_rs (Rust), 16 threads each',
             ha='center', color='white', fontsize=25, fontweight='bold')
    fig.text(0.5, 0.915,
             'Tsukuba campus 4417×2367×60 = 627M states (0.15 m/cell) · identical problem '
             '· goal (20.5, -1.0, 0°)',
             ha='center', color='#aaaaaa', fontsize=14)
    fig.text(0.26, 0.86, 'ROS1 value_iteration  (C++, 16 threads)',
             ha='center', color='#ff9966', fontsize=16, fontweight='bold')
    fig.text(0.74, 0.86, 'vi_rs frontier2d_sparse  (Rust, 16 threads)',
             ha='center', color='#66ccff', fontsize=16, fontweight='bold')

    timerL = fig.text(0.26, 0.05, '', ha='center', color='white', fontsize=27,
                      family='monospace', fontweight='bold')
    timerR = fig.text(0.74, 0.05, '', ha='center', color='white', fontsize=27,
                      family='monospace', fontweight='bold')
    lapse = fig.text(0.5, 0.86, '', ha='center', color='#ffee66', fontsize=20,
                     fontweight='bold')
    fig.text(0.5, 0.012,
             'colors: cost-to-go (s) · both timelines = measured wall clock · '
             'ROS1 stop criterion never reached on this map '
             '(vi_rs reaches the exact fixed point)',
             ha='center', color='#777777', fontsize=11)
    banner = fig.text(0.5, 0.5, '', ha='center', va='center', color='#ffee66',
                      fontsize=34, fontweight='bold',
                      bbox=dict(boxstyle='round,pad=0.6', fc='#101014', ec='#ffee66',
                                lw=2, alpha=0.93))
    banner.set_visible(False)
    intro = fig.text(0.5, 0.52, '', ha='center', va='center', color='white', fontsize=26)

    frame_no = 0

    def save():
        nonlocal frame_no
        fig.savefig(f'{OUT}/frame_{frame_no:05d}.png', facecolor=fig.get_facecolor())
        frame_no += 1

    # --- intro ---
    intro.set_text('Same 3-D (x, y, θ) value iteration, same 16-thread budget.\n\n'
                   'Left:  original ROS1 node — full-grid sweeps\n'
                   'Right: vi_rs sparse solver — frontier + θ-mask evaluation\n\n'
                   'First segment plays in REAL TIME.')
    timerL.set_text('t =   0.0 s')
    timerR.set_text('t =   0.0 s')
    for _ in range(int(INTRO_SEC * FPS)):
        save()
    intro.set_visible(False)

    # --- main: phase1 real-time, phase2 timelapse ---
    k1 = int(T_PHASE1 * FPS)
    n2 = int(np.ceil((T_END - T_PHASE1) / LAPSE * FPS))
    for k in range(k1 + n2 + 1):
        if k <= k1:
            t = k / FPS
            if k == 0:
                lapse.set_text('REAL TIME')
        else:
            t = min(T_PHASE1 + (k - k1) / FPS * LAPSE, T_END)
            lapse.set_text(f'TIMELAPSE ×{int(LAPSE)}')
            lapse.set_color('#ff6666')
        # left: ROS1, ずっと未収束
        fl, rl = L.at(t)
        imL.set_data(render_field(fl))
        timerL.set_text(f't = {t:5.1f} s   sweep {rl}')
        # right: sparse
        if t >= T_SPARSE:
            imR.set_data(render_field(final_r))
            timerR.set_text(f'CONVERGED (exact)  {T_SPARSE:.1f} s')
            timerR.set_color('#66ff88')
        else:
            fr, rr = R.at(t)
            imR.set_data(render_field(fr))
            timerR.set_text(f't = {t:5.1f} s   round {rr}/{n_round_r}')
        save()

    # --- end card ---
    timerL.set_text(f'NOT CONVERGED after {int(T_END)} s   (ΔV ≈ {resid_s:.0f} s)')
    timerL.set_color('#ff6666')
    banner.set_visible(True)
    banner.set_text(f'vi_rs: exact fixed point in {T_SPARSE:.1f} s\n'
                    f'ROS1: still ΔV ≈ {resid_s:.0f} s after {int(T_END)} s\n'
                    f'→  ≥ {int(T_END / T_SPARSE)}× faster on this map')
    for _ in range(int(END_SEC * FPS)):
        save()
    print(f'wrote {frame_no} frames to {OUT}')


if __name__ == '__main__':
    main()
