#!/usr/bin/env bash
# Run Rust u64 Value Iteration (vi_reference solvers) on a PGM/YAML map.
#
# Builds vi_bench::bench_map (release) and invokes it with sensible defaults.
# No ROS / Docker required.
#
# Examples:
#   ./scripts/run_vi_rust.sh                          # tiny map, frontier3d
#   ./scripts/run_vi_rust.sh --preset house           # house map (384×384)
#   ./scripts/run_vi_rust.sh --solver block_refine --scale 2
#   ./scripts/run_vi_rust.sh --map /path/to/map.yaml --goal-x 1.0 --goal-y 2.0
#   ./scripts/run_vi_rust.sh --out results/run.csv --dump-value /tmp/value.bin
#
# Environment overrides:
#   VI_SOLVER, VI_SCALE, VI_MAP, VI_OUT, CARGO_TARGET_DIR, VI_RS_DIR
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VI_RS_DIR="${VI_RS_DIR:-$REPO_ROOT/vi_rs}"
CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$VI_RS_DIR/target}"
BIN="$CARGO_TARGET_DIR/release/bench_map"

PRESET="tiny"
SOLVER="${VI_SOLVER:-frontier3d}"
SCALE="${VI_SCALE:-1}"
MAP="${VI_MAP:-}"
OUT="${VI_OUT:-}"
BUILD=1
MAP_WAS_SET=0
EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage: run_vi_rust.sh [OPTIONS] [-- BENCH_MAP_ARGS...]

Run vi_reference u64 Value Iteration via the bench_map CLI.

Options:
  -h, --help              Show this help
  --preset NAME           Map preset: tiny (default) | house
  --map PATH              Map YAML (overrides --preset)
  --solver NAME           U64Solver name (default: frontier3d)
  --scale N               Downsample factor (default: 1; meaningful range ~1–6)
  --out PATH              Write CSV timing summary
  --no-build              Skip cargo build (binary must already exist)
  --                      Pass remaining args to bench_map verbatim

Presets:
  tiny   vi_fpga/host/test/data/tiny.yaml (4×4, quick smoke test)
  house  vi_compare/.../maps/house.yaml (384×384, compare-bench map)

Solver names (U64Solver::from_name):
  reference, frontier3d, frontier2d, frontier2d_soa, frontier2d_pad,
  frontier2d_par, frontier2d_par_unsafe, frontier2d_fused, frontier2d_sparse,
  frontier_stack, block_refine, pyramid_sweep, stream_mimic,
  frontier2d_sparse_compact (use --compact-band / --compact-out-dir via -- passthrough),
  frontier3d_tau, frontier3d_topk, frontier3d_coarse_theta,
  prio_ls, prio_lc

Examples:
  ./scripts/run_vi_rust.sh
  ./scripts/run_vi_rust.sh --preset house --solver frontier3d
  ./scripts/run_vi_rust.sh --map mymap.yaml --solver reference --scale 3 \
      --goal-x 0 --goal-y 0 --safety-radius-m 0.2 --safety-penalty 100000
  ./scripts/run_vi_rust.sh -- --dump-value /tmp/v.bin --start-x 0 --start-y 0

Requires: cargo (Rust 1.75+), make optional.
EOF
}

resolve_preset() {
    case "$PRESET" in
        tiny)
            MAP="$REPO_ROOT/vi_fpga/host/test/data/tiny.yaml"
            PRESET_GOAL_X=0.10
            PRESET_GOAL_Y=0.10
            PRESET_GOAL_THETA=0
            PRESET_GOAL_RADIUS=0.10
            PRESET_SAFETY_RADIUS=0.0
            PRESET_SAFETY_PENALTY=30
            PRESET_GOAL_MARGIN_THETA=15
            ;;
        house)
            MAP="$REPO_ROOT/vi_compare/video/value_iteration_snap/maps/house.yaml"
            PRESET_GOAL_X=-0.425
            PRESET_GOAL_Y=-0.425
            PRESET_GOAL_THETA=0
            PRESET_GOAL_RADIUS=0.30
            PRESET_SAFETY_RADIUS=0.20
            PRESET_SAFETY_PENALTY=30
            PRESET_GOAL_MARGIN_THETA=15
            ;;
        *)
            echo "error: unknown preset '$PRESET' (use tiny or house)" >&2
            exit 2
            ;;
    esac
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --preset)
            PRESET="${2:?--preset requires a value}"
            shift 2
            ;;
        --map)
            MAP="${2:?--map requires a path}"
            MAP_WAS_SET=1
            shift 2
            ;;
        --solver)
            SOLVER="${2:?--solver requires a name}"
            shift 2
            ;;
        --scale)
            SCALE="${2:?--scale requires a number}"
            shift 2
            ;;
        --out)
            OUT="${2:?--out requires a path}"
            shift 2
            ;;
        --no-build)
            BUILD=0
            shift
            ;;
        --)
            shift
            EXTRA_ARGS=("$@")
            break
            ;;
        -*)
            echo "error: unknown option '$1' (try --help)" >&2
            exit 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ -z "$MAP" ]]; then
    if [[ -n "${VI_MAP:-}" ]]; then
        MAP="$VI_MAP"
        MAP_WAS_SET=1
    else
        resolve_preset
    fi
fi

if [[ ! -f "$MAP" ]]; then
    echo "error: map not found: $MAP" >&2
    exit 2
fi

if ! command -v cargo >/dev/null 2>&1; then
    echo "error: cargo not found (install Rust 1.75+ or set PATH)" >&2
    exit 127
fi

if [[ "$BUILD" -eq 1 ]]; then
    echo "== building bench_map (release) =="
    cargo build --release --manifest-path "$VI_RS_DIR/Cargo.toml" \
        -p vi_bench --bin bench_map
fi

if [[ ! -x "$BIN" ]]; then
    echo "error: bench_map binary missing: $BIN" >&2
    exit 1
fi

BENCH_ARGS=(
    --map "$MAP"
    --scale "$SCALE"
    --solver-name "$SOLVER"
)

# Apply preset goal/planning params when the map came from a preset (not --map / VI_MAP).
# Use =value form so negative coordinates are not parsed as flags by clap.
if [[ "$MAP_WAS_SET" -eq 0 && -n "${PRESET_GOAL_X:-}" ]]; then
    BENCH_ARGS+=(--goal-x="$PRESET_GOAL_X" --goal-y="$PRESET_GOAL_Y")
    BENCH_ARGS+=(--goal-theta-deg="$PRESET_GOAL_THETA")
    BENCH_ARGS+=(--goal-radius-m="$PRESET_GOAL_RADIUS")
    BENCH_ARGS+=(--safety-radius-m="$PRESET_SAFETY_RADIUS")
    BENCH_ARGS+=(--safety-penalty="$PRESET_SAFETY_PENALTY")
    BENCH_ARGS+=(--goal-margin-theta-deg="$PRESET_GOAL_MARGIN_THETA")
fi

if [[ -n "$OUT" ]]; then
    mkdir -p "$(dirname "$OUT")"
    BENCH_ARGS+=(--out "$OUT")
fi

echo "== running Value Iteration =="
echo "   map:    $MAP"
echo "   solver: $SOLVER"
echo "   scale:  $SCALE"
echo ""

exec "$BIN" "${BENCH_ARGS[@]}" "${EXTRA_ARGS[@]}"
