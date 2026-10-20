#!/usr/bin/env bash
# Run the native C++ Pinocchio benchmarks (pinocchio-cpp).
#
# Builds the binaries if the build directory does not exist, then runs fk_bench
# and jacobian_bench from the repo root. Results are written as Criterion-
# compatible estimates.json files so plot_bench.py picks them up automatically.
#
# Usage:
#   scripts/bench_pinocchio_cpp.sh          # build if needed, then run
#   scripts/bench_pinocchio_cpp.sh --build  # force a fresh cmake configure+build
#
# Prerequisites:
#   - Pinocchio 4.x installed via robotpkg (see docs/pinocchio.md)
#   - cmake >= 3.16
#   - LD_LIBRARY_PATH set, or /opt/openrobots/lib reachable at runtime

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$REPO_ROOT/benches/cpp/build"
PINOCCHIO_PREFIX="${PINOCCHIO_PREFIX:-/opt/openrobots}"

export LD_LIBRARY_PATH="$PINOCCHIO_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# --- Build ---
if [[ ! -f "$BUILD_DIR/fk_bench" ]] || [[ "${1:-}" == "--build" ]]; then
    echo "==> Configuring C++ benchmarks..."
    cmake -S "$REPO_ROOT/benches/cpp" -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="$PINOCCHIO_PREFIX"

    echo "==> Building C++ benchmarks..."
    cmake --build "$BUILD_DIR" -j"$(nproc)"
fi

# --- Run (from repo root so relative URDF and target/ paths resolve) ---
cd "$REPO_ROOT"

echo ""
echo "==> Running fk_bench..."
"$BUILD_DIR/fk_bench"

echo ""
echo "==> Running jacobian_bench..."
"$BUILD_DIR/jacobian_bench"

echo ""
echo "Done. Run 'python3 scripts/plot_bench.py' to regenerate charts."
