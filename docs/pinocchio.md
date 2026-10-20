# Benchmarking against Pinocchio

galaw's benchmarks can include [Pinocchio](https://github.com/stack-of-tasks/pinocchio) as a comparison target via the `bench-pinocchio` feature. Pinocchio is a C++ kinematics/dynamics library; the Rust side uses [`rs-pinocchio`](https://crates.io/crates/rs-pinocchio), which builds an FFI wrapper from vendored source and links against the system-installed Pinocchio headers and shared libraries.

## System prerequisites

### Install Pinocchio via robotpkg

robotpkg is the recommended distribution. The apt repository needs to be added manually; it is not in the default Ubuntu sources.

```bash
# 1. Add the robotpkg GPG key
sudo apt install -qqy lsb-release gnupg2 curl
curl -sSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo gpg --dearmor -o /usr/share/keyrings/robotpkg.gpg

# 2. Add the apt repository (adjust the suite if you are not on jammy)
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/robotpkg.gpg] \
http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list

# 3. Install Pinocchio 4.x and its header/cmake files
sudo apt update
sudo apt install robotpkg-py3*-pinocchio    # Python bindings optional
sudo apt install robotpkg-pinocchio         # headers + shared libs
```

robotpkg installs everything under `/opt/openrobots`.

### Set environment variables

The linker and `pkg-config` need to find the robotpkg prefix at build *and* runtime:

```bash
export PINOCCHIO_PREFIX=/opt/openrobots
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
```

Add these lines to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.) to make them permanent.

## Running the benchmarks

Pass the `bench-pinocchio` feature flag to the FK and Jacobian benchmarks:

```bash
# Both supported benchmarks
cargo bench --features bench-pinocchio

# Individual benchmarks
cargo bench --bench fk_speed       --features bench-pinocchio
cargo bench --bench jacobian_speed --features bench-pinocchio
```

Without the feature flag the Pinocchio bench arm is compiled out and the benchmarks run with only galaw and `k` as comparison targets. The IK benchmark does not have a Pinocchio arm.

## Native C++ benchmark (pinocchio-cpp)

`benches/cpp/` contains standalone C++ benchmarks that call Pinocchio directly — no Rust FFI. These exist to verify that the `rs-pinocchio` (FFI) numbers accurately reflect native C++ Pinocchio performance.

### Prerequisites

In addition to Pinocchio itself (see above), you need CMake ≥ 3.16:

```bash
sudo apt install cmake
```

### Build

```bash
# Configure (from repo root)
cmake -S benches/cpp -B benches/cpp/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/openrobots

# Build
cmake --build benches/cpp/build -j$(nproc)
```

### Run

Run from the **repo root** so URDF paths and `target/criterion/` output paths resolve correctly:

```bash
LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH ./benches/cpp/build/fk_bench
LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH ./benches/cpp/build/jacobian_bench
```

Each binary writes Criterion-compatible `estimates.json` files under `target/criterion/` as the `pinocchio-cpp` implementation. Running `cargo run --release --example plot_bench` afterwards will automatically include the `pinocchio-cpp` series in the charts.

### Design notes

- **Build flags**: `-O3 -DNDEBUG`, no `-march=native` — matches the robotpkg install flags exactly.
- **FK per iteration**: `forwardKinematics` + `updateFramePlacements` + one `data.oMf[fid]` read per BODY frame.
- **Jacobian per iteration**: `computeJointJacobians` + one `getFrameJacobian(LOCAL_WORLD_ALIGNED)` per BODY frame. A single `Matrix6x` is pre-allocated outside the loop and reused, unlike `rs-pinocchio` which allocates a fresh matrix on every `frame_jacobian()` call. This represents achievable native C++ performance.
- **Config generation**: `std::mt19937(42)`, same seed and joint-limit distribution as the Rust bench. Exact values differ (MT19937 vs ChaCha8) but FK/Jacobian timing is insensitive to specific angle values.

## rs-pinocchio implementation notes

- `rs-pinocchio` builds its FFI bridge from vendored C++ source at compile time (via `cxx`). The first build is slow.
- Each FK benchmark iteration calls `forward_kinematics(&q)` + `update_frame_placements()` + one `frame_placement(fid)` per link.
- Each Jacobian benchmark iteration calls `compute_joint_jacobians(&q)` + one `frame_jacobian(fid, LocalWorldAligned)` per link. Each `frame_jacobian` call allocates a new `Matrix6x` in C++ and a `nalgebra::DMatrix` on the Rust side.
- Pinocchio's configuration dimension (`nq`) may exceed galaw's commanded joint count when a URDF contains non-commanded joints (e.g. continuous joints without limits). The bench code pads the `DVector<f64>` to `pin_model.nq()` with zeros for those slots.
- cxx FFI call overhead is roughly 1–5 ns per call. At measured Pinocchio latencies of 1,000–10,000 ns this is well under 1% and does not materially affect the comparison.
