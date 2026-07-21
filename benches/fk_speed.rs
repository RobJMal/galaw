use std::fs;
use std::hint::black_box; // Prevents compiler from optimizing away code since we're benchmarking ("be pessimistic")

// Third-Party
use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main};
use nalgebra::Isometry3;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use sysinfo::System;

// Custom
use galaw::{fixtures::BENCH_URDFS, load_urdf};

// ----- CONSTANTS -----
const RNG_SEED: u64 = 42;
const N_POSES: usize = 100; // Random poses per robot

/// Collects host/OS/CPU/memory info into a printable block, so benchmark
/// numbers can be reproduced on (or compared against) other machines.
fn system_specs() -> String {
    let mut sys = System::new_all();
    sys.refresh_all();

    // These are ASSOCIATED functions (System::...), each returning Option<String>.
    let host = System::host_name().unwrap_or_else(|| "unknown".into());
    let os = System::long_os_version().unwrap_or_else(|| "unknown".into());
    let kernel = System::kernel_version().unwrap_or_else(|| "unknown".into());

    // CPU brand + frequency come from the per-core list.
    let cpu = sys
        .cpus()
        .first()
        .map(|c| c.brand().to_string())
        .unwrap_or_else(|| "unknown".into());
    let freq_mhz = sys.cpus().first().map(|c| c.frequency()).unwrap_or(0);
    let logical_cores = sys.cpus().len();

    // total_memory() is BYTES in sysinfo 0.39 (older versions returned KB).
    let mem_gb = sys.total_memory() as f64 / 1e9;

    format!(
        "=== System specs ===\n\
         host:   {host}\n\
         os:     {os}\n\
         kernel: {kernel}\n\
         cpu:    {cpu} @ {freq_mhz} MHz ({logical_cores} logical cores)\n\
         memory: {mem_gb:.1} GB\n\
         rustflags: target-cpu=native\n\
         ===================="
    )
}

/// Benchmarks a codegen'd `compute_fk` under the "galaw-generated" id, in the
/// same `group` as the "galaw"/"k" entries the caller registers alongside it.
/// Generic over N/M since each robot's generated `compute_fk` bakes in a
/// different array size (`[f64; N] -> [Isometry3<f64>; M]`) - see the same
/// pattern in tests/fk_correctness.rs's `check_generated_matches_dynamic`.
fn bench_generated<const N: usize, const M: usize>(
    group: &mut BenchmarkGroup<'_, WallTime>,
    bench_id: usize,
    joint_cmds: &[Vec<f64>],
    generated_compute_fk: impl Fn(&[f64; N]) -> [Isometry3<f64>; M],
) {
    // Conversion to fixed-size arrays happens once, up front - not timed.
    let joint_cmds_arr: Vec<[f64; N]> = joint_cmds
        .iter()
        .map(|c| c.clone().try_into().unwrap())
        .collect();

    group.bench_with_input(
        BenchmarkId::new("galaw-generated", bench_id),
        &joint_cmds_arr,
        |b, cmds| {
            b.iter(|| {
                for cmd in cmds {
                    let out = generated_compute_fk(black_box(cmd));
                    black_box(out);
                }
            });
        },
    );
}

fn bench_fk(c: &mut Criterion) {
    // Capture machine context once, up front (not timed).
    let specs = system_specs();
    eprintln!("{specs}");

    // Save alongside Criterion's report so results are self-documenting.
    // Criterion creates target/criterion/ before benches run.
    let _ = fs::write("target/criterion/system-specs.txt", &specs);
    let _ = fs::write(
        "target/criterion/system-specs.html",
        format!("<pre>{specs}</pre>"),
    );

    for &urdf_path in BENCH_URDFS {
        // Setup is NOT timed
        let galaw_model = load_urdf(urdf_path).unwrap();
        let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();

        // Generate commands
        let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
        let joint_cmds: Vec<Vec<f64>> = (0..N_POSES)
            .map(|_| {
                galaw_model
                    .joints
                    .iter()
                    .filter(|j| j.cmd_idx.is_some())
                    .map(|j| match (j.limit_lower, j.limit_upper) {
                        (Some(lower), Some(upper)) => rng.random_range(lower..upper),
                        _ => rng.random_range(0.0..0.0),
                    })
                    // .map(|j| rng.random_range(j.limit_lower..j.limit_upper))
                    .collect()
            })
            .collect();

        // Group makes galaw vs k show up side-by-side
        let mut group = c.benchmark_group(format!("fk/{}", galaw_model.name));
        group.throughput(criterion::Throughput::Elements(joint_cmds.len() as u64));

        // ----- galaw -----
        group.bench_with_input(
            BenchmarkId::new("galaw", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                b.iter(|| {
                    for cmd in cmds {
                        let out = galaw_model.compute_fk(black_box(cmd)).unwrap();
                        black_box(out);
                    }
                });
            },
        );

        // ----- galaw-generated -----
        // for_each_generated_robot! (src/generated/registry.rs, auto-generated
        // by scripts/codegen_all_urdfs.sh) supplies the URDF path -> compute_fk
        // mapping, so there's nothing to hand-maintain here as robots are
        // added/removed - just rerun that script. `generated_bench_registered`
        // guards against BENCH_URDFS drifting ahead of the registry (a robot
        // added to fixtures.rs but not yet codegen'd would otherwise silently
        // skip its "galaw-generated" entry instead of failing loudly).
        let mut generated_bench_registered = false;
        macro_rules! bench_if_matches {
            ($module:ident, $path:expr, $compute_fk:path) => {
                if urdf_path == $path {
                    bench_generated(
                        &mut group,
                        galaw_model.joints.len(),
                        &joint_cmds,
                        $compute_fk,
                    );
                    generated_bench_registered = true;
                }
            };
        }
        galaw::for_each_generated_robot!(bench_if_matches);
        assert!(
            generated_bench_registered,
            "no generated compute_fk registered for {urdf_path} — run scripts/codegen_all_urdfs.sh"
        );

        // ----- k -----
        group.bench_with_input(
            BenchmarkId::new("k", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                b.iter(|| {
                    for cmd in cmds {
                        k_chain.set_joint_positions(black_box(cmd)).unwrap();
                        k_chain.update_transforms();
                    }
                });
            },
        );

        group.finish();
    }
}

criterion_group!(benches, bench_fk);
criterion_main!(benches);
