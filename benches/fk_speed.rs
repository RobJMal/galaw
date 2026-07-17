use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use galaw::load_urdf;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::fs;
use std::hint::black_box; // Prevents compiler from optimizing away code since we're benchmarking ("be pessimistic")
use sysinfo::System;

const RNG_SEED: u64 = 42;
const N_POSES: usize = 100; // Random poses per robot

// Robot embodiments to test
const URDFS: &[&str] = &[
    "assets/simple_robot.urdf",
    "assets/simple_arm_6dof.urdf",
    "assets/simple_arm_10dof.urdf",
    "assets/simple_arm_20dof.urdf",
];

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

    for &urdf_path in URDFS {
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
                    .map(|j| rng.random_range(j.limit_lower..j.limit_upper))
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
