/// Bechmarks the Jacobian computations.
use std::hint::black_box;

// Third-party
use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main};
use nalgebra::SMatrix;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{fixtures::BENCH_URDFS, load_urdf};

// ---- CONSTANTS ----
const RNG_SEED: u64 = 42;
const N_POSES: usize = 100;

/// Benchmarks a codegen'd `compute_link_jacobians` under the "galaw-generated" id.
fn bench_generated_jacobian<const N: usize, const M: usize>(
    group: &mut BenchmarkGroup<'_, WallTime>,
    bench_id: usize,
    joint_cmds: &[Vec<f64>],
    generated_compute_link_jacobians: impl Fn(&[f64; N]) -> [SMatrix<f64, 6, N>; M],
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
                    let out = generated_compute_link_jacobians(black_box(cmd));
                    black_box(out);
                }
            });
        },
    );
}

fn bench_jacobian(c: &mut Criterion) {
    for &urdf_path in BENCH_URDFS {
        let galaw_model = load_urdf(urdf_path).unwrap();
        let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();

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
                    .collect()
            })
            .collect();

        let mut group = c.benchmark_group(format!("jacobian/{}", galaw_model.name));
        group.throughput(criterion::Throughput::Elements(
            (joint_cmds.len() * galaw_model.links.len()) as u64,
        ));

        // ---- galaw-runtime ----
        group.bench_with_input(
            BenchmarkId::new("galaw-runtime", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                b.iter(|| {
                    for cmd in cmds {
                        let out = galaw_model.compute_link_jacobians(black_box(cmd)).unwrap();
                        black_box(out);
                    }
                });
            },
        );

        // ---- galaw-generated ----
        let mut generated_bench_registered = false;
        macro_rules! bench_if_matches {
            ($module:ident, $path:expr, $compute_fk:path) => {
                if urdf_path == $path {
                    bench_generated_jacobian(
                        &mut group,
                        galaw_model.joints.len(),
                        &joint_cmds,
                        galaw::generated::$module::compute_link_jacobians,
                    );
                    generated_bench_registered = true;
                }
            };
        }
        galaw::for_each_generated_robot!(bench_if_matches);
        assert!(
            generated_bench_registered,
            "no generated compute_link_jacobians registered for {urdf_path} — run scripts/codegen_all_urdfs.sh"
        );

        // ---- k ----
        group.bench_with_input(
            BenchmarkId::new("k", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                b.iter(|| {
                    for cmd in cmds {
                        k_chain.set_joint_positions(black_box(cmd)).unwrap();
                        k_chain.update_transforms();
                        for link in &galaw_model.links {
                            let node = k_chain.find_link(&link.name).unwrap();
                            let serial = k::SerialChain::from_end(node);
                            black_box(k::jacobian(&serial));
                        }
                    }
                });
            },
        );

        group.finish();
    }
}

criterion_group!(benches, bench_jacobian);
criterion_main!(benches);
