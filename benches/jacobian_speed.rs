/// Bechmarks the Jacobian computations.
use std::hint::black_box;

// Third-party
use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{fixtures::BENCH_URDFS, load_urdf};
use nalgebra::{Matrix6xX, SMatrix};

// ---- CONSTANTS ----
const RNG_SEED: u64 = 42;
const N_POSES: usize = 100;

/// Benchmarks a codegen'd `compute_link_jacobians` under the given id.
/// Generic over FloatType, NUM_JOINTS (DOF count), and NUM_LINKS (link count).
fn bench_generated_jacobian<
    FloatType: nalgebra::RealField + Copy + Clone + std::fmt::Debug + 'static,
    const NUM_JOINTS: usize,
    const NUM_LINKS: usize,
>(
    group: &mut BenchmarkGroup<'_, WallTime>,
    bench_id_label: &str,
    bench_id: usize,
    joint_cmds: &[Vec<FloatType>],
    generated_compute_link_jacobians: impl Fn(
        &[FloatType; NUM_JOINTS],
        &mut [SMatrix<FloatType, 6, NUM_JOINTS>; NUM_LINKS],
    ),
) {
    // Conversion to fixed-size arrays happens once, up front - not timed.
    let joint_cmds_arr: Vec<[FloatType; NUM_JOINTS]> = joint_cmds
        .iter()
        .map(|c| c.clone().try_into().unwrap())
        .collect();

    group.bench_with_input(
        BenchmarkId::new(bench_id_label, bench_id),
        &joint_cmds_arr,
        |b, cmds| {
            let mut jacobians: [SMatrix<FloatType, 6, NUM_JOINTS>; NUM_LINKS] =
                std::array::from_fn(|_| SMatrix::zeros());
            b.iter(|| {
                for cmd in cmds {
                    generated_compute_link_jacobians(black_box(cmd), &mut jacobians);
                }
                black_box(&jacobians);
            });
        },
    );
}

fn bench_jacobian(c: &mut Criterion) {
    for &urdf_path in BENCH_URDFS {
        let galaw_model = load_urdf::<f64>(urdf_path).unwrap();
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
        let num_links = galaw_model.links.len();
        let num_actuated = galaw_model.num_actuated_joints;
        group.bench_with_input(
            BenchmarkId::new("galaw-runtime", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                let mut jacobians = vec![Matrix6xX::zeros(num_actuated); num_links];
                b.iter(|| {
                    for cmd in cmds {
                        galaw_model
                            .compute_link_jacobians(black_box(cmd), &mut jacobians)
                            .unwrap();
                    }
                    black_box(&jacobians);
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
                        "galaw-generated",
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
