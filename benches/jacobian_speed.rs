/// Bechmarks the Jacobian computations.
use std::hint::black_box;

// Third-party
use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, BenchmarkId, Criterion, criterion_group, criterion_main};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{fixtures::BENCH_URDFS, load_urdf, types::GeneratedGalawData};

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
        &mut GeneratedGalawData<FloatType, NUM_JOINTS, NUM_LINKS>,
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
            let mut data = GeneratedGalawData::new();
            let mut i = 0usize;
            b.iter(|| {
                generated_compute_link_jacobians(black_box(&cmds[i % cmds.len()]), &mut data);
                i += 1;
                black_box(&data.link_jacobians);
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
        group.throughput(criterion::Throughput::Elements(1));

        // ---- galaw-runtime ----
        group.bench_with_input(
            BenchmarkId::new("galaw-runtime", galaw_model.joints.len()),
            &joint_cmds,
            |b, cmds| {
                let mut data = galaw_model.create_galaw_data();
                let mut i = 0usize;
                b.iter(|| {
                    galaw_model
                        .compute_link_jacobians(black_box(&cmds[i % cmds.len()]), &mut data)
                        .unwrap();
                    i += 1;
                    black_box(&data.link_jacobians);
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
                let mut i = 0usize;
                b.iter(|| {
                    k_chain.set_joint_positions(black_box(&cmds[i % cmds.len()])).unwrap();
                    i += 1;
                    k_chain.update_transforms();
                    for link in &galaw_model.links {
                        let node = k_chain.find_link(&link.name).unwrap();
                        let serial = k::SerialChain::from_end(node);
                        black_box(k::jacobian(&serial));
                    }
                });
            },
        );

        group.finish();
    }
}

criterion_group!(benches, bench_jacobian);
criterion_main!(benches);
