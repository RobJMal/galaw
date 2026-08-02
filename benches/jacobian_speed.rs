/// Bechmarks the Jacobian computations. 

use std::hint::black_box;

// Third-party
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom 
use galaw::{fixtures::BENCH_URDFS, load_urdf};

// ---- CONSTANTS ----
const RNG_SEED: u64 = 42;
const N_POSES: usize = 100;

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
                        for link_idx in 0..galaw_model.links.len() {
                            let out = galaw_model.compute_jacobian(black_box(cmd), link_idx).unwrap();
                            black_box(out);
                        }
                    }
                });
            },
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
