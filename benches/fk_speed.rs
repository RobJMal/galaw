use std::hint::black_box;   // Prevents compiler from optimizing away code since we're benchmarking ("be pessimistic")
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use galaw::load_urdf;


const RNG_SEED: u64 = 42;
const N_POSES: usize = 100;    // Random poses per robot

// Robot embodiments to test
const URDFS: &[&str] = &[
    "assets/simple_robot.urdf",
];


fn bench_fk(c: &mut Criterion) {
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
            }
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
