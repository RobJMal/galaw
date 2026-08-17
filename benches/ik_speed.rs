use std::collections::HashSet;
use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use k::InverseKinematicsSolver;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

use galaw::{fixtures::BENCH_URDFS, load_urdf, types::GalawModel};

const RNG_SEED: u64 = 42;
const N_POSES: usize = 100;
const MAX_PERTURBATION: f64 = 0.5;

fn random_joint_cmds(model: &GalawModel, rng: &mut ChaCha8Rng) -> Vec<f64> {
    model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .map(|j| match (j.limit_lower, j.limit_upper) {
            (Some(lo), Some(hi)) => rng.random_range(lo..hi),
            _ => rng.random_range(0.0..0.0),
        })
        .collect()
}

fn perturbed_joint_cmds(model: &GalawModel, base: &[f64], rng: &mut ChaCha8Rng) -> Vec<f64> {
    model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .zip(base)
        .map(|(j, &v)| {
            let p = v + rng.random_range(-MAX_PERTURBATION..MAX_PERTURBATION);
            match (j.limit_lower, j.limit_upper) {
                (Some(lo), Some(hi)) => p.clamp(lo, hi),
                _ => p,
            }
        })
        .collect()
}

// First leaf link with an actuated ancestor.
fn target_link(model: &GalawModel) -> usize {
    let parents: HashSet<usize> = model.joints.iter().map(|j| j.parent_link_idx).collect();
    let mut has_actuated_ancestor = vec![false; model.links.len()];
    for joint in &model.joints {
        has_actuated_ancestor[joint.child_link_idx] =
            has_actuated_ancestor[joint.parent_link_idx] || joint.cmd_idx.is_some();
    }
    (0..model.links.len())
        .find(|&i| !parents.contains(&i) && has_actuated_ancestor[i])
        .unwrap()
}

fn bench_ik(c: &mut Criterion) {
    for &urdf_path in BENCH_URDFS {
        let galaw_model = load_urdf(urdf_path).unwrap();
        let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
        let link_idx = target_link(&galaw_model);
        let link_name = &galaw_model.links[link_idx].name;

        let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
        let trials: Vec<(Vec<f64>, Vec<f64>)> = (0..N_POSES)
            .map(|_| {
                let target = random_joint_cmds(&galaw_model, &mut rng);
                let init = perturbed_joint_cmds(&galaw_model, &target, &mut rng);
                (target, init)
            })
            .collect();

        let mut group = c.benchmark_group(format!("ik/{}", galaw_model.name));
        group.throughput(criterion::Throughput::Elements(trials.len() as u64));

        group.bench_with_input(
            BenchmarkId::new("galaw-runtime", galaw_model.joints.len()),
            &trials,
            |b, trials| {
                b.iter(|| {
                    for (target, init) in trials {
                        let pose = galaw_model.compute_fk(target).unwrap()[link_idx];
                        let _ = black_box(galaw_model.compute_ik(link_idx, &pose, black_box(init)));
                    }
                });
            },
        );

        let solver = k::JacobianIkSolver::new(1e-4, 1e-4, 1.0, 1000);
        group.bench_with_input(
            BenchmarkId::new("k", galaw_model.joints.len()),
            &trials,
            |b, trials| {
                b.iter(|| {
                    for (target, init) in trials {
                        k_chain.set_joint_positions(target).unwrap();
                        k_chain.update_transforms();
                        let pose = k_chain
                            .find_link(link_name)
                            .unwrap()
                            .world_transform()
                            .unwrap();

                        k_chain.set_joint_positions(black_box(init)).unwrap();
                        k_chain.update_transforms();
                        let serial =
                            k::SerialChain::from_end(k_chain.find_link(link_name).unwrap());
                        let _ = black_box(solver.solve(&serial, &pose));
                    }
                });
            },
        );

        group.finish();
    }
}

criterion_group!(benches, bench_ik);
criterion_main!(benches);
