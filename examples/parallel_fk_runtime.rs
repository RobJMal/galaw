//! Demonstrates usage of library with sequential vs. parallelized computing.
//!
//! Run with: `cargo run --release --example parallel_fk_runtime`

// Standard
use std::cell::RefCell;
use std::hint::black_box;
use std::time::Instant;

// Third-party
use nalgebra::Isometry3;
use rayon::prelude::*;

// galaw
use galaw::{load_urdf, types::GalawData};

const URDF: &str = "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf";
const NUM_POSES: usize = 10_000;

fn main() {
    let model = load_urdf::<f64>(URDF).unwrap();
    let n_joints = model.num_actuated_joints;
    let n_links = model.links.len();
    let n_threads = rayon::current_num_threads();

    println!(
        "Robot:   {} ({} DOF, {} links)",
        model.name, n_joints, n_links
    );
    println!("Batch:   {} poses", NUM_POSES);
    println!("Threads: {}", n_threads);
    println!();

    // Batch commands
    let batch_cmds: Vec<Vec<f64>> = (0..NUM_POSES)
        .map(|i| {
            (0..n_joints)
                .map(|j| ((i + j) as f64 * 0.1).sin())
                .collect()
        })
        .collect();

    // ---- Sequential ----
    let seq_start = Instant::now();
    let mut data = model.create_galaw_data();
    for cmds in &batch_cmds {
        model.compute_fk(cmds, &mut data).unwrap();
    }
    black_box(&data.link_poses);
    let seq_time = seq_start.elapsed();

    // ---- Parallel (thread-local buffers) ----
    // One GalawData per OS thread, lazily created and reused for every pose
    // assigned to that thread.
    // Memory: num_threads × sizeof(GalawData)
    thread_local! {
        static TL_DATA: RefCell<Option<GalawData<f64>>> = RefCell::new(None);
    }

    let par_tl_start = Instant::now();
    batch_cmds.par_iter().for_each(|cmds| {
        TL_DATA.with(|cell| {
            let mut borrow = cell.borrow_mut();
            let data = borrow.get_or_insert_with(|| model.create_galaw_data());
            model.compute_fk(cmds, data).unwrap();
            black_box(&data.link_poses);
        });
    });
    let par_tl_time = par_tl_start.elapsed();

    // ---- Results ----
    let galaw_data_size_kb = {
        let poses = n_links * std::mem::size_of::<Isometry3<f64>>();
        let jacs = n_links * 6 * n_joints * std::mem::size_of::<f64>();
        let cmds = n_joints * std::mem::size_of::<f64>();
        (poses + jacs + cmds) / 1024
    };

    println!(
        "Sequential:                  {:>8.2} ms  ({:.2} µs/pose)",
        seq_time.as_secs_f64() * 1e3,
        seq_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
    );
    println!(
        "Parallel thread-local:       {:>8.2} ms  ({:.2} µs/pose) [{} threads × {} KB = {} KB]",
        par_tl_time.as_secs_f64() * 1e3,
        par_tl_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        n_threads,
        galaw_data_size_kb,
        n_threads * galaw_data_size_kb,
    );
}
