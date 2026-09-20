//! Demonstrates usage of library with sequential vs. parallelized computing.
//!
//! Run with: `cargo run --release --example galaw_parallel`

use std::cell::RefCell;
use std::hint::black_box;
use std::time::Instant;

use rayon::prelude::*;

use galaw::{load_urdf, types::GalawData};

const URDF: &str = "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf";
const NUM_POSES: usize = 10_000;

fn main() {
    let model = load_urdf::<f64>(URDF).unwrap();
    let n_joints = model.num_actuated_joints;
    let n_links = model.links.len();
    let n_threads = rayon::current_num_threads();

    println!("Robot:   {} ({} DOF, {} links)", model.name, n_joints, n_links);
    println!("Batch:   {} poses", NUM_POSES);
    println!("Threads: {}", n_threads);
    println!();

    // Batch commands
    let batch_cmds: Vec<Vec<f64>> = (0..NUM_POSES)
        .map(|i| (0..n_joints).map(|j| ((i + j) as f64 * 0.1).sin()).collect())
        .collect();

    // ---- Sequential ----
    let seq_start = Instant::now();
    let mut data = model.create_galaw_data();
    for cmds in &batch_cmds {
        model.compute_fk(cmds, &mut data).unwrap();
    }
    black_box(&data.link_poses);
    let seq_time = seq_start.elapsed();

    // ---- Parallel: thread-local buffers ----
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

    // ---- Parallel: per-pose buffers ----
    // One GalawData pre-allocated per pose. Results remain accessible after
    // the loop, but memory cost is NUM_POSES × sizeof(GalawData).
    let mut batch_data: Vec<GalawData<f64>> =
        (0..NUM_POSES).map(|_| model.create_galaw_data()).collect();

    let par_pp_start = Instant::now();
    batch_cmds
        .par_iter()
        .zip(batch_data.par_iter_mut())
        .for_each(|(cmds, data)| {
            model.compute_fk(cmds, data).unwrap();
            black_box(&data.link_poses);
        });
    let par_pp_time = par_pp_start.elapsed();

    // ---- Results ----
    let data_size_kb = {
        let poses = n_links * std::mem::size_of::<nalgebra::Isometry3<f64>>();
        let jacs = n_links * 6 * n_joints * std::mem::size_of::<f64>();
        let cmds = n_joints * std::mem::size_of::<f64>();
        (poses + jacs + cmds) / 1024
    };

    println!(
        "Sequential:            {:>8.2} ms  ({:.2} µs/pose)",
        seq_time.as_secs_f64() * 1e3,
        seq_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
    );
    println!(
        "Parallel thread-local: {:>8.2} ms  ({:.2} µs/pose)  [{}t × {} KB = {} KB total]",
        par_tl_time.as_secs_f64() * 1e3,
        par_tl_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        n_threads,
        data_size_kb,
        n_threads * data_size_kb,
    );
    println!(
        "Parallel per-pose:     {:>8.2} ms  ({:.2} µs/pose)  [{} poses × {} KB = {} MB total]",
        par_pp_time.as_secs_f64() * 1e3,
        par_pp_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        NUM_POSES,
        data_size_kb,
        NUM_POSES * data_size_kb / 1024,
    );
    println!();
    println!(
        "Speedup (thread-local): {:.2}x  |  Speedup (per-pose): {:.2}x  (theoretical max: {}x)",
        seq_time.as_secs_f64() / par_tl_time.as_secs_f64(),
        seq_time.as_secs_f64() / par_pp_time.as_secs_f64(),
        n_threads,
    );
}
