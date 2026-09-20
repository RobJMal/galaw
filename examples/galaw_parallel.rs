//! Demonstrates usage of libray with sequential vs. parallized computing.

use std::cell::RefCell;
use std::hint::black_box;
use std::time::Instant;

use rayon::prelude::*;

use galaw::{load_urdf, types::GalawData};

// const URDF: &str = "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf";
const URDF: &str = "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf";
const NUM_POSES: usize = 10_000;

fn main() {
    let model = load_urdf::<f64>(URDF).unwrap();
    let n_joints = model.num_actuated_joints;
    let n_threads = rayon::current_num_threads();

    println!("Robot:   {} ({} DOF, {} links)", model.name, n_joints, model.links.len());
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

    // ---- Parallel (rayon, all available cores) ----
    // One GalawData buffer per OS thread via thread_local!. Created on first
    // use and reused for every pose assigned to that thread. Memory cost is 
    // (num_threads x sizeof(GalawData)) instead of (N_POSES x sizeof(GalawData)).
    thread_local! {
        static TL_DATA: RefCell<Option<GalawData<f64>>> = RefCell::new(None);
    }

    let par_start = Instant::now();
    batch_cmds.par_iter().for_each(|cmds| {
        TL_DATA.with(|cell| {
            let mut borrow = cell.borrow_mut();
            let data = borrow.get_or_insert_with(|| model.create_galaw_data());
            model.compute_fk(cmds, data).unwrap();
            black_box(&data.link_poses);
        });
    });
    let par_time = par_start.elapsed();

    let speedup = seq_time.as_secs_f64() / par_time.as_secs_f64();

    println!(
        "Sequential: {:>8.2} ms  ({:.2} µs/pose)",
        seq_time.as_secs_f64() * 1e3,
        seq_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
    );
    println!(
        "Parallel:   {:>8.2} ms  ({:.2} µs/pose)",
        par_time.as_secs_f64() * 1e3,
        par_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
    );
    println!(
        "Speedup:    {:.2}x  (theoretical max: {}x)",
        speedup, n_threads,
    );
}
