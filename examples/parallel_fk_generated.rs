//! Demonstrates batched FK with the ahead-of-time generated path: sequential vs. parallel (rayon).
//!
//! Run with: `cargo run --release --example galaw_parallel_generated`

use std::cell::RefCell;
use std::hint::black_box;
use std::time::Instant;

use nalgebra::Isometry3;
use rayon::prelude::*;

use galaw::{generated::anymal_d, types::GeneratedGalawData};

// anymal_d constants — baked into the generated code at codegen time.
const NUM_JOINTS: usize = 14;
const NUM_LINKS: usize = 96;
const NUM_POSES: usize = 10_000;

fn main() {
    let n_threads = rayon::current_num_threads();

    println!("Robot:   ANYmal-D ({NUM_JOINTS} DOF, {NUM_LINKS} links) [generated path]");
    println!("Batch:   {NUM_POSES} poses");
    println!("Threads: {n_threads}");
    println!();

    // Fixed-size array commands — no Vec, no heap allocation per pose.
    let batch_cmds: Vec<[f64; NUM_JOINTS]> = (0..NUM_POSES)
        .map(|i| std::array::from_fn(|j| ((i + j) as f64 * 0.1).sin()))
        .collect();

    // ---- Sequential ----
    let seq_start = Instant::now();
    let mut data: GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS> = GeneratedGalawData::new();
    for cmds in &batch_cmds {
        anymal_d::compute_fk(cmds, &mut data);
    }
    black_box(&data.link_poses);
    let seq_time = seq_start.elapsed();

    // ---- Parallel: thread-local buffers ----
    // No Option needed — GeneratedGalawData::new() is cheap and the type is always
    // valid, so we initialize directly rather than lazily.
    // Memory: num_threads × sizeof(GeneratedGalawData)
    thread_local! {
        static TL_DATA: RefCell<GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS>> =
            RefCell::new(GeneratedGalawData::new());
    }

    let par_tl_start = Instant::now();
    batch_cmds.par_iter().for_each(|cmds| {
        TL_DATA.with(|cell| {
            let mut data = cell.borrow_mut();
            anymal_d::compute_fk(cmds, &mut *data);
            black_box(&data.link_poses);
        });
    });
    let par_tl_time = par_tl_start.elapsed();

    // ---- Parallel: thread-local + saved results ----
    // Scratch space stays thread-local; only the poses are written out per pose.
    // Memory: (num_threads × sizeof(GeneratedGalawData)) + (NUM_POSES × NUM_LINKS × sizeof(Isometry3))
    let mut saved_poses: Vec<[Isometry3<f64>; NUM_LINKS]> =
        vec![[Isometry3::identity(); NUM_LINKS]; NUM_POSES];

    let par_tl_save_start = Instant::now();
    batch_cmds
        .par_iter()
        .zip(saved_poses.par_iter_mut())
        .for_each(|(cmds, out)| {
            TL_DATA.with(|cell| {
                let mut data = cell.borrow_mut();
                anymal_d::compute_fk(cmds, &mut *data);
                *out = data.link_poses;
            });
        });
    let par_tl_save_time = par_tl_save_start.elapsed();
    black_box(&saved_poses);

    // ---- Parallel: per-pose buffers ----
    // One GeneratedGalawData pre-allocated per pose. Results stay accessible after
    // the loop, but memory cost is NUM_POSES × sizeof(GeneratedGalawData).
    let mut batch_data: Vec<GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS>> =
        (0..NUM_POSES).map(|_| GeneratedGalawData::new()).collect();

    let par_pp_start = Instant::now();
    batch_cmds
        .par_iter()
        .zip(batch_data.par_iter_mut())
        .for_each(|(cmds, data)| {
            anymal_d::compute_fk(cmds, data);
            black_box(&data.link_poses);
        });
    let par_pp_time = par_pp_start.elapsed();

    // ---- Results ----
    let data_size_kb =
        std::mem::size_of::<GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS>>() / 1024;
    let poses_only_kb = NUM_LINKS * std::mem::size_of::<Isometry3<f64>>() / 1024;

    println!(
        "Sequential:                  {:>8.2} ms  ({:.2} µs/pose)  results: discarded",
        seq_time.as_secs_f64() * 1e3,
        seq_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
    );
    println!(
        "Parallel thread-local:       {:>8.2} ms  ({:.2} µs/pose)  results: discarded     [{}t × {} KB = {} KB]",
        par_tl_time.as_secs_f64() * 1e3,
        par_tl_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        n_threads, data_size_kb, n_threads * data_size_kb,
    );
    println!(
        "Parallel thread-local+saved: {:>8.2} ms  ({:.2} µs/pose)  results: poses only   [{}t × {} KB scratch + {} poses × {} KB = {} MB]",
        par_tl_save_time.as_secs_f64() * 1e3,
        par_tl_save_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        n_threads, data_size_kb,
        NUM_POSES, poses_only_kb, (n_threads * data_size_kb + NUM_POSES * poses_only_kb) / 1024,
    );
    println!(
        "Parallel per-pose:           {:>8.2} ms  ({:.2} µs/pose)  results: full GeneratedGalawData [{} poses × {} KB = {} MB]",
        par_pp_time.as_secs_f64() * 1e3,
        par_pp_time.as_secs_f64() * 1e6 / NUM_POSES as f64,
        NUM_POSES, data_size_kb, NUM_POSES * data_size_kb / 1024,
    );
    println!();
    println!(
        "Speedup vs sequential — thread-local: {:.2}x  thread-local+saved: {:.2}x  per-pose: {:.2}x  (theoretical max: {}x)",
        seq_time.as_secs_f64() / par_tl_time.as_secs_f64(),
        seq_time.as_secs_f64() / par_tl_save_time.as_secs_f64(),
        seq_time.as_secs_f64() / par_pp_time.as_secs_f64(),
        n_threads,
    );
}
