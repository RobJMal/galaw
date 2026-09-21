//! Demonstrates batched FK with the ahead-of-time generated path: sequential vs. parallel (rayon).
//!
//! Run with: `cargo run --release --example parallel_fk_generated`

// Standard
use std::cell::RefCell;
use std::hint::black_box;
use std::time::Instant;

// Third-party
use rayon::prelude::*;

// galaw
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

    // ---- Parallel (thread-local buffers) ----
    // One GalawData per OS thread, lazily created and reused for every pose
    // assigned to that thread.
    // Memory: num_threads × sizeof(GalawData)
    thread_local! {
        static TL_DATA: RefCell<GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS>> =
            RefCell::new(GeneratedGalawData::new());
    }

    let par_tl_start = Instant::now();
    batch_cmds.par_iter().for_each(|cmds| {
        TL_DATA.with(|cell| {
            let mut data = cell.borrow_mut();
            anymal_d::compute_fk(cmds, &mut data);
            black_box(&data.link_poses);
        });
    });
    let par_tl_time = par_tl_start.elapsed();

    // ---- Results ----
    let galaw_data_size_kb =
        std::mem::size_of::<GeneratedGalawData<f64, NUM_JOINTS, NUM_LINKS>>() / 1024;

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
