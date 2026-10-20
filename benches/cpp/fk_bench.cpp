// Native C++ Pinocchio FK benchmark.
// Writes Criterion-compatible estimates.json for plot_bench.rs ("pinocchio-cpp" series).
// Run from repo root: LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH ./benches/cpp/build/fk_bench

#include "bench_utils.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

int main(int argc, char* argv[]) {
    // Repo root defaults to "." (run from repo root). Pass a path as the first
    // argument to override, e.g. when running from a subdirectory.
    std::string repo_root = (argc > 1) ? argv[1] : ".";

    for (const auto& robot : bench::bench_robots(repo_root)) {
        std::cout << "FK: " << robot.name << "\n";

        // --- Setup (not timed) ---
        pinocchio::Model model;
        pinocchio::urdf::buildModel(robot.urdf_path, model);
        pinocchio::Data data(model);

        auto configs   = bench::generate_configs(model);
        auto frame_ids = bench::body_frame_ids(model);
        const int n    = static_cast<int>(configs.size());
        int i          = 0;

        // --- Timed region ---
        auto fn = [&]() {
            pinocchio::forwardKinematics(model, data, configs[i % n]);
            pinocchio::updateFramePlacements(model, data);
            for (auto fid : frame_ids) {
                bench::do_not_optimize(data.oMf[fid]);
            }
            ++i;
        };

        auto result = bench::run_bench(fn);
        std::cout << "  mean " << static_cast<long>(result.mean) << " ns"
                  << "  [" << static_cast<long>(result.lo)
                  << ", " << static_cast<long>(result.hi) << "]\n";

        bench::write_result(repo_root, "fk", robot, "pinocchio-cpp", result);
    }
    return 0;
}
