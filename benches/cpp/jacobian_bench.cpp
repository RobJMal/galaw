// Native C++ Pinocchio Jacobian benchmark.
// Writes Criterion-compatible estimates.json for plot_bench.rs ("pinocchio-cpp" series).
// Run from repo root: LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH ./benches/cpp/build/jacobian_bench

#include "bench_utils.hpp"

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

int main(int argc, char* argv[]) {
    std::string repo_root = (argc > 1) ? argv[1] : ".";

    for (const auto& robot : bench::bench_robots(repo_root)) {
        std::cout << "Jacobian: " << robot.name << "\n";

        // --- Setup (not timed) ---
        pinocchio::Model model;
        pinocchio::urdf::buildModel(robot.urdf_path, model);
        pinocchio::Data data(model);

        auto configs   = bench::generate_configs(model);
        auto frame_ids = bench::body_frame_ids(model);
        const int n    = static_cast<int>(configs.size());
        int i          = 0;

        // Pre-allocate J once. Reused across iterations to avoid per-call
        // heap allocation (unlike the rs-pinocchio shim which allocates inside
        // frame_jacobian). This represents native C++ achievable performance.
        pinocchio::Data::Matrix6x J(6, model.nv);

        // --- Timed region ---
        // Matches the Rust "pinocchio" arm in jacobian_speed.rs:
        //   pin_model.compute_joint_jacobians(&q)          → computeJointJacobians
        //   pin_model.frame_jacobian(fid, LWA)             → getFrameJacobian (LOCAL_WORLD_ALIGNED)
        //
        // Note: computeJointJacobians internally calls forwardKinematics, which
        // updates data.oMi. getFrameJacobian for LOCAL_WORLD_ALIGNED in Pinocchio 4.x
        // computes data.oMf[fid] on the fly from data.oMi, so no prior
        // updateFramePlacements call is needed (matching the Rust bench exactly).
        auto fn = [&]() {
            pinocchio::computeJointJacobians(model, data, configs[i % n]);
            for (auto fid : frame_ids) {
                J.setZero();
                pinocchio::getFrameJacobian(model, data, fid,
                                            pinocchio::LOCAL_WORLD_ALIGNED, J);
                bench::do_not_optimize(J);
            }
            ++i;
        };

        auto result = bench::run_bench(fn);
        std::cout << "  mean " << static_cast<long>(result.mean) << " ns"
                  << "  [" << static_cast<long>(result.lo)
                  << ", " << static_cast<long>(result.hi) << "]\n";

        bench::write_result(repo_root, "jacobian", robot, "pinocchio-cpp", result);
    }
    return 0;
}
