/// URDF fixtures used by the FK benchmark suite (`benches/fk_speed.rs`).
/// Add a robot here and the benchmarks pick it up automatically.
pub const BENCH_URDFS: &[&str] = &[
    "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",
    "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",
    "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",
    "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",
];

/// MJCF fixtures used by the FK benchmark suite.
/// Add a robot here and the benchmarks pick it up automatically.
pub const BENCH_MJCFS: &[&str] = &[
    "assets/mjcf/third_party/Boston-Dynamics_Spot/spot.xml",
    "assets/mjcf/third_party/Franka-Emika_Panda/panda.xml",
    "assets/mjcf/third_party/Shadow-Robot_Hand/right_hand.xml",
    "assets/mjcf/third_party/Unitree_G1/g1.xml",
];
