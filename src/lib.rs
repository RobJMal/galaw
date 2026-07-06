pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

pub struct EulerRPY {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

pub struct Transform {
    pub position: Position3D,
    pub orientation: Quaternion,
}

pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub transform: Transform,
    pub axis: Position3D,
    pub limit_lower: f64,
    pub limit_upper: f64,
}
