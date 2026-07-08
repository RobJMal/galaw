#[derive(Debug)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug)]
pub struct EulerRPY {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl EulerRPY {
    pub fn to_quat(&self) -> Quaternion {
        let cos_roll = (self.roll / 2.0).cos();
        let sin_roll = (self.roll / 2.0).sin();

        let cos_pitch = (self.pitch / 2.0).cos();
        let sin_pitch = (self.pitch / 2.0).sin();

        let cos_yaw = (self.yaw / 2.0).cos();
        let sin_yaw = (self.yaw / 2.0).sin();

        Quaternion { 
            x: sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw, 
            y: cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw, 
            z: cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw, 
            w: cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
        }
    }
}

#[derive(Debug)]
pub struct Transform {
    pub position: Position3D,
    pub orientation: Quaternion,
}

#[derive(Debug)]
pub struct Link {
    pub name: String,
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub transform: Transform,
    pub axis: Position3D,
    pub limit_lower: f64,
    pub limit_upper: f64,
}

#[derive(Debug)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}


#[cfg(test)]
mod tests {
    use super::*;

    mod euler_rpy_to_quaternion_tests {
        use super::EulerRPY;

        #[test]
        fn test_zero_rpy_gives_identity_quaternion() {
            let rpy = EulerRPY { roll: 0.0, pitch: 0.0, yaw: 0.0 };
            let q = rpy.to_quat();
            assert!((q.w - 1.0).abs() < 1e-10);
            assert!(q.x.abs() < 1e-10);
            assert!(q.y.abs() < 1e-10);
            assert!(q.z.abs() < 1e-10);
        }

    }
}