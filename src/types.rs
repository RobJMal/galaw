use derive_more::Add;

#[derive(Debug, Add, Clone, Copy)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quaternion {
    pub fn identity() -> Quaternion {
        Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }

    pub fn conj(&self) -> Quaternion {
        /* Returns conjugate of quaternion */
        Quaternion {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    pub fn inverse(&self) -> Quaternion {
        let sq_norm = self.x.powi(2) + self.y.powi(2) + self.z.powi(2) + self.w.powi(2);
        let q_conj = self.conj();

        Quaternion {
            x: q_conj.x / sq_norm,
            y: q_conj.y / sq_norm,
            z: q_conj.z / sq_norm,
            w: q_conj.w / sq_norm,
        }
    }

    pub fn normalize(&self) -> Quaternion {
        let magnitude =
            f64::sqrt(self.x.powi(2) + self.y.powi(2) + self.z.powi(2) + self.w.powi(2));

        Quaternion {
            x: self.x / magnitude,
            y: self.y / magnitude,
            z: self.z / magnitude,
            w: self.w / magnitude,
        }
    }

    pub fn multiply(&self, other: &Quaternion) -> Quaternion {
        Quaternion {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }
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

impl Transform {
    pub fn identity() -> Transform {
        Transform {
            position: Position3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }
}

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
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
            let rpy = EulerRPY {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            };
            let q = rpy.to_quat();
            assert!((q.w - 1.0).abs() < 1e-10);
            assert!(q.x.abs() < 1e-10);
            assert!(q.y.abs() < 1e-10);
            assert!(q.z.abs() < 1e-10);
        }
    }

    mod quaternion_tests {
        use super::Quaternion;

        const TEST_TOLERANCE: f64 = 1e-10;

        #[test]
        fn test_identity_multiply() {
            let quat_01: Quaternion = Quaternion {
                x: 0.5,
                y: 0.5,
                z: 0.5,
                w: 0.5,
            };
            let result: Quaternion = Quaternion::identity().multiply(&quat_01);

            assert!((result.x - quat_01.x).abs() < TEST_TOLERANCE);
            assert!((result.y - quat_01.y).abs() < TEST_TOLERANCE);
            assert!((result.z - quat_01.z).abs() < TEST_TOLERANCE);
            assert!((result.w - quat_01.w).abs() < TEST_TOLERANCE);
        }

        #[test]
        fn test_compose_two_90_deg_rotations() {
            use std::f64::consts::PI;

            let q90z: Quaternion = Quaternion {
                w: (PI / 4.0).cos(),
                x: 0.0,
                y: 0.0,
                z: (PI / 4.0).sin(),
            };
            let q180z: Quaternion = q90z.multiply(&q90z);

            assert!(q180z.x.abs() < TEST_TOLERANCE);
            assert!(q180z.y.abs() < TEST_TOLERANCE);
            assert!((q180z.z - 1.0).abs() < TEST_TOLERANCE);
            assert!(q180z.w.abs() < TEST_TOLERANCE);
        }

        #[test]
        fn test_numerical_stability() {
            // Using small incremental rotation
            let mut q = Quaternion {
                x: 0.001,
                y: 0.0,
                z: 0.0,
                w: 0.9999995,
            };

            for _ in 0..1_000_000 {
                q = q.multiply(&q);
                q = q.normalize();

                let magnitude = q.x.powi(2) + q.y.powi(2) + q.z.powi(2) + q.w.powi(2);
                assert!((magnitude - 1.0).abs() < TEST_TOLERANCE);
            }
        }

        #[test]
        fn test_conjugate() {
            /*
            Given that q* is the conjugate of q,
            Assert that qq* = ||q||^2 (squared norm)
            */
            let q1 = Quaternion {
                x: 0.1,
                y: 0.2,
                z: 0.3,
                w: 0.4,
            };
            let q2 = q1.multiply(&q1.conj());

            let q1_sq_norm = q1.x.powi(2) + q1.y.powi(2) + q1.z.powi(2) + q1.w.powi(2);

            // Propery 01: Output is a real number, imaginary vector parts cancel out
            assert!(
                q2.x.abs() < TEST_TOLERANCE,
                "X component of Quaternion didn't cancel out"
            );
            assert!(
                q2.y.abs() < TEST_TOLERANCE,
                "Y component of Quaternion didn't cancel out"
            );
            assert!(
                q2.z.abs() < TEST_TOLERANCE,
                "Z component of Quaternion didn't cancel out"
            );

            // Property 02: Real part is equal to the squared norm
            assert!(
                (q2.w - q1_sq_norm).abs() < TEST_TOLERANCE,
                "W component of Quaternion doesn't equal squred norm"
            );
        }

        #[test]
        fn test_inverse() {
            let q1 = Quaternion {
                x: 0.4,
                y: -0.3,
                z: 0.2,
                w: 0.1,
            };
            let q1_inv = q1.inverse();

            // Test 01: right inverse (q * q^-1 = 1)
            let right_result = q1.multiply(&q1_inv);
            assert!((right_result.x).abs() < TEST_TOLERANCE);
            assert!((right_result.y).abs() < TEST_TOLERANCE);
            assert!((right_result.z).abs() < TEST_TOLERANCE);
            assert!(
                (right_result.w - 1.0).abs() < TEST_TOLERANCE,
                "Right inverse failed"
            );

            // Test 02: left inverse (q^-1 * q = 1)
            let left_result = q1_inv.multiply(&q1);
            assert!((left_result.x).abs() < TEST_TOLERANCE);
            assert!((left_result.y).abs() < TEST_TOLERANCE);
            assert!((left_result.z).abs() < TEST_TOLERANCE);
            assert!(
                (left_result.w - 1.0).abs() < TEST_TOLERANCE,
                "Left inverse failed"
            );
        }
    }
}
