use glam::Vec3;

use crate::geometry::unit_vec::UnitVec3;

#[derive(Debug, Clone)]
pub struct Basis {
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Basis {
    pub fn new(x: Vec3, y: Vec3, z: Vec3) -> Self {
        Self { x, y, z }
    }

    pub fn world() -> Self {
        Self::default()
    }

    pub fn from_direction(direction: UnitVec3) -> Self {
        let mut x = Vec3::ZERO;
        let z: Vec3 = direction.into();

        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            if z.dot(axis).abs() < 0.99 {
                x = z.cross(axis).normalize();
                break;
            }
        }

        let y = z.cross(x).normalize();
        Self::new(x, y, z)
    }

    pub fn x(&self) -> Vec3 {
        self.x
    }

    pub fn y(&self) -> Vec3 {
        self.y
    }

    pub fn z(&self) -> Vec3 {
        self.z
    }

    pub fn change(&self, vec: Vec3) -> Vec3 {
        Vec3::new(vec.dot(self.x), vec.dot(self.y), vec.dot(self.z))
    }
}

impl Default for Basis {
    fn default() -> Self {
        Self {
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        }
    }
}

pub trait ToBasis {
    fn to_basis(&self, basis: &Basis) -> Self;
}

impl ToBasis for Vec3 {
    fn to_basis(&self, basis: &Basis) -> Self {
        basis.change(*self)
    }
}
