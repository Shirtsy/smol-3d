use std::ops::Deref;

use glam::Vec3;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum UnitVecError {
    #[error("value must be unit vector")]
    Conversion,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UnitVec3(Vec3);

impl UnitVec3 {
    pub const X: Self = Self(Vec3::X);
    pub const Y: Self = Self(Vec3::Y);
    pub const Z: Self = Self(Vec3::Z);
    pub const NEG_X: Self = Self(Vec3::NEG_X);
    pub const NEG_Y: Self = Self(Vec3::NEG_Y);
    pub const NEG_Z: Self = Self(Vec3::NEG_Z);

    pub fn new(v: Vec3) -> Self {
        Self(v.normalize())
    }

    pub fn vec3(&self) -> Vec3 {
        self.0
    }
}

impl TryFrom<Vec3> for UnitVec3 {
    type Error = UnitVecError;

    fn try_from(value: Vec3) -> Result<Self, Self::Error> {
        match value.is_normalized() {
            true => Ok(Self(value)),
            false => Err(UnitVecError::Conversion),
        }
    }
}

impl From<UnitVec3> for Vec3 {
    fn from(value: UnitVec3) -> Self {
        value.0
    }
}

impl Deref for UnitVec3 {
    type Target = Vec3;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
