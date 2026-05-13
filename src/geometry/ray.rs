use glam::Vec3;

use crate::{geometry::unit_vec::UnitVec3, traits::Intersects};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HitRecord {
    pub position: Vec3,
    pub length: f32,
    pub surface_normal: UnitVec3,
    pub incident_direction: UnitVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: UnitVec3,
    inv_direction: Vec3,
    pub(crate) extra_length: f32,
}

impl Ray {
    pub fn new(origin: Vec3, direction: UnitVec3) -> Self {
        Self {
            origin,
            direction,
            inv_direction: 1.0 / direction.vec3(),
            extra_length: 0.0,
        }
    }

    pub fn new_wih_length(origin: Vec3, direction: UnitVec3, extra_length: f32) -> Self {
        Self {
            extra_length,
            ..Self::new(origin, direction)
        }
    }

    pub fn inv_direction(&self) -> Vec3 {
        self.inv_direction
    }
}

impl<T: Intersects<Ray>> Intersects<T> for Ray {
    fn intersects(&self, shape: T) -> Option<HitRecord> {
        shape.intersects(*self)
    }
}
