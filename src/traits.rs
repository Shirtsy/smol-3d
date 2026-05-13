use glam::Vec3;

use crate::geometry::{aabb::Aabb, ray::HitRecord, tri::Tri};

pub trait Intersects<T> {
    fn intersects(&self, shape: T) -> Option<HitRecord>;
}

pub trait Bounded {
    fn aabb(&self) -> Aabb;
}

impl Bounded for &[Vec3] {
    fn aabb(&self) -> Aabb {
        Aabb::from_verts(self)
    }
}

impl Bounded for &[Tri] {
    fn aabb(&self) -> Aabb {
        Aabb::from_tris(self.iter())
    }
}
