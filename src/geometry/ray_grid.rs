use glam::Vec3;

use crate::{
    geometry::{
        aabb::Aabb,
        basis::{Basis, ToBasis},
        ray::Ray,
        unit_vec::UnitVec3,
    },
    traits::Bounded,
};

#[derive(Debug)]
pub struct RayGrid {
    pub rays: Vec<Ray>,
}

impl RayGrid {
    pub fn new<T: Bounded>(shape: &T, direction: UnitVec3, ray_spacing: f32) -> Self {
        let bounds = shape.aabb();

        let dir_basis = Basis::from_direction(direction);

        let dir_basis_bounds: [Vec3; 8] = bounds
            .corners()
            .map(|x| x.to_basis(&dir_basis));
        let dir_basis_bounds = Aabb::from_verts(&dir_basis_bounds);

        let z_plane = dir_basis_bounds.min.z - 0.0001;

        let min = Vec3::new(dir_basis_bounds.min.x, dir_basis_bounds.min.y, z_plane);
        let max = Vec3::new(dir_basis_bounds.max.x, dir_basis_bounds.max.y, z_plane);

        let x_count = ((max.x - min.x) / ray_spacing).ceil() as usize;
        let y_count = ((max.y - min.y) / ray_spacing).ceil() as usize;

        let mut rays: Vec<Ray> = vec![];

        for y in 0..=y_count {
            for x in 0..=x_count {
                let x = min.x + x as f32 * ray_spacing;
                let y = min.y + y as f32 * ray_spacing;
                let origin = Vec3::new(x, y, z_plane).to_basis(&Basis::world());
                rays.push(Ray::new(origin, direction))
            }
        }

        Self { rays }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Ray> {
        self.rays.iter()
    }
}

impl<'a> IntoIterator for &'a RayGrid {
    type Item = &'a Ray;

    type IntoIter = std::slice::Iter<'a, Ray>;

    fn into_iter(self) -> Self::IntoIter {
        self.rays.iter()
    }
}
