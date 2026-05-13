use glam::Vec3;

use crate::{
    geometry::{
        mesh::Mesh,
        ray::{HitRecord, Ray},
        tri::Tri,
        unit_vec::UnitVec3,
    },
    traits::{Bounded, Intersects},
};

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn from_mesh(mesh: &Mesh) -> Self {
        Self::from_tris(mesh.tris.as_slice())
    }

    pub fn from_tri(tri: &Tri) -> Self {
        Self::from_verts(tri.vertices.as_slice())
    }

    pub fn from_tris<'a>(tris: impl IntoIterator<Item = &'a Tri>) -> Self {
        let verts: Vec<Vec3> = tris
            .into_iter()
            .flat_map(|t| t.vertices)
            .collect();
        Self::from_verts(verts.as_slice())
    }

    pub fn from_verts(verts: &[Vec3]) -> Self {
        if verts.is_empty() {
            return Self {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            };
        }

        let (min, max) = verts
            .iter()
            .fold((verts[0], verts[0]), |(min, max), &x| {
                (min.min(x), max.max(x))
            });

        Aabb { min, max }
    }

    pub fn intersect_distance(&self, shape: &Ray) -> Option<f32> {
        let (min_distance, max_distance) = (0..3_usize)
            .map(|d| {
                let t1 = (self.min[d] - shape.origin[d]) * shape.inv_direction()[d];
                let t2 = (self.max[d] - shape.origin[d]) * shape.inv_direction()[d];

                let t_min = f32::min(t1, t2);
                let t_max = f32::max(t1, t2);

                (t_min, t_max)
            })
            .fold(
                (f32::MIN, f32::MAX),
                |(acc_min, acc_max), (t_min, t_max)| {
                    let min_distance = f32::max(acc_min, t_min);
                    let max_distance = f32::min(acc_max, t_max);

                    (min_distance, max_distance)
                },
            );

        // max_distance < 0 means the entire box is behind the ray.
        // min_distance > max_distance means the slabs don't overlap.
        if max_distance < 0.0 || min_distance > max_distance {
            return None;
        }

        // If min_distance is negative, the ray starts inside the box.
        // Return 0 to indicate "already here" rather than the exit distance.
        Some(min_distance)
    }

    pub fn surface_area(&self) -> f32 {
        let x_area = (self.max.y - self.min.y) * (self.max.z - self.min.z) * 2.0;
        let y_area = (self.max.x - self.min.x) * (self.max.z - self.min.z) * 2.0;
        let z_area = (self.max.x - self.min.x) * (self.max.y - self.min.y) * 2.0;

        x_area + y_area + z_area
    }

    pub fn corners(&self) -> [Vec3; 8] {
        [
            Vec3::new(self.min.x, self.min.y, self.min.z),
            Vec3::new(self.min.x, self.min.y, self.max.z),
            Vec3::new(self.min.x, self.max.y, self.min.z),
            Vec3::new(self.max.x, self.min.y, self.min.z),
            Vec3::new(self.min.x, self.max.y, self.max.z),
            Vec3::new(self.max.x, self.max.y, self.min.z),
            Vec3::new(self.max.x, self.min.y, self.max.z),
            Vec3::new(self.max.x, self.max.y, self.max.z),
        ]
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            min: Vec3::MAX,
            max: Vec3::MIN,
        }
    }
}

impl Intersects<&Ray> for &Aabb {
    fn intersects(&self, shape: &Ray) -> Option<HitRecord> {
        let length = self.intersect_distance(shape)?;
        let position = shape.origin + shape.direction.vec3() * length;

        let center = (self.min + self.max) * 0.5;
        let half_size = (self.max - self.min) * 0.5;

        let surface_normal = UnitVec3::new(nearest_axis((position - center) / half_size));

        Some(HitRecord {
            position,
            length,
            surface_normal,
            incident_direction: shape.direction,
        })
    }
}

impl Bounded for Aabb {
    fn aabb(&self) -> Aabb {
        *self
    }
}

fn nearest_axis(v: Vec3) -> Vec3 {
    let abs = v.abs();
    if abs.x >= abs.y && abs.x >= abs.z {
        Vec3::X * v.x.signum()
    } else if abs.y >= abs.x && abs.y >= abs.z {
        Vec3::Y * v.y.signum()
    } else {
        Vec3::Z * v.z.signum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_intersect_distance() {
        let ray = Ray::new(Vec3::ZERO, UnitVec3::Z);
        let aabb = Aabb::new(Vec3::new(-0.5, -0.5, 1.0), Vec3::new(0.5, 0.5, 2.0));
        let distance = aabb
            .intersect_distance(&ray)
            .expect("No collision found");
        dbg!(distance);
        assert_eq!(distance, 1.0)
    }
}
