use std::ops::Index;

use glam::Vec3;

use crate::{
    geometry::{
        aabb::Aabb,
        ray::{HitRecord, Ray},
        stl::{self, STL_TRI_SIZE},
        unit_vec::UnitVec3,
    },
    traits::{Bounded, Intersects},
};

#[derive(Debug, Clone, Copy)]
pub enum WindingOrder {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Clone, Copy)]
pub struct Tri {
    pub vertices: [Vec3; 3],
    pub normal: UnitVec3,
    pub winding: WindingOrder,
}

impl Tri {
    pub fn recalculate_normal(&self) -> Self {
        let normal = match self.winding {
            WindingOrder::Clockwise => (self[2] - self[0]).cross(self[1] - self[0]),
            WindingOrder::CounterClockwise => (self[1] - self[0]).cross(self[2] - self[0]),
        };
        let normal = UnitVec3::new(normal);

        Self { normal, ..*self }
    }

    pub fn from_stl_bytes(bytes: &[u8; STL_TRI_SIZE]) -> Self {
        let normal = stl::vector_from_bytes(
            bytes[0..12]
                .try_into()
                .unwrap(),
        );
        let normal = UnitVec3::new(normal);

        let vec_a = stl::vector_from_bytes(
            bytes[12..24]
                .try_into()
                .unwrap(),
        );
        let vec_b = stl::vector_from_bytes(
            bytes[24..36]
                .try_into()
                .unwrap(),
        );
        let vec_c = stl::vector_from_bytes(
            bytes[36..48]
                .try_into()
                .unwrap(),
        );

        let _attribute = u16::from_le_bytes(
            bytes[48..50]
                .try_into()
                .unwrap(),
        );

        Self {
            vertices: [vec_a, vec_b, vec_c],
            normal,
            winding: WindingOrder::CounterClockwise,
        }
    }

    pub fn centroid(&self) -> Vec3 {
        Vec3 {
            x: (self[0].x + self[1].x + self[2].x) / 3.0,
            y: (self[0].y + self[1].y + self[2].y) / 3.0,
            z: (self[0].z + self[1].z + self[2].z) / 3.0,
        }
    }

    pub fn area(&self) -> f32 {
        let ab = self[1] - self[0];
        let ac = self[2] - self[0];
        ab.cross(ac).length() / 2.0
    }

    pub fn iter(&self) -> impl Iterator {
        self.vertices.iter()
    }
}

impl Index<usize> for Tri {
    type Output = Vec3;

    fn index(&self, index: usize) -> &Self::Output {
        &self.vertices[index]
    }
}

impl Bounded for Tri {
    fn aabb(&self) -> Aabb {
        Aabb::from_tri(self)
    }
}

impl Intersects<Ray> for &Tri {
    fn intersects(&self, shape: Ray) -> Option<HitRecord> {
        moller_trumbore_intersection(&shape, self)
    }
}

impl Intersects<Ray> for &[Tri] {
    fn intersects(&self, shape: Ray) -> Option<HitRecord> {
        self.iter()
            .filter_map(|t| shape.intersects(t))
            .min_by(|a, b| a.length.total_cmp(&b.length))
    }
}

fn moller_trumbore_intersection(ray: &Ray, triangle: &Tri) -> Option<HitRecord> {
    let origin = ray.origin;
    let direction = ray.direction;

    let e1 = triangle[1] - triangle[0];
    let e2 = triangle[2] - triangle[0];

    let ray_cross_e2 = direction.cross(e2);
    let det = e1.dot(ray_cross_e2);

    if det > -f32::EPSILON && det < f32::EPSILON {
        return None; // This ray is parallel to this triangle.
    }

    let inv_det = 1.0 / det;
    let s = origin - triangle[0];
    let u = inv_det * s.dot(ray_cross_e2);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let s_cross_e1 = s.cross(e1);
    let v = inv_det * direction.dot(s_cross_e1);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    // At this stage we can compute t to find out where the intersection point is on the line.
    let t = inv_det * e2.dot(s_cross_e1);

    if t > f32::EPSILON {
        // ray intersection
        let intersection_point = origin + direction.vec3() * t;
        Some(HitRecord {
            position: intersection_point,
            length: t + ray.extra_length,
            surface_normal: triangle.normal,
            incident_direction: ray.direction,
        })
    } else {
        // This means that there is a line intersection but not a ray intersection.
        None
    }
}
