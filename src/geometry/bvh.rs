use core::f32;
use std::{cmp::Ordering, ops::Index};

use slab::Slab;

use crate::{
    geometry::{
        aabb::Aabb,
        ray::{HitRecord, Ray},
        tri::Tri,
    },
    traits::{Bounded, Intersects},
};

type ArenaID = usize;

#[derive(Debug, Clone)]
enum BvhNodeKind {
    Leaf(Vec<Tri>),
    Branch { left: ArenaID, right: ArenaID },
}

#[derive(Debug, Clone)]
struct BvhNode {
    bounds: Aabb,
    kind: BvhNodeKind,
}

impl BvhNode {
    fn branch(&mut self, left: ArenaID, right: ArenaID) {
        self.kind = BvhNodeKind::Branch { left, right }
    }
}

#[derive(Debug)]
pub struct Bvh {
    arena: Slab<BvhNode>,
    root: ArenaID,
}

impl Bvh {
    const TARGET_TRIS_PER_LEAF: usize = 4;
    const SPLIT_BUCKET_SIZE: f32 = 0.01;

    pub fn from_tris(tris: &[Tri]) -> Self {
        let tris: Vec<Tri> = tris.to_vec();
        let mut arena = Slab::new();

        let root_node: BvhNode = BvhNode {
            bounds: Aabb::from_tris(&tris),
            kind: BvhNodeKind::Leaf(tris),
        };

        let root = arena.insert(root_node);
        let mut bvh = Self { arena, root };
        bvh.build();
        bvh
    }

    fn build(&mut self) {
        let mut work_list: Vec<ArenaID> = vec![self.root];

        while let Some(id) = work_list.pop() {
            if let Some((left, right)) = self.try_split_node(id) {
                work_list.push(left);
                work_list.push(right);
            }
        }
    }

    fn try_split_node(&mut self, id: ArenaID) -> Option<(ArenaID, ArenaID)> {
        let parent_node = self.get_node(id);
        let parent_sah = self.sah_node(parent_node);

        let BvhNodeKind::Leaf(tris) = &parent_node.kind else {
            return None;
        };

        if tris.len() <= Self::TARGET_TRIS_PER_LEAF {
            return None;
        }

        let splits = DimSplitPositions::from_tris(tris, Self::SPLIT_BUCKET_SIZE);

        #[derive(Default)]
        struct MinSplit {
            split: f32,
            dim: usize,
            sah: f32,
        }

        let mut min_sah_split = MinSplit {
            sah: f32::MAX,
            ..MinSplit::default()
        };

        for (dim, positions) in splits.iter().enumerate() {
            for &split in positions {
                let (left, right): (Vec<Tri>, Vec<Tri>) = tris
                    .iter()
                    .partition(|&t| t.centroid()[dim] < split);

                let left = left.as_slice();
                let right = right.as_slice();

                let aabb_left = Aabb::from_tris(left);
                let aabb_right = Aabb::from_tris(right);

                let sah_left = surface_area_heuristic(left, aabb_left);
                let sah_right = surface_area_heuristic(right, aabb_right);

                let sah = sah_left + sah_right;

                if sah < min_sah_split.sah {
                    min_sah_split = MinSplit { split, dim, sah }
                }
            }
        }

        if parent_sah < min_sah_split.sah {
            return None;
        }

        let (tris_left, tris_right): (Vec<Tri>, Vec<Tri>) = tris
            .iter()
            .partition(|&t| t.centroid()[min_sah_split.dim] < min_sah_split.split);

        let aabb_left = Aabb::from_tris(tris_left.as_slice());
        let aabb_right = Aabb::from_tris(tris_right.as_slice());

        let node_left = BvhNode {
            bounds: aabb_left,
            kind: BvhNodeKind::Leaf(tris_left),
        };
        let node_right = BvhNode {
            bounds: aabb_right,
            kind: BvhNodeKind::Leaf(tris_right),
        };

        let id_left = self.arena.insert(node_left);
        let id_right = self.arena.insert(node_right);

        let parent_node = self.get_node_mut(id);
        parent_node.branch(id_left, id_right);

        Some((id_left, id_right))
    }

    fn sah_node(&self, node: &BvhNode) -> f32 {
        match &node.kind {
            BvhNodeKind::Branch { left, right } => {
                let left = self.get_node(*left);
                let right = self.get_node(*right);

                self.sah_node(left) + self.sah_node(right)
            }

            BvhNodeKind::Leaf(tris) => surface_area_heuristic(tris, node.bounds),
        }
    }

    fn get_node(&self, id: ArenaID) -> &BvhNode {
        self.arena
            .get(id)
            .expect("Cannot get node that does not exist")
    }

    fn get_node_mut(&mut self, id: ArenaID) -> &mut BvhNode {
        self.arena
            .get_mut(id)
            .expect("Cannot get node that does not exist")
    }
}

impl Intersects<Ray> for &Bvh {
    fn intersects(&self, shape: Ray) -> Option<HitRecord> {
        let mut work_list: Vec<ArenaID> = vec![self.root];
        let mut min_collision_distance = f32::INFINITY;
        let mut closest_hit: Option<HitRecord> = None;

        while let Some(id) = work_list.pop() {
            let node = self.get_node(id);
            match &node.kind {
                BvhNodeKind::Leaf(tris) => {
                    if let Some(hit) = shape.intersects(tris.as_slice())
                        && hit.length < min_collision_distance
                    {
                        closest_hit = Some(hit);
                        min_collision_distance = hit.length;
                    }
                }

                BvhNodeKind::Branch { left, right } => {
                    let left_distance = self
                        .get_node(*left)
                        .bounds
                        .intersect_distance(&shape);
                    let right_distance = self
                        .get_node(*right)
                        .bounds
                        .intersect_distance(&shape);
                    let distances = (left_distance, right_distance);

                    match distances {
                        (None, None) => {}
                        (None, Some(r)) => {
                            if r < min_collision_distance {
                                work_list.push(*right);
                            }
                        }
                        (Some(l), None) => {
                            if l < min_collision_distance {
                                work_list.push(*left);
                            }
                        }
                        (Some(l), Some(r)) => {
                            if l > r {
                                if l < min_collision_distance {
                                    work_list.push(*left);
                                }
                                if r < min_collision_distance {
                                    work_list.push(*right);
                                }
                            } else {
                                if r < min_collision_distance {
                                    work_list.push(*right);
                                }
                                if l < min_collision_distance {
                                    work_list.push(*left);
                                }
                            }
                        }
                    }
                }
            }
        }
        closest_hit
    }
}

impl Bounded for Bvh {
    fn aabb(&self) -> Aabb {
        self.get_node(self.root)
            .bounds
    }
}

#[derive(Debug)]
struct DimSplitPositions([Vec<f32>; 3]);

impl DimSplitPositions {
    fn from_tris(tris: &[Tri], bucket_size: f32) -> Self {
        let centroids = tris
            .iter()
            .map(|t| t.centroid());

        let mut x_splits = vec![];
        let mut y_splits = vec![];
        let mut z_splits = vec![];

        for centroid in centroids {
            x_splits.push(centroid.x);
            y_splits.push(centroid.y);
            z_splits.push(centroid.z);
        }

        let mut xyz_splits = Self([x_splits, y_splits, z_splits]);
        xyz_splits.apply_bucketing(bucket_size);
        xyz_splits
    }

    fn apply_bucketing(&mut self, bucket_size: f32) {
        for dim_splits in &mut self.0 {
            let mut splits = dim_splits.clone();

            if splits.is_empty() {
                continue;
            }

            splits.sort_by(|a, b| a.total_cmp(b));

            let mut max = *splits.first().unwrap() + bucket_size;
            let mut new_splits: Vec<f32> = vec![];
            let mut bucket: Vec<f32> = vec![];

            for x in splits {
                match x.total_cmp(&max) {
                    Ordering::Less | Ordering::Equal => bucket.push(x),

                    Ordering::Greater => {
                        new_splits.push(bucket.iter().sum::<f32>() / bucket.len() as f32);
                        bucket.clear();
                        max = x + bucket_size;
                        bucket.push(x);
                    }
                }
            }

            new_splits.push(bucket.iter().sum::<f32>() / bucket.len() as f32);
            *dim_splits = new_splits;
        }
    }

    fn iter(&self) -> impl Iterator<Item = &Vec<f32>> {
        self.0.iter()
    }
}

impl Index<usize> for DimSplitPositions {
    type Output = Vec<f32>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

fn surface_area_heuristic(tris: &[Tri], bounds: Aabb) -> f32 {
    bounds.surface_area() * tris.len() as f32
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use glam::Vec3;

    use crate::geometry::{mesh::Mesh, unit_vec::UnitVec3};

    use super::*;

    #[test]
    fn build_bvh() {
        let mesh = load_test_mesh();
        let start = Instant::now();
        let bvh = mesh.calculate_bvh();
        let bvh_build_time = start.elapsed();

        let leaf_count = bvh
            .arena
            .iter()
            .filter(|(_, n)| match n.kind {
                BvhNodeKind::Leaf(_) => true,
                BvhNodeKind::Branch { left: _, right: _ } => false,
            })
            .count();

        let branch_count = bvh.arena.len() - leaf_count;

        dbg!(leaf_count);
        dbg!(branch_count);
        dbg!(bvh_build_time);

        assert_eq!(leaf_count - 1, branch_count)
    }

    #[test]
    fn test_bvh_traversal() {
        let mesh = load_test_mesh();
        let mesh_tri_count = mesh.tris.len();
        let bvh = mesh.calculate_bvh();
        let num_theta: usize = 128;
        let num_phi: usize = 128;
        let num_rays: u32 = (num_theta * num_phi) as u32;
        let rays = generate_test_rays(100.0, num_theta, num_phi);

        let normal_start = Instant::now();
        let normal_collisions: Vec<Option<HitRecord>> = rays
            .iter()
            .map(|ray| ray.intersects(&mesh))
            .collect();
        let normal_duration = normal_start.elapsed();
        let normal_time_per_ray = normal_duration / num_rays;

        let bvh_start = Instant::now();
        let bvh_collisions: Vec<Option<HitRecord>> = rays
            .iter()
            .map(|ray| ray.intersects(&bvh))
            .collect();
        let bvh_duration = bvh_start.elapsed();
        let bvh_time_per_ray = bvh_duration / num_rays;

        dbg!(mesh_tri_count);
        dbg!(normal_duration);
        dbg!(normal_time_per_ray);
        dbg!(bvh_duration);
        dbg!(bvh_time_per_ray);

        for (normal, bvh) in normal_collisions
            .iter()
            .zip(bvh_collisions.iter())
        {
            match (normal, bvh) {
                (Some(a), Some(b)) => {
                    assert_approx_vec3_eq(a.position, b.position);
                    assert_approx_f32_eq(a.length, b.length);
                    assert_approx_vec3_eq(a.incident_direction.into(), b.incident_direction.into());
                }
                _ => {
                    assert_eq!(normal, bvh)
                }
            }
        }
    }

    fn assert_approx_f32_eq(left: f32, right: f32) {
        let epsilon: f32 = 1e-4;
        let difference = (left - right).abs();
        assert!(
            difference < epsilon,
            "{left} and {right} are not approx equal. Difference: {difference}"
        );
    }

    fn assert_approx_vec3_eq(left: Vec3, right: Vec3) {
        assert_approx_f32_eq(left.x, right.x);
        assert_approx_f32_eq(left.y, right.y);
        assert_approx_f32_eq(left.z, right.z);
    }

    fn load_test_mesh() -> Mesh {
        Mesh::suzanne()
    }

    fn generate_test_rays(radius: f32, num_theta: usize, num_phi: usize) -> Vec<Ray> {
        let mut rays = Vec::new();

        for i in 0..num_theta {
            let theta = std::f32::consts::PI * (i as f32 + 0.5) / num_theta as f32;

            for j in 0..num_phi {
                let phi = 2.0 * std::f32::consts::PI * j as f32 / num_phi as f32;

                let x = radius * theta.sin() * phi.cos();
                let y = radius * theta.sin() * phi.sin();
                let z = radius * theta.cos();

                let origin = Vec3::new(x, y, z);
                let direction = UnitVec3::new(-origin);

                rays.push(Ray::new(origin, direction));
            }
        }

        rays
    }
}
