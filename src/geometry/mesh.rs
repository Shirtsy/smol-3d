use std::{fs, path::Path};

use glam::Quat;
use thiserror::Error;

use crate::{
    geometry::{
        aabb::Aabb,
        bvh::Bvh,
        ray::{HitRecord, Ray},
        stl,
        tri::Tri,
    },
    traits::{Bounded, Intersects},
};

#[derive(Error, Debug)]
pub enum MeshError {
    #[error("failed to read file: {0}")]
    Io(#[from] std::io::Error),

    #[error("failed to parse slice into tri: {0}")]
    Parse(#[from] std::array::TryFromSliceError),

    #[error(
        "expected stl length of {} for {} triangles, but was {}",
        expected,
        tri_count,
        actual
    )]
    StlSizeError {
        expected: usize,
        tri_count: usize,
        actual: usize,
    },
}

#[derive(Debug)]
pub struct Mesh {
    pub tris: Vec<Tri>,
}

impl Mesh {
    pub fn from_stl_file(stl_path: &Path, recalculate_normals: bool) -> Result<Self, MeshError> {
        let bytes = fs::read(stl_path)?;
        let tri_count = u32::from_le_bytes(bytes[stl::STL_TRIS_COUNT].try_into()?) as usize;
        let expected = stl::STL_TRIS_START + stl::STL_TRI_SIZE * tri_count;

        let triangle_bytes = bytes
            .get(stl::STL_TRIS_START..stl::STL_TRIS_START + stl::STL_TRI_SIZE * tri_count)
            .ok_or(MeshError::StlSizeError {
                expected,
                tri_count,
                actual: bytes.len(),
            })?;

        let mut tris: Vec<Tri> = triangle_bytes
            .chunks_exact(stl::STL_TRI_SIZE)
            .map(|x| Ok(Tri::from_stl_bytes(x.try_into()?)))
            .collect::<Result<Vec<Tri>, MeshError>>()?;

        if recalculate_normals {
            tris = tris
                .iter()
                .map(|x| x.recalculate_normal())
                .collect();
        }

        Ok(Mesh { tris })
    }

    pub fn calculate_aabb(&self) -> Aabb {
        Aabb::from_mesh(self)
    }

    pub fn recalculate_normals(&mut self) {
        for tri in &mut self.tris {
            *tri = tri.recalculate_normal();
        }
    }

    pub fn apply_rotation(&mut self, rotation: Quat) {
        for tri in &mut self.tris {
            for vert in &mut tri.vertices {
                *vert = rotation * *vert;
            }
        }
        self.recalculate_normals()
    }

    pub fn calculate_bvh(&self) -> Bvh {
        Bvh::from_tris(self.tris.as_slice())
    }
}

impl Bounded for Mesh {
    fn aabb(&self) -> Aabb {
        Aabb::from_mesh(self)
    }
}

impl Intersects<Ray> for &Mesh {
    fn intersects(&self, shape: Ray) -> Option<HitRecord> {
        shape.intersects(self.tris.as_slice())
    }
}
