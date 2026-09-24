use std::{fs, path::Path};

use glam::{Quat, Vec3};
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

// src/geometry/mesh.rs
const SUZANNE_STL: &[u8] = include_bytes!("../../models/suzanne.stl");

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
        Self::from_stl_bytes(&bytes, recalculate_normals)
    }

    pub fn from_stl_bytes(bytes: &[u8], recalculate_normals: bool) -> Result<Self, MeshError> {
        let tri_count = u32::from_le_bytes(bytes[stl::STL_TRIS_COUNT].try_into()?) as usize;
        let expected = stl::STL_TRIS_START + stl::STL_TRI_SIZE * tri_count;

        let triangle_bytes = bytes
            .get(stl::STL_TRIS_START..stl::STL_TRIS_START + stl::STL_TRI_SIZE * tri_count)
            .ok_or(MeshError::StlSizeError {
                expected,
                tri_count,
                actual: bytes.len(),
            })?;

        let tris: Vec<Tri> = triangle_bytes
            .as_chunks::<{ stl::STL_TRI_SIZE }>()
            .0
            .iter()
            .map(|x| Ok(Tri::from_stl_bytes(x)))
            .collect::<Result<Vec<Tri>, MeshError>>()?;

        let mut mesh = Mesh { tris };

        if recalculate_normals {
            mesh.recalculate_normals();
        }

        Ok(mesh)
    }

    pub fn suzanne() -> Self {
        Self::from_stl_bytes(SUZANNE_STL, true)
            .expect("embedded Suzanne STL should always be valid")
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

    pub fn verts(&self) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(self.tris.len() * 3);
        out.extend(
            self.tris
                .iter()
                .flat_map(|tri| tri.ordered_verts()),
        );
        out
    }

    /// Emits a normal per vert
    pub fn normals(&self) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(self.tris.len() * 3);
        out.extend(
            self.tris
                .iter()
                .flat_map(|tri| [tri.normal.vec3(); 3]),
        );
        out
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
