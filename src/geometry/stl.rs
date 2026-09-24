use glam::Vec3;

pub(crate) const STL_TRIS_COUNT: std::ops::Range<usize> = 80..84;
pub(crate) const STL_TRIS_START: usize = 84;
pub(crate) const STL_TRI_SIZE: usize = 50;

pub(crate) fn vector_from_bytes(bytes: [u8; 12]) -> Vec3 {
    let x = f32::from_le_bytes(
        bytes[0..4]
            .try_into()
            .unwrap(),
    );
    let y = f32::from_le_bytes(
        bytes[4..8]
            .try_into()
            .unwrap(),
    );
    let z = f32::from_le_bytes(
        bytes[8..12]
            .try_into()
            .unwrap(),
    );
    Vec3::from([x, y, z])
}
