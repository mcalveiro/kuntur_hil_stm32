//! icd.rs -- Contrato Kuntur-IC v1.3 en Rust.
//!
//! *** ARCHIVO GENERADO -- NO EDITAR A MANO ***
//! Generado por tools/gen_icd.py desde icd_schema/kuntur_ic_v1_3.json.
//! Sin `#[repr(C, packed)]` ni transmute -- empaquetado explícito campo a
//! campo, igual que icd.py con struct.pack/unpack.

pub const MAGIC_STATES: u32 = 0x4B535433;
pub const MAGIC_COMMAND: u32 = 0x434D4432;

pub const STATES_SIZE: usize = 92;
pub const COMMAND_SIZE: usize = 34;

const _: () = assert!(STATES_SIZE == 92);
const _: () = assert!(COMMAND_SIZE == 34);

#[derive(Debug, Clone, Copy, Default)]
pub struct StatesPacket {
    pub t_sim_us: u64,
    pub seq_num: u32,
    pub north: f32,
    pub east: f32,
    pub down: f32,
    pub phi: f32,
    pub theta: f32,
    pub psi: f32,
    pub p: f32,
    pub q: f32,
    pub r: f32,
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub vn: f32,
    pub ve: f32,
    pub vd: f32,
    pub ias: f32,
    pub alpha: f32,
    pub beta: f32,
    pub status: u16,
}

#[derive(Debug)]
pub enum IcdError {
    BadSize(usize),
    BadMagic(u32),
}

impl std::fmt::Display for IcdError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IcdError::BadSize(n) => write!(f, "tamaño {n} != {STATES_SIZE} bytes"),
            IcdError::BadMagic(m) => write!(f, "magic 0x{m:08X} != 0x{MAGIC_STATES:08X}"),
        }
    }
}

pub fn unpack_states(data: &[u8]) -> Result<StatesPacket, IcdError> {
    if data.len() != STATES_SIZE {
        return Err(IcdError::BadSize(data.len()));
    }
    let magic = u32::from_le_bytes(data[0..4].try_into().unwrap());
    if magic != MAGIC_STATES {
        return Err(IcdError::BadMagic(magic));
    }

    let f32_at = |a: usize, b: usize| f32::from_le_bytes(data[a..b].try_into().unwrap());

    let t_sim_us = u64::from_le_bytes(data[4..12].try_into().unwrap());
    let seq_num = u32::from_le_bytes(data[12..16].try_into().unwrap());
    let north = f32_at(16, 20);
    let east = f32_at(20, 24);
    let down = f32_at(24, 28);
    let phi = f32_at(28, 32);
    let theta = f32_at(32, 36);
    let psi = f32_at(36, 40);
    let p = f32_at(40, 44);
    let q = f32_at(44, 48);
    let r = f32_at(48, 52);
    let ax = f32_at(52, 56);
    let ay = f32_at(56, 60);
    let az = f32_at(60, 64);
    let vn = f32_at(64, 68);
    let ve = f32_at(68, 72);
    let vd = f32_at(72, 76);
    let ias = f32_at(76, 80);
    let alpha = f32_at(80, 84);
    let beta = f32_at(84, 88);
    let status = u16::from_le_bytes(data[88..90].try_into().unwrap());

    Ok(StatesPacket {
        t_sim_us,
        seq_num,
        north,
        east,
        down,
        phi,
        theta,
        psi,
        p,
        q,
        r,
        ax,
        ay,
        az,
        vn,
        ve,
        vd,
        ias,
        alpha,
        beta,
        status,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn pack_command(t_sim_us_grant: u64, elevator: f32, aileron: f32, rudder: f32, throttle: f32, ground_elev_m: f32, exec_mode: u16) -> [u8; COMMAND_SIZE] {
    let mut buf = [0u8; COMMAND_SIZE];
    buf[0..4].copy_from_slice(&MAGIC_COMMAND.to_le_bytes());
    buf[4..12].copy_from_slice(&t_sim_us_grant.to_le_bytes());
    buf[12..16].copy_from_slice(&elevator.to_le_bytes());
    buf[16..20].copy_from_slice(&aileron.to_le_bytes());
    buf[20..24].copy_from_slice(&rudder.to_le_bytes());
    buf[24..28].copy_from_slice(&throttle.to_le_bytes());
    buf[28..32].copy_from_slice(&ground_elev_m.to_le_bytes());
    buf[32..34].copy_from_slice(&exec_mode.to_le_bytes());
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_round_trip_size() {
        let buf = pack_command(123_456, 0.1, -0.2, 0.0, 0.5, 589.0, 0);
        assert_eq!(buf.len(), COMMAND_SIZE);
    }

    #[test]
    fn rejects_bad_size() {
        let buf = [0u8; 10];
        assert!(matches!(unpack_states(&buf), Err(IcdError::BadSize(10))));
    }

    #[test]
    fn rejects_bad_magic() {
        let buf = [0u8; STATES_SIZE];
        assert!(matches!(unpack_states(&buf), Err(IcdError::BadMagic(0))));
    }

    #[test]
    fn states_round_trip() {
        let mut buf = [0u8; STATES_SIZE];
        buf[0..4].copy_from_slice(&MAGIC_STATES.to_le_bytes());
    buf[4..12].copy_from_slice(&123_456u64.to_le_bytes());
    buf[12..16].copy_from_slice(&7u32.to_le_bytes());
    buf[16..20].copy_from_slice(&1.0f32.to_le_bytes());
    buf[20..24].copy_from_slice(&2.1f32.to_le_bytes());
    buf[24..28].copy_from_slice(&3.2f32.to_le_bytes());
    buf[28..32].copy_from_slice(&4.3f32.to_le_bytes());
    buf[32..36].copy_from_slice(&5.4f32.to_le_bytes());
    buf[36..40].copy_from_slice(&6.5f32.to_le_bytes());
    buf[40..44].copy_from_slice(&7.6f32.to_le_bytes());
    buf[44..48].copy_from_slice(&8.7f32.to_le_bytes());
    buf[48..52].copy_from_slice(&9.8f32.to_le_bytes());
    buf[52..56].copy_from_slice(&10.9f32.to_le_bytes());
    buf[56..60].copy_from_slice(&12.0f32.to_le_bytes());
    buf[60..64].copy_from_slice(&13.1f32.to_le_bytes());
    buf[64..68].copy_from_slice(&14.2f32.to_le_bytes());
    buf[68..72].copy_from_slice(&15.3f32.to_le_bytes());
    buf[72..76].copy_from_slice(&16.4f32.to_le_bytes());
    buf[76..80].copy_from_slice(&17.5f32.to_le_bytes());
    buf[80..84].copy_from_slice(&18.6f32.to_le_bytes());
    buf[84..88].copy_from_slice(&19.7f32.to_le_bytes());
    buf[88..90].copy_from_slice(&0u16.to_le_bytes());
        let sp = unpack_states(&buf).expect("debe decodificar");
    assert_eq!(sp.t_sim_us, 123_456);
    assert_eq!(sp.seq_num, 7);
    assert!((sp.north - 1.0f32).abs() < 1e-4);
    assert!((sp.east - 2.1f32).abs() < 1e-4);
    assert!((sp.down - 3.2f32).abs() < 1e-4);
    assert!((sp.phi - 4.3f32).abs() < 1e-4);
    assert!((sp.theta - 5.4f32).abs() < 1e-4);
    assert!((sp.psi - 6.5f32).abs() < 1e-4);
    assert!((sp.p - 7.6f32).abs() < 1e-4);
    assert!((sp.q - 8.7f32).abs() < 1e-4);
    assert!((sp.r - 9.8f32).abs() < 1e-4);
    assert!((sp.ax - 10.9f32).abs() < 1e-4);
    assert!((sp.ay - 12.0f32).abs() < 1e-4);
    assert!((sp.az - 13.1f32).abs() < 1e-4);
    assert!((sp.vn - 14.2f32).abs() < 1e-4);
    assert!((sp.ve - 15.3f32).abs() < 1e-4);
    assert!((sp.vd - 16.4f32).abs() < 1e-4);
    assert!((sp.ias - 17.5f32).abs() < 1e-4);
    assert!((sp.alpha - 18.6f32).abs() < 1e-4);
    assert!((sp.beta - 19.7f32).abs() < 1e-4);
    }
}
