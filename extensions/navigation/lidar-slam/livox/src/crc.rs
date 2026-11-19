//! CRC-32 of the data packet's `crc32` field.
//!
//! The Mid-360 protocol (Livox wiki, "Livox LiDAR Communication Protocol --
//! Mid360", section 6 "CRC Algorithm") specifies CRC-32 with polynomial
//! 0x04C11DB7, initial value 0xFFFFFFFF, result XOR 0xFFFFFFFF and both input
//! and output reflected: the common IEEE 802.3 CRC-32 (check value
//! `crc32(b"123456789") == 0xCBF43926`).

/// Reflected form of polynomial 0x04C11DB7.
const POLY_REFLECTED: u32 = 0xEDB8_8320;

const TABLE: [u32; 256] = build_table();

const fn build_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                POLY_REFLECTED ^ (c >> 1)
            } else {
                c >> 1
            };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

/// CRC-32 (IEEE) of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c = TABLE[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    c ^ 0xFFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value_matches_the_catalogue() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }
}
