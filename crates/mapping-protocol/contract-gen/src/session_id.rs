//! `session_id = <device_id>-<UTC yyyymmddThhmmssZ>-<4 hex>`.

/// Build a session id from the device id, the UTC start time in Unix seconds
/// and a 16-bit disambiguator (random, or a counter).
pub fn session_id(device_id: &str, unix_secs: i64, suffix: u16) -> String {
    let days = unix_secs.div_euclid(86_400);
    let secs = unix_secs.rem_euclid(86_400);
    let (y, m, d) = civil_from_days(days);
    format!(
        "{device_id}-{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z-{suffix:04x}",
        secs / 3600,
        (secs / 60) % 60,
        secs % 60
    )
}

/// Proleptic Gregorian date from days since 1970-01-01 (H. Hinnant).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = yoe + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_utc_and_suffix() {
        assert_eq!(
            session_id("visor-1", 0, 0xab),
            "visor-1-19700101T000000Z-00ab"
        );
        // 2026-10-19T10:15:00Z
        assert_eq!(
            session_id("visor-1", 1_792_404_900, 0x3fa2),
            "visor-1-20261019T101500Z-3fa2"
        );
        // Leap day.
        assert_eq!(session_id("d", 951_782_400, 1), "d-20000229T000000Z-0001");
    }
}
