//! Host monotonic time, the timebase revolutions are stamped on.

/// Host `CLOCK_MONOTONIC`, nanoseconds (the sensor bus timebase).
#[cfg(unix)]
pub fn mono_ns() -> u64 {
    match nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC) {
        Ok(ts) => (ts.tv_sec() as i128 * 1_000_000_000 + ts.tv_nsec() as i128).max(0) as u64,
        Err(_) => 0,
    }
}

/// Monotonic nanoseconds since the first call (no `CLOCK_MONOTONIC` off
/// Unix; enough for the bench tools that run there).
#[cfg(not(unix))]
pub fn mono_ns() -> u64 {
    use std::sync::LazyLock;
    use std::time::Instant;
    static ORIGIN: LazyLock<Instant> = LazyLock::new(Instant::now);
    ORIGIN.elapsed().as_nanos() as u64
}
