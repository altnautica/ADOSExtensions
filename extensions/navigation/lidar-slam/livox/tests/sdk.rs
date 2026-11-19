//! Run-time SDK binding. The load-failure test always runs with the `sdk`
//! feature; the start/stop test needs the library built by `build-sdk2.sh`
//! and named in `LIVOX_SDK2_LIB`, and binds loopback ports only.
#![cfg(all(feature = "sdk", unix))]

use std::path::PathBuf;
use std::sync::Mutex;

use ados_livox::{DecoderConfig, SdkDriver, SdkDriverConfig, SdkError, SdkNetConfig};

/// The SDK is a process-wide singleton: tests take turns.
static SDK: Mutex<()> = Mutex::new(());

fn cfg(library: PathBuf, config_json: PathBuf) -> SdkDriverConfig {
    SdkDriverConfig {
        library,
        config_json,
        decoder: DecoderConfig::default(),
        enable_imu: true,
        queue: 64,
    }
}

#[test]
fn missing_library_fails_closed() {
    let _turn = SDK.lock().unwrap_or_else(|e| e.into_inner());
    let err = SdkDriver::start(
        cfg(
            "/nonexistent/liblivox_lidar_sdk_shared.so".into(),
            "/dev/null".into(),
        ),
        |_| {},
    )
    .err()
    .expect("no library");
    assert!(matches!(err, SdkError::Load { .. }), "{err}");
}

#[test]
fn starts_and_stops_with_the_real_library() {
    let _turn = SDK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(lib) = std::env::var_os("LIVOX_SDK2_LIB") else {
        eprintln!("LIVOX_SDK2_LIB unset; skipped");
        return;
    };
    let dir = std::env::temp_dir().join(format!("ados-livox-sdk-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let json = dir.join("mid360.json");
    std::fs::write(&json, SdkNetConfig::new("127.0.0.1").to_json().to_string()).unwrap();
    let driver = SdkDriver::start(cfg(lib.into(), json), |_| {}).expect("sdk start");
    assert!(matches!(
        SdkDriver::start(cfg("unused".into(), "unused".into()), |_| {}),
        Err(SdkError::AlreadyRunning)
    ));
    std::thread::sleep(std::time::Duration::from_millis(300));
    assert_eq!(driver.stats().dropped, 0);
    drop(driver);
    std::fs::remove_dir_all(&dir).ok();
}
