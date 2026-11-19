//! `device = "auto"`: find an RPLIDAR on a USB serial bridge.
//!
//! Every `/dev/serial/by-id` link is resolved to its tty, kept when the USB
//! device behind it reports `idVendor:idProduct` `10c4:ea60` in sysfs (the
//! CP2102 bridge the device ships with), then probed with `GET_INFO`. The
//! first port that answers is used, by its stable `by-id` path.

#[cfg(any(target_os = "linux", test))]
use std::path::Path;
use std::time::Duration;

use crate::error::RplidarError;
use crate::proto::DeviceInfo;
use crate::transport;

/// USB vendor id of the serial bridge.
pub const USB_VID: u16 = 0x10c4;
/// USB product id of the serial bridge.
pub const USB_PID: u16 = 0xea60;
/// How long a probed port may take to answer `GET_INFO`.
pub const PROBE_TIMEOUT: Duration = Duration::from_millis(500);

/// The first answering RPLIDAR's port path and info.
#[cfg(target_os = "linux")]
pub fn find(baud: u32) -> Result<(String, DeviceInfo), RplidarError> {
    let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") else {
        return Err(RplidarError::NotFound);
    };
    let mut links: Vec<_> = entries.flatten().map(|e| e.path()).collect();
    links.sort();
    for link in links {
        let Ok(tty) = std::fs::canonicalize(&link) else {
            continue;
        };
        let Some(name) = tty.file_name() else {
            continue;
        };
        let sys = Path::new("/sys/class/tty").join(name).join("device");
        if usb_id(&sys) != Some((USB_VID, USB_PID)) {
            continue;
        }
        let path = link.to_string_lossy().into_owned();
        match probe(&path, baud) {
            Ok(info) => {
                tracing::info!(
                    path = %path,
                    model = info.model,
                    firmware = %info.firmware(),
                    hardware = info.hardware,
                    serial = %info.serial_hex(),
                    "rplidar_info"
                );
                return Ok((path, info));
            }
            Err(e) => tracing::debug!(path = %path, error = %e, "rplidar_probe_no_answer"),
        }
    }
    Err(RplidarError::NotFound)
}

/// `auto` needs `/dev/serial/by-id` and sysfs.
#[cfg(not(target_os = "linux"))]
pub fn find(_baud: u32) -> Result<(String, DeviceInfo), RplidarError> {
    Err(RplidarError::AutoUnsupported)
}

/// Open `path`, stop any running scan and ask for the device info.
pub fn probe(path: &str, baud: u32) -> Result<DeviceInfo, RplidarError> {
    let mut link = transport::open(path, baud)?;
    transport::stop_and_drain(link.as_mut())?;
    transport::query_info(link.as_mut(), PROBE_TIMEOUT)
}

/// `(idVendor, idProduct)` of the USB device a tty's sysfs `device` node sits
/// under: the nearest ancestor carrying both files.
#[cfg(any(target_os = "linux", test))]
fn usb_id(sys_device: &Path) -> Option<(u16, u16)> {
    let mut dir = std::fs::canonicalize(sys_device).ok()?;
    loop {
        let vid = std::fs::read_to_string(dir.join("idVendor"));
        let pid = std::fs::read_to_string(dir.join("idProduct"));
        if let (Ok(v), Ok(p)) = (vid, pid) {
            let v = u16::from_str_radix(v.trim(), 16).ok()?;
            let p = u16::from_str_radix(p.trim(), 16).ok()?;
            return Some((v, p));
        }
        if !dir.pop() {
            return None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn the_usb_id_comes_from_the_nearest_ancestor_with_both_files() {
        let tmp = tempfile::tempdir().unwrap();
        let usb = tmp.path().join("usb1/1-1");
        // The interface directory (`1-1:1.0` in sysfs; no `:` in Windows
        // file names) holds the tty.
        let intf = usb.join("1-1.intf0");
        let iface = intf.join("ttyUSB0");
        fs::create_dir_all(&iface).unwrap();
        fs::write(usb.join("idVendor"), "10c4\n").unwrap();
        fs::write(usb.join("idProduct"), "ea60\n").unwrap();
        // A file of one kind only, nearer than the real device, is skipped.
        fs::write(intf.join("idVendor"), "dead\n").unwrap();
        assert_eq!(usb_id(&iface), Some((USB_VID, USB_PID)));
    }

    #[test]
    fn a_tty_not_under_a_usb_device_has_no_usb_id() {
        let tmp = tempfile::tempdir().unwrap();
        let tty = tmp.path().join("platform/serial8250/tty/ttyS0");
        fs::create_dir_all(&tty).unwrap();
        assert_eq!(usb_id(&tty), None);
        assert_eq!(usb_id(&tmp.path().join("missing")), None);
    }
}
