//! Files only the user running the node may open: the owner token and the
//! issued-credential store. On unix that is mode 0600; on Windows a protected
//! DACL (no inherited entries) granting the current user alone full access.
//! The restriction is applied before any secret is written.

use std::fs::File;
use std::path::Path;

/// Create (or truncate) `path` for writing, restricted to the current user
/// before the caller writes anything into it.
pub(crate) fn create(path: &Path) -> std::io::Result<File> {
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt as _;
        opts.mode(0o600);
    }
    let file = opts.open(path)?;
    restrict(path)?;
    Ok(file)
}

/// Restrict an existing file to the current user (an earlier file keeps its
/// old mode through `open`, so this is applied every time).
#[cfg(unix)]
pub(crate) fn restrict(path: &Path) -> std::io::Result<()> {
    use std::os::unix::fs::PermissionsExt as _;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
}

#[cfg(windows)]
pub(crate) fn restrict(path: &Path) -> std::io::Result<()> {
    win::restrict(path)
}

#[cfg(not(any(unix, windows)))]
pub(crate) fn restrict(_path: &Path) -> std::io::Result<()> {
    Ok(())
}

#[cfg(windows)]
mod win {
    use std::ffi::OsStr;
    use std::io;
    use std::os::windows::ffi::OsStrExt as _;
    use std::path::Path;
    use std::ptr::null_mut;

    use windows_sys::core::PWSTR;
    use windows_sys::Win32::Foundation::{CloseHandle, LocalFree, ERROR_SUCCESS, HANDLE};
    use windows_sys::Win32::Security::Authorization::{
        ConvertSidToStringSidW, ConvertStringSecurityDescriptorToSecurityDescriptorW,
        SetNamedSecurityInfoW, SDDL_REVISION_1, SE_FILE_OBJECT,
    };
    use windows_sys::Win32::Security::{
        GetSecurityDescriptorDacl, GetTokenInformation, TokenUser, ACL, DACL_SECURITY_INFORMATION,
        PROTECTED_DACL_SECURITY_INFORMATION, PSECURITY_DESCRIPTOR, TOKEN_QUERY, TOKEN_USER,
    };
    use windows_sys::Win32::System::Threading::{GetCurrentProcess, OpenProcessToken};

    pub(super) fn wide(s: &OsStr) -> Vec<u16> {
        s.encode_wide().chain(std::iter::once(0)).collect()
    }

    /// Read a NUL-terminated wide string the system allocated, then free it.
    ///
    /// # Safety
    /// `s` is a valid NUL-terminated string from `LocalAlloc`.
    pub(super) unsafe fn take_wide(s: PWSTR) -> String {
        let mut len = 0;
        while *s.add(len) != 0 {
            len += 1;
        }
        let out = String::from_utf16_lossy(std::slice::from_raw_parts(s, len));
        LocalFree(s.cast());
        out
    }

    /// The current user's SID as a string (`S-1-5-21-...`).
    pub(super) fn current_user_sid() -> io::Result<String> {
        // SAFETY: the token handle is closed on every path; the TOKEN_USER
        // buffer is u64-aligned and sized by the first call.
        unsafe {
            let mut token: HANDLE = null_mut();
            if OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &mut token) == 0 {
                return Err(io::Error::last_os_error());
            }
            let mut len = 0u32;
            GetTokenInformation(token, TokenUser, null_mut(), 0, &mut len);
            let mut buf = vec![0u64; (len as usize).div_ceil(8)];
            let ok = GetTokenInformation(token, TokenUser, buf.as_mut_ptr().cast(), len, &mut len);
            let err = io::Error::last_os_error();
            CloseHandle(token);
            if ok == 0 {
                return Err(err);
            }
            let user = &*(buf.as_ptr() as *const TOKEN_USER);
            let mut s: PWSTR = null_mut();
            if ConvertSidToStringSidW(user.User.Sid, &mut s) == 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(take_wide(s))
        }
    }

    /// The DACL `restrict` sets: protected, the current user full access.
    pub(super) fn owner_only_sddl() -> io::Result<String> {
        Ok(format!("D:P(A;;FA;;;{})", current_user_sid()?))
    }

    pub(super) fn restrict(path: &Path) -> io::Result<()> {
        let sddl = wide(OsStr::new(&owner_only_sddl()?));
        let target = wide(path.as_os_str());
        // SAFETY: the descriptor is freed after use; the DACL points into it.
        unsafe {
            let mut sd: PSECURITY_DESCRIPTOR = null_mut();
            if ConvertStringSecurityDescriptorToSecurityDescriptorW(
                sddl.as_ptr(),
                SDDL_REVISION_1,
                &mut sd,
                null_mut(),
            ) == 0
            {
                return Err(io::Error::last_os_error());
            }
            let (mut present, mut defaulted) = (0, 0);
            let mut dacl: *mut ACL = null_mut();
            if GetSecurityDescriptorDacl(sd, &mut present, &mut dacl, &mut defaulted) == 0 {
                let err = io::Error::last_os_error();
                LocalFree(sd);
                return Err(err);
            }
            let rc = SetNamedSecurityInfoW(
                target.as_ptr(),
                SE_FILE_OBJECT,
                DACL_SECURITY_INFORMATION | PROTECTED_DACL_SECURITY_INFORMATION,
                null_mut(),
                null_mut(),
                dacl,
                std::ptr::null(),
            );
            LocalFree(sd);
            if rc != ERROR_SUCCESS {
                return Err(io::Error::from_raw_os_error(rc as i32));
            }
        }
        Ok(())
    }
}

/// Test support: the file's DACL read back from the system as SDDL, and the
/// owner-only DACL it must equal.
#[cfg(all(test, windows))]
pub(crate) mod acl {
    use std::path::Path;
    use std::ptr::null_mut;

    use windows_sys::core::PWSTR;
    use windows_sys::Win32::Foundation::{LocalFree, ERROR_SUCCESS};
    use windows_sys::Win32::Security::Authorization::{
        ConvertSecurityDescriptorToStringSecurityDescriptorW, GetNamedSecurityInfoW,
        SDDL_REVISION_1, SE_FILE_OBJECT,
    };
    use windows_sys::Win32::Security::{DACL_SECURITY_INFORMATION, PSECURITY_DESCRIPTOR};

    pub(crate) fn read_dacl(path: &Path) -> String {
        let target = super::win::wide(path.as_os_str());
        // SAFETY: both system allocations are freed.
        unsafe {
            let mut sd: PSECURITY_DESCRIPTOR = null_mut();
            let rc = GetNamedSecurityInfoW(
                target.as_ptr(),
                SE_FILE_OBJECT,
                DACL_SECURITY_INFORMATION,
                null_mut(),
                null_mut(),
                null_mut(),
                null_mut(),
                &mut sd,
            );
            assert_eq!(rc, ERROR_SUCCESS, "GetNamedSecurityInfoW");
            let mut s: PWSTR = null_mut();
            let ok = ConvertSecurityDescriptorToStringSecurityDescriptorW(
                sd,
                SDDL_REVISION_1,
                DACL_SECURITY_INFORMATION,
                &mut s,
                null_mut(),
            );
            LocalFree(sd);
            assert_ne!(
                ok, 0,
                "ConvertSecurityDescriptorToStringSecurityDescriptorW"
            );
            super::win::take_wide(s)
        }
    }

    /// The single entry the DACL must hold: the current user, full access.
    pub(crate) fn owner_ace() -> String {
        format!("(A;;FA;;;{})", super::win::current_user_sid().unwrap())
    }
}

/// Assert `path` is readable by its owner alone.
#[cfg(test)]
pub(crate) fn assert_owner_only(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt as _;
        let mode = std::fs::metadata(path).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600, "{}", path.display());
    }
    #[cfg(windows)]
    {
        // `D:P...` is protected (nothing inherited from the directory); the
        // system may add the auto-inherited flag `AI`, which grants nothing.
        let sddl = acl::read_dacl(path);
        let (flags, aces) = sddl.split_at(sddl.find('(').unwrap_or(sddl.len()));
        assert!(flags.starts_with("D:P"), "{}: {sddl}", path.display());
        assert_eq!(aces, acl::owner_ace(), "{}", path.display());
    }
}

#[cfg(test)]
mod tests {
    use std::io::Write as _;

    use super::*;

    #[test]
    fn a_created_file_is_owner_only_even_when_it_existed_open() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("secret");
        std::fs::write(&p, b"old").unwrap();
        create(&p).unwrap().write_all(b"new").unwrap();
        assert_eq!(std::fs::read(&p).unwrap(), b"new");
        assert_owner_only(&p);
    }
}
