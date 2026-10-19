//! Session id and relative path validation, and `Content-Range` parsing.

/// Longest accepted session id.
pub(crate) const MAX_ID_LEN: usize = 128;
/// Deepest accepted relative path, in segments.
pub(crate) const MAX_DEPTH: usize = 6;

/// One name: non-empty, `[A-Za-z0-9._-]`, no leading or trailing `.` (rules
/// out `.`/`..` and the hidden bookkeeping names), and not a name Windows
/// maps to a device.
fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-'))
        && !name.starts_with('.')
        && !name.ends_with('.')
        && !windows_device(name)
}

/// `CON`, `PRN`, `AUX`, `NUL`, `COM<n>`, `LPT<n>`, with any extension.
fn windows_device(name: &str) -> bool {
    let stem = name.split('.').next().unwrap_or("").to_ascii_uppercase();
    match stem.as_str() {
        "CON" | "PRN" | "AUX" | "NUL" => true,
        s if s.len() == 4 && (s.starts_with("COM") || s.starts_with("LPT")) => {
            s.as_bytes()[3].is_ascii_digit()
        }
        _ => false,
    }
}

/// A session id: `[A-Za-z0-9._-]`, at most [`MAX_ID_LEN`] bytes.
pub(crate) fn valid_session_id(id: &str) -> bool {
    id.len() <= MAX_ID_LEN && valid_name(id)
}

/// A `/`-separated path inside a session: at most [`MAX_DEPTH`] segments,
/// every segment a valid name (no `..`, empty, absolute, drive or backslash
/// forms).
pub(crate) fn valid_rel(rel: &str) -> bool {
    let mut depth = 0;
    for seg in rel.split('/') {
        depth += 1;
        if depth > MAX_DEPTH || !valid_name(seg) {
            return false;
        }
    }
    true
}

/// A parsed `Content-Range: bytes <start>-<end>/<total|*>`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ContentRange {
    pub start: u64,
    /// Inclusive.
    pub end: u64,
    pub total: Option<u64>,
}

impl ContentRange {
    /// Number of bytes the range covers.
    pub fn len(&self) -> u64 {
        self.end - self.start + 1
    }
}

/// Parse a `Content-Range` value; `None` when malformed, inverted, or past the
/// declared total.
pub(crate) fn parse_content_range(value: &str) -> Option<ContentRange> {
    let rest = value.trim().strip_prefix("bytes ")?;
    let (span, total) = rest.split_once('/')?;
    let (start, end) = span.split_once('-')?;
    let start: u64 = parse_u64(start)?;
    let end: u64 = parse_u64(end)?;
    let total = match total.trim() {
        "*" => None,
        t => Some(parse_u64(t)?),
    };
    if end < start || total.is_some_and(|t| end >= t) {
        return None;
    }
    Some(ContentRange { start, end, total })
}

fn parse_u64(s: &str) -> Option<u64> {
    let s = s.trim();
    if s.is_empty() || !s.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    s.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids() {
        assert!(valid_session_id("sim-1-20261019T000000Z-0001"));
        assert!(!valid_session_id(""));
        assert!(!valid_session_id(".."));
        assert!(!valid_session_id(".hidden"));
        assert!(!valid_session_id("a/b"));
        assert!(!valid_session_id("nul"));
        assert!(!valid_session_id(&"a".repeat(MAX_ID_LEN + 1)));
        assert!(valid_session_id(&"a".repeat(MAX_ID_LEN)));
    }

    #[test]
    fn rel_paths() {
        assert!(valid_rel("segments/000000.mcap"));
        assert!(valid_rel("a/b/c/d/e/f"));
        for bad in [
            "",
            "..",
            "a/../b",
            "a//b",
            "/abs",
            "a/",
            "a\\b",
            "C:/x",
            ".hidden",
            "a/.part",
            "a.",
            "a b",
            "a/b/c/d/e/f/g",
            "COM1.txt",
        ] {
            assert!(!valid_rel(bad), "{bad:?} accepted");
        }
    }

    #[test]
    fn ranges() {
        assert_eq!(
            parse_content_range("bytes 0-4095/22197"),
            Some(ContentRange {
                start: 0,
                end: 4095,
                total: Some(22197)
            })
        );
        assert_eq!(
            parse_content_range("bytes 10-19/*").map(|r| (r.len(), r.total)),
            Some((10, None))
        );
        for bad in [
            "bytes 5-4/10",
            "bytes 0-10/10",
            "bytes -1-4/10",
            "bytes 0-4",
            "items 0-4/10",
            "bytes +0-4/10",
            "bytes */10",
        ] {
            assert_eq!(parse_content_range(bad), None, "{bad:?} accepted");
        }
    }
}
