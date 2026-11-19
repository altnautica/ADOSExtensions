//! Zhang–Suen thinning (Zhang & Suen 1984): a binary mask peeled to its
//! one-cell-wide medial axis by two alternating deletion sub-passes.

/// The medial axis of the set cells of a `w × h` row-major mask. Cells outside
/// the grid count as unset.
pub(crate) fn thin(mask: &[bool], w: usize, h: usize) -> Vec<bool> {
    let mut m = mask.to_vec();
    let at = |m: &[bool], i: i64, j: i64| -> bool {
        i >= 0 && j >= 0 && (i as usize) < w && (j as usize) < h && m[j as usize * w + i as usize]
    };
    let mut doomed = Vec::new();
    loop {
        let mut changed = false;
        for pass in 0..2 {
            doomed.clear();
            for j in 0..h as i64 {
                for i in 0..w as i64 {
                    if !at(&m, i, j) {
                        continue;
                    }
                    // P2..P9 clockwise from north.
                    let p = [
                        at(&m, i, j - 1),
                        at(&m, i + 1, j - 1),
                        at(&m, i + 1, j),
                        at(&m, i + 1, j + 1),
                        at(&m, i, j + 1),
                        at(&m, i - 1, j + 1),
                        at(&m, i - 1, j),
                        at(&m, i - 1, j - 1),
                    ];
                    let b = p.iter().filter(|&&v| v).count();
                    let a = (0..8).filter(|&k| !p[k] && p[(k + 1) % 8]).count();
                    let (n, e, s, wst) = (p[0], p[2], p[4], p[6]);
                    let side = if pass == 0 {
                        !(n && e && s) && !(e && s && wst)
                    } else {
                        !(n && e && wst) && !(n && s && wst)
                    };
                    if (2..=6).contains(&b) && a == 1 && side {
                        doomed.push(j as usize * w + i as usize);
                    }
                }
            }
            for &k in &doomed {
                m[k] = false;
            }
            changed |= !doomed.is_empty();
        }
        if !changed {
            return m;
        }
    }
}
