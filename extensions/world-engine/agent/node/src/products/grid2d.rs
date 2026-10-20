//! Raster helpers shared by the 2-D products: the exact distance transform of
//! a free-space mask and its 4-connected components.

use std::collections::VecDeque;

/// Exact Euclidean distance, in cells, from every cell of a `w × h` row-major
/// mask to the nearest cell that is not free; everything outside the grid
/// counts as not free. Blocked cells are 0.
pub(crate) fn distance_to_blocked(free: &[bool], w: usize, h: usize) -> Vec<f64> {
    // A one-cell blocked frame makes the grid border a boundary.
    let (pw, ph) = (w + 2, h + 2);
    const INF: f32 = 1e20;
    let mut sq = vec![0.0f32; pw * ph];
    for j in 0..h {
        for i in 0..w {
            if free[j * w + i] {
                sq[(j + 1) * pw + i + 1] = INF;
            }
        }
    }
    let mut line = Vec::with_capacity(pw.max(ph));
    let mut out = vec![0.0f32; pw.max(ph)];
    for j in 0..ph {
        line.clear();
        line.extend_from_slice(&sq[j * pw..(j + 1) * pw]);
        crate::world_descriptors::transform_1d(&line, &mut out[..pw]);
        sq[j * pw..(j + 1) * pw].copy_from_slice(&out[..pw]);
    }
    for i in 0..pw {
        line.clear();
        line.extend((0..ph).map(|j| sq[j * pw + i]));
        crate::world_descriptors::transform_1d(&line, &mut out[..ph]);
        for (j, v) in out[..ph].iter().enumerate() {
            sq[j * pw + i] = *v;
        }
    }
    let mut d = vec![0.0; w * h];
    for j in 0..h {
        for i in 0..w {
            d[j * w + i] = (sq[(j + 1) * pw + i + 1].max(0.0) as f64).sqrt();
        }
    }
    d
}

/// 4-connected component labels of the set cells (0 = unset) and the number
/// of components.
pub(crate) fn components(mask: &[bool], w: usize, h: usize) -> (Vec<u32>, u32) {
    let mut labels = vec![0u32; w * h];
    let mut next = 0u32;
    let mut queue = VecDeque::new();
    for start in 0..labels.len() {
        if labels[start] != 0 || !mask[start] {
            continue;
        }
        next += 1;
        labels[start] = next;
        queue.push_back(start);
        while let Some(k) = queue.pop_front() {
            for n in neighbours4(k, w, h).into_iter().flatten() {
                if labels[n] == 0 && mask[n] {
                    labels[n] = next;
                    queue.push_back(n);
                }
            }
        }
    }
    (labels, next)
}

/// The 4-neighbours of cell `k` inside the grid.
pub(crate) fn neighbours4(k: usize, w: usize, h: usize) -> [Option<usize>; 4] {
    let (i, j) = (k % w, k / w);
    [
        (i > 0).then(|| k - 1),
        (i + 1 < w).then(|| k + 1),
        (j > 0).then(|| k - w),
        (j + 1 < h).then(|| k + w),
    ]
}

/// Morphological dilation (`all == false`: a cell is set when any cell of its
/// `(2r+1)²` square is) or erosion (`all == true`: when every cell of the
/// square inside the grid is) of a `w × h` mask, done separably.
pub(crate) fn square_filter(mask: &[bool], w: usize, h: usize, r: usize, all: bool) -> Vec<bool> {
    let pass = |src: &[bool], along_x: bool| -> Vec<bool> {
        let mut out = vec![false; w * h];
        for j in 0..h {
            for i in 0..w {
                let (c, n) = if along_x { (i, w) } else { (j, h) };
                let mut window = (c.saturating_sub(r)..(c + r + 1).min(n)).map(|t| {
                    if along_x {
                        src[j * w + t]
                    } else {
                        src[t * w + i]
                    }
                });
                out[j * w + i] = if all {
                    window.all(|v| v)
                } else {
                    window.any(|v| v)
                };
            }
        }
        out
    };
    pass(&pass(mask, true), false)
}
