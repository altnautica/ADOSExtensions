//! A byte-bounded queue between the capture loop and the recorder's writer
//! thread that drops its OLDEST entry when a new one would pass the cap, so
//! the producer never blocks and RAM never exceeds its share of
//! `atlas.memory_budget_mb`. Used for keyframe JPEGs and polar scan records.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use parking_lot::Mutex;

/// One queued entry and the bytes it holds.
struct Entry<T> {
    value: T,
    bytes: usize,
}

/// A FIFO bounded by the bytes its entries hold.
pub struct BoundedQueue<T> {
    inner: Mutex<(VecDeque<Entry<T>>, usize)>,
    cap: AtomicUsize,
    dropped: AtomicU64,
}

impl<T> BoundedQueue<T> {
    /// A queue holding at most `cap_bytes`.
    pub fn new(cap_bytes: usize) -> Self {
        Self {
            inner: Mutex::new((VecDeque::new(), 0)),
            cap: AtomicUsize::new(cap_bytes),
            dropped: AtomicU64::new(0),
        }
    }

    /// Change the cap; entries past it go on the next push.
    pub fn set_cap(&self, cap_bytes: usize) {
        self.cap.store(cap_bytes, Ordering::Relaxed);
    }

    /// Queue `value` (`bytes` large), dropping the oldest entries until it
    /// fits. An entry larger than the whole cap replaces everything queued,
    /// so the newest data always lands. Returns how many were dropped.
    pub fn push(&self, value: T, bytes: usize) -> u64 {
        let cap = self.cap.load(Ordering::Relaxed);
        let mut g = self.inner.lock();
        let mut n = 0u64;
        while g.1 + bytes > cap {
            let Some(old) = g.0.pop_front() else { break };
            g.1 -= old.bytes;
            n += 1;
        }
        g.0.push_back(Entry { value, bytes });
        g.1 += bytes;
        drop(g);
        if n > 0 {
            self.dropped.fetch_add(n, Ordering::Relaxed);
        }
        n
    }

    /// Take everything queued, oldest first.
    pub fn drain(&self) -> Vec<T> {
        let mut g = self.inner.lock();
        g.1 = 0;
        g.0.drain(..).map(|e| e.value).collect()
    }

    /// Bytes queued now.
    pub fn bytes(&self) -> usize {
        self.inner.lock().1
    }

    /// Entries dropped to stay under the cap since the queue was made.
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_oldest_entries_go_first_and_are_counted() {
        let q = BoundedQueue::new(100);
        assert_eq!(q.push("a", 40), 0);
        assert_eq!(q.push("b", 40), 0);
        assert_eq!(q.push("c", 40), 1, "a goes to make room for c");
        assert_eq!(q.bytes(), 80);
        assert_eq!(q.dropped(), 1);
        assert_eq!(q.drain(), vec!["b", "c"]);
        assert_eq!(q.bytes(), 0);
    }

    #[test]
    fn an_entry_bigger_than_the_cap_still_lands_alone() {
        let q = BoundedQueue::new(10);
        q.push(1, 4);
        q.push(2, 4);
        assert_eq!(q.push(3, 50), 2);
        assert_eq!(q.drain(), vec![3]);
    }

    #[test]
    fn a_lowered_cap_bites_on_the_next_push() {
        let q = BoundedQueue::new(100);
        q.push(1, 30);
        q.push(2, 30);
        q.set_cap(40);
        assert_eq!(q.push(3, 10), 1);
        assert_eq!(q.drain(), vec![2, 3]);
    }
}
