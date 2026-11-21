//! The bounded hand-off from the sensor paths to the map thread. A push
//! never waits for the map: a full queue drops its oldest input and counts
//! it, so a slow map frame costs the map data, never the LiDAR path.

use std::collections::VecDeque;
use std::time::Duration;

use parking_lot::{Condvar, Mutex};

use super::feeder::Input;

/// Inputs waiting for the map thread at most.
pub const QUEUE_DEPTH: usize = 4;

pub struct Queue {
    state: Mutex<State>,
    ready: Condvar,
    cap: usize,
}

struct State {
    items: VecDeque<Input>,
    drops: u64,
    stop: bool,
}

impl Queue {
    pub fn new(cap: usize) -> Self {
        let cap = cap.max(1);
        Self {
            state: Mutex::new(State {
                items: VecDeque::with_capacity(cap),
                drops: 0,
                stop: false,
            }),
            ready: Condvar::new(),
            cap,
        }
    }

    /// Queue `input`, dropping the oldest waiting input when full. Returns
    /// whether one was dropped.
    pub fn push(&self, input: Input) -> bool {
        let evicted = {
            let mut s = self.state.lock();
            let evicted = if s.items.len() >= self.cap {
                s.drops += 1;
                s.items.pop_front()
            } else {
                None
            };
            s.items.push_back(input);
            evicted
        };
        self.ready.notify_one();
        // Freed outside the lock.
        evicted.is_some()
    }

    /// Move every waiting input into `out`, waiting up to `timeout` for the
    /// first. Returns false once [`Queue::stop`] was called.
    pub fn take(&self, out: &mut VecDeque<Input>, timeout: Duration) -> bool {
        let mut s = self.state.lock();
        if s.items.is_empty() && !s.stop {
            self.ready.wait_for(&mut s, timeout);
        }
        out.append(&mut s.items);
        !s.stop
    }

    /// Inputs dropped so far.
    pub fn drops(&self) -> u64 {
        self.state.lock().drops
    }

    /// Wake the consumer for good.
    pub fn stop(&self) {
        self.state.lock().stop = true;
        self.ready.notify_all();
    }

    pub fn stopped(&self) -> bool {
        self.state.lock().stop
    }
}
