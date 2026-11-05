//! The `nav.state` event: what the navigation feed is doing right now.
//!
//! Payload (snake_case, the shared wire shape the agent host's nav
//! stream and every operator surface read):
//!
//! ```json
//! {"mode": "external_nav", "active_set": 1, "extnav_health": "good",
//!  "odom_rate_hz": 30.0, "last_reset_counter": 0, "obstacle_rate_hz": 10.0,
//!  "min_obstacle_m": 3.2, "gps_fix": 0}
//! ```
//!
//! Fields a mode does not produce are `null`.

use rmpv::Value;

use crate::external_nav::ExtnavHealth;

/// The event topic.
pub const NAV_STATE_TOPIC: &str = "nav.state";

#[derive(Debug, Clone, Default, PartialEq)]
pub struct NavState {
    pub mode: String,
    pub active_set: Option<u8>,
    pub extnav_health: Option<ExtnavHealth>,
    pub odom_rate_hz: Option<f32>,
    pub last_reset_counter: Option<u8>,
    pub obstacle_rate_hz: Option<f32>,
    pub min_obstacle_m: Option<f32>,
    pub gps_fix: u8,
}

impl NavState {
    pub fn to_value(&self) -> Value {
        let f = |v: Option<f32>| v.map_or(Value::Nil, |x| Value::F64(f64::from(x)));
        let u = |v: Option<u8>| v.map_or(Value::Nil, Value::from);
        Value::Map(vec![
            (Value::from("mode"), Value::from(self.mode.as_str())),
            (Value::from("active_set"), u(self.active_set)),
            (
                Value::from("extnav_health"),
                self.extnav_health
                    .map_or(Value::Nil, |h| Value::from(h.as_str())),
            ),
            (Value::from("odom_rate_hz"), f(self.odom_rate_hz)),
            (
                Value::from("last_reset_counter"),
                u(self.last_reset_counter),
            ),
            (Value::from("obstacle_rate_hz"), f(self.obstacle_rate_hz)),
            (Value::from("min_obstacle_m"), f(self.min_obstacle_m)),
            (Value::from("gps_fix"), Value::from(self.gps_fix)),
        ])
    }
}

/// Send rate over a sliding window of event times.
pub struct RateMeter {
    window_ns: i64,
    times: std::collections::VecDeque<i64>,
}

impl RateMeter {
    pub fn new(window_ns: i64) -> Self {
        Self {
            window_ns,
            times: std::collections::VecDeque::new(),
        }
    }

    pub fn mark(&mut self, now_ns: i64) {
        self.times.push_back(now_ns);
        self.trim(now_ns);
    }

    /// Events per second over the window ending at `now_ns`.
    pub fn rate_hz(&mut self, now_ns: i64) -> f32 {
        self.trim(now_ns);
        self.times.len() as f32 * 1e9 / self.window_ns as f32
    }

    fn trim(&mut self, now_ns: i64) {
        while self
            .times
            .front()
            .is_some_and(|&t| now_ns - t >= self.window_ns)
        {
            self.times.pop_front();
        }
    }
}

/// How many recent pose ages [`AgeWindow`] keeps.
pub const AGE_WINDOW: usize = 50;

/// The median of the last [`AGE_WINDOW`] pose ages (ms): how old a pose
/// is when its `ODOMETRY` goes out.
#[derive(Default)]
pub struct AgeWindow {
    ages: std::collections::VecDeque<f32>,
}

impl AgeWindow {
    pub fn record(&mut self, age_ms: f32) {
        if self.ages.len() == AGE_WINDOW {
            self.ages.pop_front();
        }
        self.ages.push_back(age_ms);
    }

    /// `None` before the first age.
    pub fn median_ms(&self) -> Option<f32> {
        if self.ages.is_empty() {
            return None;
        }
        let mut v: Vec<f32> = self.ages.iter().copied().collect();
        v.sort_by(f32::total_cmp);
        let n = v.len();
        Some(if n % 2 == 1 {
            v[n / 2]
        } else {
            (v[n / 2 - 1] + v[n / 2]) / 2.0
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field<'a>(v: &'a Value, key: &str) -> &'a Value {
        match v {
            Value::Map(e) => e
                .iter()
                .find(|(k, _)| k.as_str() == Some(key))
                .map(|(_, val)| val)
                .unwrap_or_else(|| panic!("missing {key}")),
            _ => panic!("not a map"),
        }
    }

    #[test]
    fn nav_state_matches_the_wire_shape() {
        let s = NavState {
            mode: "external_nav".into(),
            active_set: Some(1),
            extnav_health: Some(ExtnavHealth::Good),
            odom_rate_hz: Some(30.0),
            last_reset_counter: Some(2),
            obstacle_rate_hz: Some(10.0),
            min_obstacle_m: Some(3.25),
            gps_fix: 0,
        };
        let v = s.to_value();
        assert_eq!(field(&v, "mode").as_str(), Some("external_nav"));
        assert_eq!(field(&v, "active_set").as_u64(), Some(1));
        assert_eq!(field(&v, "extnav_health").as_str(), Some("good"));
        assert_eq!(field(&v, "odom_rate_hz").as_f64(), Some(30.0));
        assert_eq!(field(&v, "last_reset_counter").as_u64(), Some(2));
        assert_eq!(field(&v, "min_obstacle_m").as_f64(), Some(3.25));
        assert_eq!(field(&v, "gps_fix").as_u64(), Some(0));
        let Value::Map(e) = &v else { unreachable!() };
        assert_eq!(e.len(), 8);
    }

    #[test]
    fn absent_fields_are_null() {
        let v = NavState {
            mode: "optical_flow".into(),
            ..NavState::default()
        }
        .to_value();
        assert!(field(&v, "active_set").is_nil());
        assert!(field(&v, "extnav_health").is_nil());
        assert!(field(&v, "min_obstacle_m").is_nil());
    }

    #[test]
    fn rate_meter_counts_the_window() {
        let mut m = RateMeter::new(1_000_000_000);
        for i in 0..30 {
            m.mark(i * 33_333_333);
        }
        let r = m.rate_hz(1_000_000_000);
        assert!((29.0..=31.0).contains(&r), "{r}");
        assert_eq!(m.rate_hz(5_000_000_000), 0.0);
    }

    #[test]
    fn age_window_is_the_median_of_the_last_fifty() {
        let mut w = AgeWindow::default();
        assert_eq!(w.median_ms(), None);
        // 60 old outliers, then 50 ages around 120 ms with a few spikes.
        for _ in 0..60 {
            w.record(900.0);
        }
        for i in 0..50 {
            w.record(if i % 10 == 0 { 400.0 } else { 120.0 });
        }
        assert_eq!(w.median_ms(), Some(120.0));
        let mut two = AgeWindow::default();
        two.record(100.0);
        two.record(140.0);
        assert_eq!(two.median_ms(), Some(120.0));
    }
}
