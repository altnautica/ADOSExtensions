//! Live per-drone config.
//!
//! The host hands `on_configure` the static manifest map; the operator's
//! per-drone values (the config form and the engage Skill's `active`
//! key) live in the host's config store, read through `ctx.config.get`.
//! [`read_live`] reads every key the plugin understands, [`merge`] lets a
//! live value override the static one, and [`spawn_config_watch`] polls
//! the store once a second. The host pushes no change notice, so on a
//! change the watcher exits the process with status 75 and the unit's
//! restart policy brings it back on the new config: one code path for
//! every key, and a disengage stops emission within about a second.

use std::collections::BTreeMap;
use std::time::Duration;

use ados_sdk::context::PluginContext;
use rmpv::Value;

/// Every per-drone key the plugin reads.
pub const CONFIG_KEYS: &[&str] = &[
    "mode",
    "active",
    "camera",
    "rangefinder",
    "firmware",
    "flow_quality_min",
    "outdoor",
];

/// Exit status that asks the unit to restart the plugin on new config.
pub const RESTART_EXIT_CODE: i32 = 75;

const POLL_INTERVAL: Duration = Duration::from_secs(1);

/// Read the live value of every [`CONFIG_KEYS`] entry, keeping only the
/// keys that hold a value. `None` when a read fails, so a caller never
/// mistakes an unreachable store for an empty one.
pub async fn read_live(ctx: &PluginContext) -> Option<BTreeMap<String, Value>> {
    let mut out = BTreeMap::new();
    for key in CONFIG_KEYS {
        match ctx.config.get(key, Value::Nil).await {
            Ok(Value::Nil) => {}
            Ok(v) => {
                out.insert((*key).to_string(), v);
            }
            Err(e) => {
                eprintln!("vision-nav: config read {key} failed: {e}");
                return None;
            }
        }
    }
    Some(out)
}

/// Overlay `live` on `static_`: a live value wins, a nil live value is
/// dropped so it never masks a static one.
pub fn merge(
    static_: &BTreeMap<String, Value>,
    live: &BTreeMap<String, Value>,
) -> BTreeMap<String, Value> {
    let mut out = static_.clone();
    for (k, v) in live {
        if !matches!(v, Value::Nil) {
            out.insert(k.clone(), v.clone());
        }
    }
    out
}

/// Whether the merged config the process started with differs from the
/// one the store holds now.
pub fn changed(started: &BTreeMap<String, Value>, now: &BTreeMap<String, Value>) -> bool {
    started != now
}

/// Poll the live store every second; when the merged config differs from
/// `started`, log and exit with [`RESTART_EXIT_CODE`]. A failed read skips
/// that poll.
pub fn spawn_config_watch(
    ctx: &PluginContext,
    static_: BTreeMap<String, Value>,
    started: BTreeMap<String, Value>,
) -> tokio::task::JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(POLL_INTERVAL);
        interval.tick().await;
        loop {
            interval.tick().await;
            let Some(live) = read_live(&ctx).await else {
                continue;
            };
            if changed(&started, &merge(&static_, &live)) {
                eprintln!("vision-nav: config changed, restarting");
                std::process::exit(RESTART_EXIT_CODE);
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    #[test]
    fn merge_prefers_live_over_static() {
        let s = m(&[
            ("mode", Value::from("off")),
            ("outdoor", Value::from(false)),
        ]);
        let l = m(&[("mode", Value::from("optical_flow"))]);
        let out = merge(&s, &l);
        assert_eq!(out["mode"].as_str(), Some("optical_flow"));
        assert_eq!(out["outdoor"].as_bool(), Some(false));
    }

    #[test]
    fn nil_live_values_are_dropped() {
        let s = m(&[("mode", Value::from("optical_flow"))]);
        let l = m(&[("mode", Value::Nil)]);
        assert_eq!(merge(&s, &l)["mode"].as_str(), Some("optical_flow"));
    }

    #[test]
    fn changed_detects_an_active_flip() {
        let started = m(&[("active", Value::from(true))]);
        let same = m(&[("active", Value::from(true))]);
        let flipped = m(&[("active", Value::from(false))]);
        assert!(!changed(&started, &same));
        assert!(changed(&started, &flipped));
    }
}
