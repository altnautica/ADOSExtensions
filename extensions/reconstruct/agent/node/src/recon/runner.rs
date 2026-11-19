//! How a pipeline stage runs an external tool.
//!
//! Every third-party program the node drives (COLMAP, Brush, the gsplat
//! training project, the splat converters, the calibration containers) goes
//! through a [`ToolRunner`]: the stage builds a [`ToolInvocation`], the runner
//! resolves the tool from the pinned set ([`crate::tools`]) and runs it,
//! streaming each stdout/stderr line back. The real runner spawns processes; a
//! stand-in runner (feature `mock` or tests) writes deterministic outputs so
//! the whole pipeline runs with no tool installed.

use std::ffi::OsString;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

/// A stage failure with a stable code the API surfaces as
/// `error: {code, message, tool?}`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
#[error("{code}: {message}")]
pub struct StageError {
    pub code: String,
    pub message: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool: Option<String>,
}

impl StageError {
    pub fn new(code: &str, message: impl Into<String>) -> Self {
        Self {
            code: code.to_string(),
            message: message.into(),
            tool: None,
        }
    }

    /// A pinned tool is not installed on this node.
    pub fn tool_missing(tool: &str, detail: impl Into<String>) -> Self {
        Self {
            code: "E_TOOL_MISSING".into(),
            message: detail.into(),
            tool: Some(tool.to_string()),
        }
    }

    /// A tool ran and failed.
    pub fn tool_failed(tool: &str, detail: impl Into<String>) -> Self {
        Self {
            code: "E_TOOL_FAILED".into(),
            message: detail.into(),
            tool: Some(tool.to_string()),
        }
    }

    /// The input session or job request is unusable.
    pub fn input(detail: impl Into<String>) -> Self {
        Self::new("E_INPUT", detail)
    }

    /// A local filesystem or encoding fault.
    pub fn io(detail: impl std::fmt::Display) -> Self {
        Self::new("E_IO", detail.to_string())
    }
}

impl From<std::io::Error> for StageError {
    fn from(e: std::io::Error) -> Self {
        Self::io(e)
    }
}

impl From<serde_json::Error> for StageError {
    fn from(e: serde_json::Error) -> Self {
        Self::io(e)
    }
}

impl From<ados_capture_contract::Error> for StageError {
    fn from(e: ados_capture_contract::Error) -> Self {
        Self::input(e.to_string())
    }
}

/// A resolved tool: the program to spawn plus any fixed leading arguments
/// (a uv project runs as `uv run --project <dir> python <script>`, a container
/// as `docker run ... <image>`).
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedTool {
    pub name: String,
    pub version: String,
    pub program: PathBuf,
    pub prefix_args: Vec<OsString>,
    /// The tool's install or project directory (a uv project's scripts live
    /// here).
    pub home: PathBuf,
    /// `program` is a Linux program run inside the WSL distribution (Windows
    /// hosts): it is spawned through `wsl.exe --exec` and every absolute
    /// Windows path in its arguments is mapped to its `/mnt/<drive>` form.
    pub wsl: bool,
}

/// One tool run, built by a stage.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolInvocation {
    /// Tool name from the pinned set (`colmap`, `brush`, `gsplat_train`, ...).
    pub tool: String,
    /// Arguments after the resolved program and its prefix arguments.
    pub args: Vec<OsString>,
    /// Working directory.
    pub cwd: PathBuf,
    /// Extra environment.
    pub env: Vec<(String, String)>,
}

impl ToolInvocation {
    pub fn new(tool: &str, cwd: impl Into<PathBuf>) -> Self {
        Self {
            tool: tool.to_string(),
            args: Vec::new(),
            cwd: cwd.into(),
            env: Vec::new(),
        }
    }

    /// Append one argument.
    pub fn arg(mut self, a: impl Into<OsString>) -> Self {
        self.args.push(a.into());
        self
    }

    /// Append several arguments.
    pub fn args<I, S>(mut self, it: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<OsString>,
    {
        self.args.extend(it.into_iter().map(Into::into));
        self
    }

    /// The arguments as UTF-8 strings (lossy), for logs and tests.
    pub fn args_lossy(&self) -> Vec<String> {
        self.args
            .iter()
            .map(|a| a.to_string_lossy().into_owned())
            .collect()
    }
}

/// Runs tools for the pipeline stages.
pub trait ToolRunner: Send + Sync {
    /// Resolve a pinned tool, or `E_TOOL_MISSING`.
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError>;

    /// Run `inv` to completion, handing every output line to `on_line`.
    /// A non-zero exit is `E_TOOL_FAILED` carrying the output tail.
    fn run(&self, inv: &ToolInvocation, on_line: &mut dyn FnMut(&str)) -> Result<(), StageError>;
}

/// A runner that reports every tool it runs (name and resolved version,
/// `override` for an `ADOS_TOOL_<NAME>` executable) to `on_run` before
/// handing the run to `inner`, so a job records the tools it actually used.
pub struct RecordingRunner<'a> {
    pub inner: &'a dyn ToolRunner,
    pub on_run: &'a (dyn Fn(&ResolvedTool) + Send + Sync),
}

impl ToolRunner for RecordingRunner<'_> {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        self.inner.resolve(tool)
    }

    fn run(&self, inv: &ToolInvocation, on_line: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        (self.on_run)(&self.inner.resolve(&inv.tool)?);
        self.inner.run(inv, on_line)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use super::*;

    /// Resolves every tool except `missing`; `brush` as an override.
    struct Fake;

    impl ToolRunner for Fake {
        fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
            if tool == "missing" {
                return Err(StageError::tool_missing(tool, "absent"));
            }
            Ok(ResolvedTool {
                name: tool.to_string(),
                version: if tool == "brush" { "override" } else { "1.2.3" }.into(),
                program: PathBuf::from(tool),
                prefix_args: Vec::new(),
                home: PathBuf::new(),
                wsl: false,
            })
        }

        fn run(&self, inv: &ToolInvocation, _: &mut dyn FnMut(&str)) -> Result<(), StageError> {
            self.resolve(&inv.tool).map(|_| ())
        }
    }

    #[test]
    fn records_only_tools_that_run() {
        let seen = Mutex::new(Vec::new());
        let record = |t: &ResolvedTool| {
            seen.lock()
                .unwrap()
                .push((t.name.clone(), t.version.clone()))
        };
        let runner = RecordingRunner {
            inner: &Fake,
            on_run: &record,
        };
        runner.resolve("colmap").unwrap();
        runner
            .run(&ToolInvocation::new("glim", "."), &mut |_| {})
            .unwrap();
        runner
            .run(&ToolInvocation::new("brush", "."), &mut |_| {})
            .unwrap();
        assert!(runner
            .run(&ToolInvocation::new("missing", "."), &mut |_| {})
            .is_err());
        assert_eq!(
            *seen.lock().unwrap(),
            [
                ("glim".to_string(), "1.2.3".to_string()),
                ("brush".to_string(), "override".to_string())
            ]
        );
    }
}
