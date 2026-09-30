//! The map being edited in an MCP session, with undo history.

use std::path::PathBuf;

use serde_json::{Value, json};
use vectormap_core::{BatchError, ChangeSet, Command, Map};
use vectormap_io::Format;

/// Maximum number of undo steps kept.
pub const MAX_UNDO: usize = 50;

/// A failed tool call, reported to the client as a tool result with
/// `isError: true` and a machine-readable `code`.
#[derive(Debug, Clone, PartialEq)]
pub struct ToolError {
    /// Stable code, e.g. `no_map`, `invalid_arguments`, `not_found`.
    pub code: String,
    /// Human readable explanation.
    pub message: String,
    /// Structured details (e.g. the serialized `EditError`).
    pub details: Option<Value>,
}

impl ToolError {
    /// Creates an error without details.
    pub fn new(code: &str, message: impl Into<String>) -> Self {
        Self {
            code: code.to_string(),
            message: message.into(),
            details: None,
        }
    }

    /// The error as JSON.
    pub fn to_json(&self) -> Value {
        let mut v = json!({"error": {"code": self.code, "message": self.message}});
        if let Some(d) = &self.details {
            v["error"]["details"] = d.clone();
        }
        v
    }
}

impl From<vectormap_io::IoError> for ToolError {
    fn from(e: vectormap_io::IoError) -> Self {
        ToolError::new("io_error", e.to_string())
    }
}

impl From<BatchError> for ToolError {
    fn from(e: BatchError) -> Self {
        let details = serde_json::to_value(&e).ok();
        let code = details
            .as_ref()
            .and_then(|d| d["error"]["code"].as_str())
            .unwrap_or("edit_error")
            .to_string();
        ToolError {
            code,
            message: e.to_string(),
            details,
        }
    }
}

/// State of one MCP session.
#[derive(Debug, Default)]
pub struct Session {
    map: Option<Map>,
    source: Option<(PathBuf, Format)>,
    history: Vec<Map>,
    dirty: bool,
}

impl Session {
    /// A session without an open map.
    pub fn new() -> Self {
        Self::default()
    }

    /// Replaces the open map (clears the undo history).
    pub fn open(&mut self, map: Map, source: Option<(PathBuf, Format)>) {
        self.map = Some(map);
        self.source = source;
        self.history.clear();
        self.dirty = false;
    }

    /// The open map.
    pub fn map(&self) -> Result<&Map, ToolError> {
        self.map.as_ref().ok_or_else(|| {
            ToolError::new("no_map", "no map is open; call open_map or new_map first")
        })
    }

    /// Where the map was loaded from / last saved to.
    pub fn source(&self) -> Option<&(PathBuf, Format)> {
        self.source.as_ref()
    }

    /// Records a new save location and clears the dirty flag.
    pub fn saved(&mut self, source: (PathBuf, Format)) {
        self.source = Some(source);
        self.dirty = false;
    }

    /// `true` if there are unsaved edits.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Number of available undo steps.
    pub fn undo_depth(&self) -> usize {
        self.history.len()
    }

    /// Applies commands atomically and records an undo step.
    pub fn apply(&mut self, commands: &[Command]) -> Result<Vec<ChangeSet>, ToolError> {
        let map = self.map.as_mut().ok_or_else(|| {
            ToolError::new("no_map", "no map is open; call open_map or new_map first")
        })?;
        let before = map.clone();
        let changes = map.apply_all(commands)?;
        if changes.iter().any(|c| !c.is_empty()) {
            self.history.push(before);
            if self.history.len() > MAX_UNDO {
                self.history.remove(0);
            }
            self.dirty = true;
        }
        Ok(changes)
    }

    /// Reverts the last edit.
    pub fn undo(&mut self) -> Result<(), ToolError> {
        let previous = self
            .history
            .pop()
            .ok_or_else(|| ToolError::new("nothing_to_undo", "there is no edit to undo"))?;
        self.map = Some(previous);
        self.dirty = true;
        Ok(())
    }
}
