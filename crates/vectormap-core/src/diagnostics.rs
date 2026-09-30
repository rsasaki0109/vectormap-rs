//! Structured, machine-readable diagnostics.
//!
//! [`Issue`] is the single diagnostic type shared by editing operations
//! (warnings in a [`ChangeSet`](crate::ChangeSet)), importers (load reports)
//! and the validator. Codes are stable snake_case strings so that tools and
//! language models can match on them.

use std::borrow::Cow;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::id::EntityRef;
use crate::ops::Command;

/// How serious an issue is. Ordered from least to most severe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    /// Informational note.
    Info,
    /// Suspicious but usable.
    Warning,
    /// The map is inconsistent or unusable in this respect.
    Error,
}

impl fmt::Display for Severity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Severity::Info => "info",
            Severity::Warning => "warning",
            Severity::Error => "error",
        })
    }
}

/// Stable machine-readable issue code, e.g. `dangling_lane_reference`.
///
/// Codes are namespaced by convention: core and validation codes are plain
/// (`asymmetric_link`), format-specific codes carry a prefix
/// (`lanelet2.unsupported_relation`, `autoware.missing_speed_limit`).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct IssueCode(pub Cow<'static, str>);

impl IssueCode {
    /// Creates a code from a static string.
    pub const fn new(code: &'static str) -> Self {
        Self(Cow::Borrowed(code))
    }

    /// The code as a string slice.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for IssueCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl PartialEq<&str> for IssueCode {
    fn eq(&self, other: &&str) -> bool {
        self.0 == *other
    }
}

/// Issue codes emitted by editing operations in this crate.
pub mod codes {
    use super::IssueCode;

    /// The same ID is defined more than once.
    pub const DUPLICATE_ID: IssueCode = IssueCode::new("duplicate_id");
    /// A requested change was already in effect; nothing was done.
    pub const NO_OP: IssueCode = IssueCode::new("no_op");
    /// Connected lanes do not meet geometrically.
    pub const GEOMETRIC_GAP: IssueCode = IssueCode::new("geometric_gap");
    /// A boundary shared with other lanes had to be duplicated.
    pub const BOUNDARY_UNSHARED: IssueCode = IssueCode::new("boundary_unshared");
    /// A neighbor relation could not be preserved.
    pub const NEIGHBOR_DROPPED: IssueCode = IssueCode::new("neighbor_dropped");
    /// A rule no longer applies to any lane.
    pub const ORPHAN_RULE: IssueCode = IssueCode::new("orphan_rule");
    /// Geometry was synthesized from defaults and should be reviewed.
    pub const GEOMETRY_SYNTHESIZED: IssueCode = IssueCode::new("geometry_synthesized");
    /// A rule was replaced by another rule.
    pub const RULE_REPLACED: IssueCode = IssueCode::new("rule_replaced");
    /// Two merged lanes had different attributes; the first one's were kept.
    pub const ATTRIBUTE_CONFLICT: IssueCode = IssueCode::new("attribute_conflict");
    /// Topology inference found several candidates and picked one.
    pub const AMBIGUOUS_TOPOLOGY: IssueCode = IssueCode::new("ambiguous_topology");
    /// A placement was adjusted to stay inside the valid range.
    pub const PLACEMENT_CLAMPED: IssueCode = IssueCode::new("placement_clamped");
}

/// A single structured diagnostic.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Issue {
    /// Severity.
    pub severity: Severity,
    /// Stable machine-readable code.
    pub code: IssueCode,
    /// Human readable explanation.
    pub message: String,
    /// Primary entity concerned, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entity: Option<EntityRef>,
    /// Other entities involved.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub related: Vec<EntityRef>,
    /// A command that would resolve the issue, if one is known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fix: Option<Box<Command>>,
}

impl Issue {
    /// Creates an issue.
    pub fn new(severity: Severity, code: IssueCode, message: impl Into<String>) -> Self {
        Self {
            severity,
            code,
            message: message.into(),
            entity: None,
            related: Vec::new(),
            fix: None,
        }
    }

    /// Shorthand for an error.
    pub fn error(code: IssueCode, message: impl Into<String>) -> Self {
        Self::new(Severity::Error, code, message)
    }

    /// Shorthand for a warning.
    pub fn warning(code: IssueCode, message: impl Into<String>) -> Self {
        Self::new(Severity::Warning, code, message)
    }

    /// Shorthand for an informational note.
    pub fn info(code: IssueCode, message: impl Into<String>) -> Self {
        Self::new(Severity::Info, code, message)
    }

    /// Sets the primary entity.
    pub fn with_entity(mut self, entity: impl Into<EntityRef>) -> Self {
        self.entity = Some(entity.into());
        self
    }

    /// Adds a related entity.
    pub fn with_related(mut self, entity: impl Into<EntityRef>) -> Self {
        self.related.push(entity.into());
        self
    }

    /// Attaches a suggested fix.
    pub fn with_fix(mut self, fix: Command) -> Self {
        self.fix = Some(Box::new(fix));
        self
    }
}

impl fmt::Display for Issue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} [{}]", self.severity, self.code)?;
        if let Some(e) = self.entity {
            write!(f, " {e}")?;
        }
        write!(f, ": {}", self.message)
    }
}
