//! # vectormap-io
//!
//! Readers and writers for [`vectormap_core::Map`]:
//!
//! - [`json`]: the lossless JSON form of the IR (`vectormap-ir`);
//! - [`lanelet2`]: the Lanelet2 OSM XML adapter (`Lanelet2 ⇄ adapter ⇄ IR`);
//! - [`autoware`]: the Autoware conventions on top of Lanelet2 (export
//!   profile, compatibility checks, `map_projector_info.yaml`);
//! - [`projection`]: WGS84 ⇄ local metric coordinates.
//!
//! Format-specific knowledge lives only here; `vectormap-core` stays
//! format-independent.

pub mod autoware;
pub mod json;
pub mod lanelet2;
pub mod projection;

use std::fmt;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use vectormap_core::{DocumentError, Issue, Map};

pub use lanelet2::{load_lanelet2, save_lanelet2};

/// Errors of reading or writing maps.
#[derive(Debug, thiserror::Error)]
pub enum IoError {
    /// File system error.
    #[error("{}", path.display())]
    File {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        source: std::io::Error,
    },
    /// Invalid JSON.
    #[error("invalid JSON: {0}")]
    Json(#[from] serde_json::Error),
    /// Invalid XML.
    #[error("invalid XML: {0}")]
    Xml(#[from] roxmltree::Error),
    /// Structurally valid document with unsupported content.
    #[error(transparent)]
    Document(#[from] DocumentError),
    /// The file does not follow the expected format.
    #[error("format error: {0}")]
    Format(String),
    /// The format could not be determined.
    #[error("cannot determine the map format of {0:?}; specify it explicitly")]
    UnknownFormat(PathBuf),
}

/// A loaded map plus the issues found while loading it (duplicate IDs,
/// unsupported primitives, ...).
#[derive(Debug, Clone, PartialEq)]
pub struct Loaded {
    /// The map.
    pub map: Map,
    /// Load-time issues.
    pub issues: Vec<Issue>,
}

/// Supported map formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    /// vectormap IR as JSON.
    Json,
    /// Lanelet2 OSM XML.
    Lanelet2,
}

impl Format {
    /// All formats.
    pub const ALL: [Format; 2] = [Format::Json, Format::Lanelet2];

    /// Canonical name (`json`, `lanelet2`).
    pub const fn name(self) -> &'static str {
        match self {
            Format::Json => "json",
            Format::Lanelet2 => "lanelet2",
        }
    }

    /// Guesses the format from a file extension (`.json`, `.osm`).
    pub fn from_path(path: &Path) -> Option<Format> {
        match path.extension()?.to_str()?.to_ascii_lowercase().as_str() {
            "json" => Some(Format::Json),
            "osm" | "xml" => Some(Format::Lanelet2),
            _ => None,
        }
    }
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for Format {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_ascii_lowercase().as_str() {
            "json" | "ir" => Ok(Format::Json),
            "lanelet2" | "osm" => Ok(Format::Lanelet2),
            other => Err(format!(
                "unknown format {other:?} (expected json or lanelet2)"
            )),
        }
    }
}

pub(crate) fn read_file(path: &Path) -> Result<String, IoError> {
    std::fs::read_to_string(path).map_err(|source| IoError::File {
        path: path.to_path_buf(),
        source,
    })
}

pub(crate) fn write_file(path: &Path, contents: &str) -> Result<(), IoError> {
    std::fs::write(path, contents).map_err(|source| IoError::File {
        path: path.to_path_buf(),
        source,
    })
}

fn resolve(path: &Path, format: Option<Format>) -> Result<Format, IoError> {
    format
        .or_else(|| Format::from_path(path))
        .ok_or_else(|| IoError::UnknownFormat(path.to_path_buf()))
}

/// Loads a map with default options, detecting the format from the file
/// extension when `format` is `None`.
pub fn load(path: impl AsRef<Path>, format: Option<Format>) -> Result<Loaded, IoError> {
    let path = path.as_ref();
    match resolve(path, format)? {
        Format::Json => json::load(path),
        Format::Lanelet2 => lanelet2::load_lanelet2(path, &lanelet2::LoadOptions::default()),
    }
}

/// Saves a map with default options, detecting the format from the file
/// extension when `format` is `None`. Returns export warnings.
pub fn save(
    map: &Map,
    path: impl AsRef<Path>,
    format: Option<Format>,
) -> Result<Vec<Issue>, IoError> {
    let path = path.as_ref();
    match resolve(path, format)? {
        Format::Json => json::save(map, path).map(|()| Vec::new()),
        Format::Lanelet2 => lanelet2::save_lanelet2(map, path, &lanelet2::SaveOptions::default()),
    }
}
