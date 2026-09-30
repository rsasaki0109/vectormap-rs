//! Lanelet2 OSM XML adapter.
//!
//! ```text
//! Lanelet2 (OSM XML) ──reader──▶ vectormap IR ──writer──▶ Lanelet2 (OSM XML)
//! ```
//!
//! Lanelet2 is *not* the internal model: this module maps Lanelet2
//! primitives to IR entities and back. See `docs/lanelet2.md` for the full
//! mapping table. Highlights:
//!
//! - lanelets become [`Lane`](vectormap_core::Lane)s (or
//!   [`Crosswalk`](vectormap_core::Crosswalk)s for `subtype=crosswalk`);
//! - bound orientation is re-derived exactly like Lanelet2's
//!   `geometry::align`, and topology is inferred from shared geometry;
//! - tags without a typed IR counterpart are kept as `lanelet2:*`
//!   attributes and written back on export;
//! - primitives the IR cannot represent are reported as structured issues
//!   instead of being dropped silently.

pub(crate) mod osm;
mod reader;
mod tags;
mod writer;

use std::path::Path;

use vectormap_core::{GeoReference, InferOptions, Issue, Map};

pub use osm::{
    MemberType, OsmData, OsmMember, OsmNode, OsmRelation, OsmWay, parse as parse_osm,
    write as write_osm,
};
pub use reader::read_str;
pub use writer::write_string;

use crate::{IoError, Loaded, read_file, write_file};

/// Attribute prefix for Lanelet2 tags kept in the IR.
pub const ATTRIBUTE_PREFIX: &str = "lanelet2";

/// Attribute namespaces that belong to specific formats. Keys in these
/// namespaces are never written as plain Lanelet2 tags.
pub const FORMAT_PREFIXES: &[&str] = &["lanelet2", "opendrive", "geojson", "vectormap"];

/// Issue codes emitted by the Lanelet2 adapter.
pub mod codes {
    use vectormap_core::IssueCode;

    pub use vectormap_core::diagnostics::codes::DUPLICATE_ID;

    /// A way or relation references a node that does not exist.
    pub const MISSING_NODE: IssueCode = IssueCode::new("lanelet2.missing_node");
    /// A relation references a member that does not exist or has the wrong type.
    pub const MISSING_MEMBER: IssueCode = IssueCode::new("lanelet2.missing_member");
    /// A lanelet or regulatory element is structurally invalid and was skipped.
    pub const INVALID_PRIMITIVE: IssueCode = IssueCode::new("lanelet2.invalid_primitive");
    /// A tag value could not be interpreted; it was kept as an attribute.
    pub const INVALID_TAG: IssueCode = IssueCode::new("lanelet2.invalid_tag");
    /// Ways the IR does not model were skipped.
    pub const UNSUPPORTED_WAY: IssueCode = IssueCode::new("lanelet2.unsupported_way");
    /// Relations the IR does not model were skipped.
    pub const UNSUPPORTED_RELATION: IssueCode = IssueCode::new("lanelet2.unsupported_relation");
    /// Members of a relation could not be represented.
    pub const UNSUPPORTED_MEMBER: IssueCode = IssueCode::new("lanelet2.unsupported_member");
    /// How coordinates were projected.
    pub const PROJECTION: IssueCode = IssueCode::new("lanelet2.projection");
    /// The map has no georeference; placeholder coordinates were written.
    pub const NO_GEOREFERENCE: IssueCode = IssueCode::new("lanelet2.no_georeference");
    /// An entity could not keep its ID in the OSM file.
    pub const RENUMBERED: IssueCode = IssueCode::new("lanelet2.renumbered");
    /// Geometry required by Lanelet2 was synthesized.
    pub const GEOMETRY_SYNTHESIZED: IssueCode = IssueCode::new("lanelet2.geometry_synthesized");
    /// IR entities without a Lanelet2 counterpart (roads) were not written.
    pub const NOT_EXPORTED: IssueCode = IssueCode::new("lanelet2.not_exported");
    /// An entity references something that does not exist and was skipped.
    pub const DANGLING_REFERENCE: IssueCode = IssueCode::new("lanelet2.dangling_reference");
}

/// How node coordinates are turned into local metric coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum ProjectionChoice {
    /// Decide from the data:
    ///
    /// 1. every node has `local_x` / `local_y` and lat/lon are placeholders
    ///    (Autoware `Local` maps): use the local tags, no georeference;
    /// 2. every node has local tags that are consistent with lat/lon under a
    ///    UTM or transverse Mercator projection (files written by
    ///    vectormap-rs, Autoware MGRS maps): use the local tags and the
    ///    recovered georeference;
    /// 3. otherwise: project lat/lon with UTM relative to the south-west
    ///    corner of the data.
    #[default]
    Auto,
    /// Use the `local_x` / `local_y` node tags.
    LocalTags,
    /// Project lat/lon with the given georeference.
    Georeferenced(GeoReference),
}

/// Options of [`load_lanelet2`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LoadOptions {
    /// Coordinate handling.
    pub projection: ProjectionChoice,
    /// Topology inference parameters.
    pub infer: InferOptions,
}

/// Export profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Profile {
    /// Plain Lanelet2 conventions (plus Autoware extension roles, which
    /// Lanelet2 treats as generic regulatory elements).
    #[default]
    Generic,
    /// Autoware conventions: required tag defaults, `local_x` / `local_y`.
    /// See [`crate::autoware`].
    Autoware,
}

/// Options of [`save_lanelet2`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SaveOptions {
    /// Export profile.
    pub profile: Profile,
    /// Georeference used for lat/lon; defaults to the map's.
    pub georeference: Option<GeoReference>,
    /// Write `local_x` / `local_y` node tags (default: `true`). They make the
    /// local frame survive a round trip exactly; Lanelet2 treats them as
    /// plain attributes. Always written when the map has no georeference.
    pub local_coordinates: Option<bool>,
}

impl SaveOptions {
    /// Options for the Autoware profile.
    pub fn autoware() -> Self {
        Self {
            profile: Profile::Autoware,
            ..Self::default()
        }
    }
}

/// Loads a Lanelet2 OSM file.
pub fn load_lanelet2(path: impl AsRef<Path>, options: &LoadOptions) -> Result<Loaded, IoError> {
    read_str(&read_file(path.as_ref())?, options)
}

/// Saves a map as a Lanelet2 OSM file. Returns export warnings.
pub fn save_lanelet2(
    map: &Map,
    path: impl AsRef<Path>,
    options: &SaveOptions,
) -> Result<Vec<Issue>, IoError> {
    let (text, issues) = write_string(map, options);
    write_file(path.as_ref(), &text)?;
    Ok(issues)
}
