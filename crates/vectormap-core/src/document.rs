//! The serialized form of the IR (`vectormap-ir`, version 1).
//!
//! A [`MapDocument`] stores every entity kind as an array sorted by ID, and
//! the topology as an array of per-lane entries. Arrays (rather than JSON
//! objects keyed by ID) make duplicate IDs in hand-written files detectable
//! and keep the format easy to read for humans and language models.

use std::collections::BTreeSet;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::diagnostics::{Issue, codes};
use crate::entities::{
    Boundary, Crosswalk, Junction, Lane, RegulatoryElement, Road, StopLine, TrafficSignal,
};
use crate::id::{EntityRef, LaneId};
use crate::map::{Map, MapError, MapMetadata};
use crate::topology::LaneLinks;

/// Value of the `format` field.
pub const FORMAT_NAME: &str = "vectormap-ir";
/// Current value of the `version` field.
pub const FORMAT_VERSION: u32 = 1;

fn default_format() -> String {
    FORMAT_NAME.to_string()
}

fn default_version() -> u32 {
    FORMAT_VERSION
}

/// Topology of one lane in a [`MapDocument`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopologyEntry {
    /// The lane.
    pub lane: LaneId,
    /// Its links.
    #[serde(flatten)]
    pub links: LaneLinks,
}

/// Serializable representation of a [`Map`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapDocument {
    /// Always `"vectormap-ir"`.
    #[serde(default = "default_format")]
    pub format: String,
    /// Format version.
    #[serde(default = "default_version")]
    pub version: u32,
    /// Map metadata.
    #[serde(default)]
    pub metadata: MapMetadata,
    /// Lanes.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lanes: Vec<Lane>,
    /// Boundaries.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub boundaries: Vec<Boundary>,
    /// Roads.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub roads: Vec<Road>,
    /// Junctions.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub junctions: Vec<Junction>,
    /// Stop lines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stop_lines: Vec<StopLine>,
    /// Traffic signals.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub traffic_signals: Vec<TrafficSignal>,
    /// Crosswalks.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub crosswalks: Vec<Crosswalk>,
    /// Regulatory elements.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub regulatory_elements: Vec<RegulatoryElement>,
    /// Lane topology.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub topology: Vec<TopologyEntry>,
    /// Next ID the allocator hands out, when it is larger than
    /// `max(id) + 1` (because entities were deleted). Keeps ID allocation
    /// identical after a save / load cycle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub next_id: Option<u64>,
}

impl Default for MapDocument {
    fn default() -> Self {
        Self {
            format: default_format(),
            version: default_version(),
            metadata: MapMetadata::default(),
            lanes: Vec::new(),
            boundaries: Vec::new(),
            roads: Vec::new(),
            junctions: Vec::new(),
            stop_lines: Vec::new(),
            traffic_signals: Vec::new(),
            crosswalks: Vec::new(),
            regulatory_elements: Vec::new(),
            topology: Vec::new(),
            next_id: None,
        }
    }
}

/// Errors when converting a document into a map.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DocumentError {
    /// The `format` field is not `vectormap-ir`.
    #[error("unsupported format {0:?} (expected \"vectormap-ir\")")]
    UnsupportedFormat(String),
    /// The document was written by a newer version.
    #[error("unsupported vectormap-ir version {0} (this build supports up to {FORMAT_VERSION})")]
    UnsupportedVersion(u32),
    /// Strict conversion found a duplicate ID.
    #[error(transparent)]
    Map(#[from] MapError),
}

impl Map {
    /// Converts the map into its serializable document form.
    pub fn to_document(&self) -> MapDocument {
        fn values<K: Copy, V: Clone>(
            m: &std::collections::BTreeMap<K, V>,
            fix: impl Fn(K, &mut V),
        ) -> Vec<V> {
            m.iter()
                .map(|(k, v)| {
                    let mut v = v.clone();
                    fix(*k, &mut v);
                    v
                })
                .collect()
        }
        let max_id = self
            .entity_refs()
            .iter()
            .map(|e| e.raw())
            .max()
            .unwrap_or(0);
        let next_id = (self.next_id > max_id + 1).then_some(self.next_id);
        MapDocument {
            next_id,
            format: FORMAT_NAME.to_string(),
            version: FORMAT_VERSION,
            metadata: self.metadata.clone(),
            lanes: values(&self.lanes, |k, v| v.id = k),
            boundaries: values(&self.boundaries, |k, v| v.id = k),
            roads: values(&self.roads, |k, v| v.id = k),
            junctions: values(&self.junctions, |k, v| v.id = k),
            stop_lines: values(&self.stop_lines, |k, v| v.id = k),
            traffic_signals: values(&self.traffic_signals, |k, v| v.id = k),
            crosswalks: values(&self.crosswalks, |k, v| v.id = k),
            regulatory_elements: values(&self.regulatory_elements, |k, v| v.id = k),
            topology: self
                .topology
                .iter()
                .map(|(lane, links)| TopologyEntry {
                    lane,
                    links: links.clone(),
                })
                .collect(),
        }
    }
}

impl MapDocument {
    fn check_header(&self) -> Result<(), DocumentError> {
        if self.format != FORMAT_NAME {
            return Err(DocumentError::UnsupportedFormat(self.format.clone()));
        }
        if self.version > FORMAT_VERSION {
            return Err(DocumentError::UnsupportedVersion(self.version));
        }
        Ok(())
    }

    /// Converts the document into a map, tolerating duplicate IDs.
    ///
    /// The first occurrence of a duplicated ID is kept; every later one is
    /// dropped and reported as a `duplicate_id` error issue. References are
    /// not checked here; run the validator for that.
    pub fn into_map(self) -> Result<(Map, Vec<Issue>), DocumentError> {
        self.check_header()?;
        let mut map = Map::new();
        let mut issues = Vec::new();
        map.metadata = self.metadata;

        fn report(issues: &mut Vec<Issue>, r: Result<(), MapError>) {
            if let Err(MapError::DuplicateId(e)) = r {
                issues.push(
                    Issue::error(
                        codes::DUPLICATE_ID,
                        format!("{e} is defined more than once; later definitions were dropped"),
                    )
                    .with_entity(e),
                );
            }
        }
        for e in self.lanes {
            report(&mut issues, map.insert_lane(e));
        }
        for e in self.boundaries {
            report(&mut issues, map.insert_boundary(e));
        }
        for e in self.roads {
            report(&mut issues, map.insert_road(e));
        }
        for e in self.junctions {
            report(&mut issues, map.insert_junction(e));
        }
        for e in self.stop_lines {
            report(&mut issues, map.insert_stop_line(e));
        }
        for e in self.traffic_signals {
            report(&mut issues, map.insert_traffic_signal(e));
        }
        for e in self.crosswalks {
            report(&mut issues, map.insert_crosswalk(e));
        }
        for e in self.regulatory_elements {
            report(&mut issues, map.insert_regulatory_element(e));
        }
        let mut seen = BTreeSet::new();
        for mut entry in self.topology {
            if !seen.insert(entry.lane) {
                issues.push(
                    Issue::error(
                        codes::DUPLICATE_ID,
                        format!(
                            "topology entry for {} appears more than once; later entries were dropped",
                            entry.lane
                        ),
                    )
                    .with_entity(EntityRef::Lane(entry.lane)),
                );
                continue;
            }
            // Keep lists sorted and unique, as the rest of the crate expects.
            entry.links.predecessors.sort();
            entry.links.predecessors.dedup();
            entry.links.successors.sort();
            entry.links.successors.dedup();
            map.topology.insert_raw(entry.lane, entry.links);
        }
        if let Some(n) = self.next_id
            && n > 0
        {
            map.bump_next_id(n - 1);
        }
        Ok((map, issues))
    }

    /// Converts the document into a map, failing on the first duplicate ID.
    pub fn try_into_map(self) -> Result<Map, DocumentError> {
        let (map, issues) = self.into_map()?;
        match issues.into_iter().find_map(|i| i.entity) {
            Some(e) => Err(MapError::DuplicateId(e).into()),
            None => Ok(map),
        }
    }
}

impl Serialize for Map {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.to_document().serialize(s)
    }
}

impl<'de> Deserialize<'de> for Map {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        MapDocument::deserialize(d)?
            .try_into_map()
            .map_err(serde::de::Error::custom)
    }
}
