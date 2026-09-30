//! The [`Map`] container.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::attributes::Attributes;
use crate::entities::{
    Boundary, Crosswalk, Junction, Lane, RegulatoryElement, Road, Side, StopLine, TrafficSignal,
};
use crate::geometry::{BoundingBox, Polygon3, Polyline3};
use crate::id::{
    BoundaryId, CrosswalkId, EntityRef, JunctionId, LaneId, RegulatoryElementId, RoadId, SignalId,
    StopLineId,
};
use crate::topology::{Neighbor, Topology};

/// Map projection used to relate the local metric frame to WGS84.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProjectionKind {
    /// UTM zone containing the origin; local coordinates are UTM coordinates
    /// minus those of the origin (Lanelet2 `UtmProjector`, Autoware
    /// `LocalCartesianUTM`).
    Utm,
    /// Transverse Mercator centred on the origin's meridian, with the origin
    /// at (0, 0) (Autoware `TransverseMercator`).
    TransverseMercator,
    /// UTM-based MGRS coordinates within the origin's 100 km grid square.
    /// The origin identifies the square, rather than the local (0, 0).
    /// Polar UPS grids and maps spanning multiple squares are unsupported.
    Mgrs,
}

/// A WGS84 geographic position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GeoPoint {
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lon: f64,
    /// Altitude in metres.
    #[serde(default)]
    pub alt: f64,
}

impl GeoPoint {
    /// Creates a position at altitude 0.
    pub const fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon, alt: 0.0 }
    }
}

/// How the map's local frame relates to the earth.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GeoReference {
    /// Projection type.
    pub projection: ProjectionKind,
    /// Geographic position of the local origin.
    pub origin: GeoPoint,
}

/// Map-level metadata.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct MapMetadata {
    /// Human readable name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Georeference of the local frame, if known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub georeference: Option<GeoReference>,
    /// Extension attributes.
    #[serde(default, skip_serializing_if = "Attributes::is_empty")]
    pub attributes: Attributes,
}

/// Errors of raw map manipulation.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MapError {
    /// An entity with the same ID already exists.
    #[error("duplicate id: {0}")]
    DuplicateId(EntityRef),
}

/// A vector map: entities plus topology.
///
/// Read access is provided through typed accessors (`lane`, `lanes`, ...).
/// Mutations should go through the editing API (see [`crate::edit`]) or
/// [`Command`](crate::Command)s, which keep references and topology
/// consistent. The raw `insert_*` methods exist for importers.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Map {
    pub(crate) metadata: MapMetadata,
    pub(crate) lanes: BTreeMap<LaneId, Lane>,
    pub(crate) boundaries: BTreeMap<BoundaryId, Boundary>,
    pub(crate) roads: BTreeMap<RoadId, Road>,
    pub(crate) junctions: BTreeMap<JunctionId, Junction>,
    pub(crate) stop_lines: BTreeMap<StopLineId, StopLine>,
    pub(crate) traffic_signals: BTreeMap<SignalId, TrafficSignal>,
    pub(crate) crosswalks: BTreeMap<CrosswalkId, Crosswalk>,
    pub(crate) regulatory_elements: BTreeMap<RegulatoryElementId, RegulatoryElement>,
    pub(crate) topology: Topology,
    pub(crate) next_id: u64,
}

macro_rules! collection_accessors {
    (
        $field:ident: $id:ident => $ty:ident, $variant:ident,
        get: $get:ident, get_mut: $get_mut:ident, iter: $iter:ident,
        insert: $insert:ident, count: $count:ident
    ) => {
        #[doc = concat!("Returns the ", stringify!($ty), " with the given ID.")]
        pub fn $get(&self, id: $id) -> Option<&$ty> {
            self.$field.get(&id)
        }

        #[doc = concat!("Mutable access to a ", stringify!($ty), ".")]
        ///
        /// Prefer the editing API, which keeps references consistent. Changing
        /// the `id` field through this reference is a logic error.
        pub fn $get_mut(&mut self, id: $id) -> Option<&mut $ty> {
            self.$field.get_mut(&id)
        }

        #[doc = concat!("Iterates over all ", stringify!($field), " in ID order.")]
        pub fn $iter(&self) -> impl ExactSizeIterator<Item = &$ty> + DoubleEndedIterator {
            self.$field.values()
        }

        #[doc = concat!("Number of ", stringify!($field), ".")]
        pub fn $count(&self) -> usize {
            self.$field.len()
        }

        #[doc = concat!("Inserts a ", stringify!($ty), " as-is, without checking its references.")]
        ///
        /// Intended for importers. Fails if the ID is already in use.
        pub fn $insert(&mut self, entity: $ty) -> Result<(), MapError> {
            let id = entity.id;
            if self.$field.contains_key(&id) {
                return Err(MapError::DuplicateId(EntityRef::$variant(id)));
            }
            self.bump_next_id(id.0);
            self.$field.insert(id, entity);
            Ok(())
        }
    };
}

impl Map {
    /// Creates an empty map.
    pub fn new() -> Self {
        Self::default()
    }

    /// Map metadata.
    pub fn metadata(&self) -> &MapMetadata {
        &self.metadata
    }

    /// Mutable map metadata.
    pub fn metadata_mut(&mut self) -> &mut MapMetadata {
        &mut self.metadata
    }

    /// The topology store.
    pub fn topology(&self) -> &Topology {
        &self.topology
    }

    /// Replaces the whole topology (e.g. with the result of
    /// [`infer_topology`](crate::infer_topology)).
    pub fn set_topology(&mut self, topology: Topology) {
        self.topology = topology;
    }

    collection_accessors!(lanes: LaneId => Lane, Lane,
        get: lane, get_mut: lane_mut, iter: lanes, insert: insert_lane, count: lane_count);
    collection_accessors!(boundaries: BoundaryId => Boundary, Boundary,
        get: boundary, get_mut: boundary_mut, iter: boundaries, insert: insert_boundary,
        count: boundary_count);
    collection_accessors!(roads: RoadId => Road, Road,
        get: road, get_mut: road_mut, iter: roads, insert: insert_road, count: road_count);
    collection_accessors!(junctions: JunctionId => Junction, Junction,
        get: junction, get_mut: junction_mut, iter: junctions, insert: insert_junction,
        count: junction_count);
    collection_accessors!(stop_lines: StopLineId => StopLine, StopLine,
        get: stop_line, get_mut: stop_line_mut, iter: stop_lines, insert: insert_stop_line,
        count: stop_line_count);
    collection_accessors!(traffic_signals: SignalId => TrafficSignal, TrafficSignal,
        get: traffic_signal, get_mut: traffic_signal_mut, iter: traffic_signals,
        insert: insert_traffic_signal, count: traffic_signal_count);
    collection_accessors!(crosswalks: CrosswalkId => Crosswalk, Crosswalk,
        get: crosswalk, get_mut: crosswalk_mut, iter: crosswalks, insert: insert_crosswalk,
        count: crosswalk_count);
    collection_accessors!(regulatory_elements: RegulatoryElementId => RegulatoryElement,
        RegulatoryElement,
        get: regulatory_element, get_mut: regulatory_element_mut, iter: regulatory_elements,
        insert: insert_regulatory_element, count: regulatory_element_count);

    /// `true` if the referenced entity exists.
    pub fn contains(&self, entity: EntityRef) -> bool {
        match entity {
            EntityRef::Lane(id) => self.lanes.contains_key(&id),
            EntityRef::Boundary(id) => self.boundaries.contains_key(&id),
            EntityRef::Road(id) => self.roads.contains_key(&id),
            EntityRef::Junction(id) => self.junctions.contains_key(&id),
            EntityRef::StopLine(id) => self.stop_lines.contains_key(&id),
            EntityRef::TrafficSignal(id) => self.traffic_signals.contains_key(&id),
            EntityRef::Crosswalk(id) => self.crosswalks.contains_key(&id),
            EntityRef::RegulatoryElement(id) => self.regulatory_elements.contains_key(&id),
        }
    }

    /// Attributes of any entity.
    pub fn attributes(&self, entity: EntityRef) -> Option<&Attributes> {
        Some(match entity {
            EntityRef::Lane(id) => &self.lanes.get(&id)?.attributes,
            EntityRef::Boundary(id) => &self.boundaries.get(&id)?.attributes,
            EntityRef::Road(id) => &self.roads.get(&id)?.attributes,
            EntityRef::Junction(id) => &self.junctions.get(&id)?.attributes,
            EntityRef::StopLine(id) => &self.stop_lines.get(&id)?.attributes,
            EntityRef::TrafficSignal(id) => &self.traffic_signals.get(&id)?.attributes,
            EntityRef::Crosswalk(id) => &self.crosswalks.get(&id)?.attributes,
            EntityRef::RegulatoryElement(id) => &self.regulatory_elements.get(&id)?.attributes,
        })
    }

    pub(crate) fn attributes_mut(&mut self, entity: EntityRef) -> Option<&mut Attributes> {
        Some(match entity {
            EntityRef::Lane(id) => &mut self.lanes.get_mut(&id)?.attributes,
            EntityRef::Boundary(id) => &mut self.boundaries.get_mut(&id)?.attributes,
            EntityRef::Road(id) => &mut self.roads.get_mut(&id)?.attributes,
            EntityRef::Junction(id) => &mut self.junctions.get_mut(&id)?.attributes,
            EntityRef::StopLine(id) => &mut self.stop_lines.get_mut(&id)?.attributes,
            EntityRef::TrafficSignal(id) => &mut self.traffic_signals.get_mut(&id)?.attributes,
            EntityRef::Crosswalk(id) => &mut self.crosswalks.get_mut(&id)?.attributes,
            EntityRef::RegulatoryElement(id) => {
                &mut self.regulatory_elements.get_mut(&id)?.attributes
            }
        })
    }

    /// All entity references of the map in a stable order.
    pub fn entity_refs(&self) -> Vec<EntityRef> {
        let mut out: Vec<EntityRef> = Vec::new();
        out.extend(self.lanes.keys().map(|&id| EntityRef::Lane(id)));
        out.extend(self.boundaries.keys().map(|&id| EntityRef::Boundary(id)));
        out.extend(self.roads.keys().map(|&id| EntityRef::Road(id)));
        out.extend(self.junctions.keys().map(|&id| EntityRef::Junction(id)));
        out.extend(self.stop_lines.keys().map(|&id| EntityRef::StopLine(id)));
        out.extend(
            self.traffic_signals
                .keys()
                .map(|&id| EntityRef::TrafficSignal(id)),
        );
        out.extend(self.crosswalks.keys().map(|&id| EntityRef::Crosswalk(id)));
        out.extend(
            self.regulatory_elements
                .keys()
                .map(|&id| EntityRef::RegulatoryElement(id)),
        );
        out
    }

    // -- ID allocation -----------------------------------------------------

    pub(crate) fn bump_next_id(&mut self, used: u64) {
        if used >= self.next_id {
            self.next_id = used.saturating_add(1);
        }
    }

    /// The next raw ID that will be allocated.
    ///
    /// IDs come from a single counter shared by all entity kinds, so newly
    /// allocated IDs are unique across kinds.
    pub fn peek_next_id(&self) -> u64 {
        self.next_id.max(1)
    }

    pub(crate) fn alloc_raw(&mut self) -> u64 {
        let id = self.next_id.max(1);
        self.next_id = id + 1;
        id
    }

    /// Allocates a fresh lane ID.
    pub fn allocate_lane_id(&mut self) -> LaneId {
        LaneId(self.alloc_raw())
    }

    /// Allocates a fresh boundary ID.
    pub fn allocate_boundary_id(&mut self) -> BoundaryId {
        BoundaryId(self.alloc_raw())
    }

    // -- topology queries --------------------------------------------------

    /// Successors of `lane` (empty for unknown lanes).
    pub fn successors(&self, lane: LaneId) -> &[LaneId] {
        self.topology.successors(lane)
    }

    /// Predecessors of `lane` (empty for unknown lanes).
    pub fn predecessors(&self, lane: LaneId) -> &[LaneId] {
        self.topology.predecessors(lane)
    }

    /// Neighbor of `lane` on `side`, with its direction.
    pub fn neighbor(&self, lane: LaneId, side: Side) -> Option<Neighbor> {
        self.topology.neighbor(lane, side)
    }

    /// ID of the left neighbor of `lane`.
    pub fn left_neighbor(&self, lane: LaneId) -> Option<LaneId> {
        self.neighbor(lane, Side::Left).map(|n| n.lane)
    }

    /// ID of the right neighbor of `lane`.
    pub fn right_neighbor(&self, lane: LaneId) -> Option<LaneId> {
        self.neighbor(lane, Side::Right).map(|n| n.lane)
    }

    // -- derived geometry --------------------------------------------------

    /// Geometry of a lane boundary, oriented along the lane.
    pub fn oriented_boundary(&self, lane: LaneId, side: Side) -> Option<Polyline3> {
        let lane = self.lanes.get(&lane)?;
        let r = lane.boundary(side);
        Some(r.oriented(self.boundaries.get(&r.boundary)?))
    }

    /// Centreline of a lane: the explicit one if present, otherwise the
    /// average of its left and right boundaries.
    pub fn centerline(&self, lane: LaneId) -> Option<Polyline3> {
        let l = self.lanes.get(&lane)?;
        if let Some(c) = &l.centerline {
            return Some(c.clone());
        }
        Polyline3::centerline(
            &self.oriented_boundary(lane, Side::Left)?,
            &self.oriented_boundary(lane, Side::Right)?,
        )
    }

    /// Length of the lane centreline.
    pub fn lane_length(&self, lane: LaneId) -> Option<f64> {
        self.centerline(lane).map(|c| c.length())
    }

    /// Outline polygon of a lane.
    pub fn lane_polygon(&self, lane: LaneId) -> Option<Polygon3> {
        Some(Polygon3::from_sides(
            &self.oriented_boundary(lane, Side::Left)?,
            &self.oriented_boundary(lane, Side::Right)?,
        ))
    }

    /// Bounding box of all geometry in the map.
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        let boxes = self
            .boundaries
            .values()
            .filter_map(|b| b.geometry.bounding_box())
            .chain(
                self.lanes
                    .values()
                    .filter_map(|l| l.centerline.as_ref()?.bounding_box()),
            )
            .chain(
                self.stop_lines
                    .values()
                    .filter_map(|s| s.geometry.bounding_box()),
            )
            .chain(
                self.traffic_signals
                    .values()
                    .filter_map(|s| s.geometry.bounding_box()),
            )
            .chain(
                self.crosswalks
                    .values()
                    .filter_map(|c| c.outline().bounding_box()),
            )
            .chain(
                self.junctions
                    .values()
                    .filter_map(|j| j.outline.as_ref()?.bounding_box()),
            );
        boxes.reduce(BoundingBox::union)
    }

    // -- reverse lookups ---------------------------------------------------

    /// Lanes that use `boundary` on either side, in ID order.
    pub fn lanes_using_boundary(&self, boundary: BoundaryId) -> Vec<LaneId> {
        self.lanes
            .values()
            .filter(|l| l.left.boundary == boundary || l.right.boundary == boundary)
            .map(|l| l.id)
            .collect()
    }

    /// Regulatory elements that apply to `lane`.
    pub fn rules_for_lane(&self, lane: LaneId) -> Vec<&RegulatoryElement> {
        self.regulatory_elements
            .values()
            .filter(|r| r.lanes.contains(&lane))
            .collect()
    }

    /// Regulatory elements referencing the given stop line.
    pub fn rules_for_stop_line(&self, stop_line: StopLineId) -> Vec<&RegulatoryElement> {
        self.regulatory_elements
            .values()
            .filter(|r| r.rule.stop_lines().contains(&stop_line))
            .collect()
    }

    /// The road containing `lane`, if any.
    pub fn road_of(&self, lane: LaneId) -> Option<RoadId> {
        self.roads
            .values()
            .find(|r| r.lanes.contains(&lane))
            .map(|r| r.id)
    }

    /// The junction containing `lane`, if any.
    pub fn junction_of(&self, lane: LaneId) -> Option<JunctionId> {
        self.junctions
            .values()
            .find(|j| j.lanes.contains(&lane))
            .map(|j| j.id)
    }
}
