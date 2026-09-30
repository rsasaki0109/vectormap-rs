//! # vectormap-core
//!
//! Format-independent vector map intermediate representation (IR) for
//! autonomous driving and robotics.
//!
//! - [`Map`] holds typed entities ([`Lane`], [`Boundary`], [`Road`],
//!   [`Junction`], [`StopLine`], [`TrafficSignal`], [`Crosswalk`],
//!   [`RegulatoryElement`]) and a separate lane [`Topology`].
//! - Every entity kind has its own ID newtype ([`LaneId`], [`BoundaryId`], ...).
//! - All mutations go through high-level editing operations that return a
//!   [`ChangeSet`], or through serializable [`Command`]s.
//! - Diagnostics are structured [`Issue`]s with stable codes.
//!
//! ```
//! use vectormap_core::prelude::*;
//!
//! let mut map = Map::new();
//! let (a, _) = map
//!     .add_lane(NewLane::from_centerline(Polyline3::from_xy(&[[0.0, 0.0], [10.0, 0.0]]), 3.5))
//!     .unwrap();
//! let (b, _) = map
//!     .add_lane(
//!         NewLane::from_centerline(Polyline3::from_xy(&[[10.0, 0.0], [20.0, 0.0]]), 3.5)
//!             .with_predecessors(vec![a]),
//!     )
//!     .unwrap();
//! assert_eq!(map.successors(a), &[b]);
//!
//! let changes = map.split_lane(a, SplitAt::Fraction(0.5), SplitOptions::default()).unwrap().1;
//! assert_eq!(changes.created_ids(EntityKind::Lane).len(), 1);
//! ```

pub mod attributes;
pub mod diagnostics;
pub mod document;
pub mod edit;
pub mod entities;
pub mod geometry;
pub mod id;
pub mod map;
pub mod ops;
pub mod query;
pub mod samples;
pub mod topology;

pub use attributes::Attributes;
pub use diagnostics::{Issue, IssueCode, Severity};
pub use document::{DocumentError, MapDocument, TopologyEntry};
pub use edit::{
    BoundarySpec, ChangeSet, CrosswalkGeometry, EditError, EditResult, LaneGeometry, NewCrosswalk,
    NewLane, NewStopLine, NewTrafficSignal, SplitAt, SplitOptions, StopLineChoice,
    StopLinePlacement, StopRule,
};
pub use entities::{
    Boundary, BoundaryKind, BoundaryRef, BulbArrow, BulbColor, Crosswalk, Junction, Lane, LaneKind,
    MarkingPattern, MarkingWeight, RegulatoryElement, Road, Rule, Side, SignalBulb, SignalKind,
    SpeedLimit, StopLine, TrafficSignal, TurnDirection,
};
pub use geometry::{
    BoundingBox, NearestPoint, Point2, Point3, Polygon2, Polygon3, Polyline2, Polyline3,
};
pub use id::{
    BoundaryId, CrosswalkId, EntityKind, EntityRef, JunctionId, LaneId, RegulatoryElementId,
    RoadId, SignalId, StopLineId,
};
pub use map::{GeoPoint, GeoReference, Map, MapError, MapMetadata, ProjectionKind};
pub use ops::{BatchError, Command};
pub use query::{EntityCounts, LaneInfo, MapSummary, NearestLane, TopologySummary};
pub use topology::{
    InferOptions, LaneLinks, Neighbor, NeighborDirection, Topology, infer_topology,
};

/// Convenient glob import of the most used types.
pub mod prelude {
    pub use crate::{
        Attributes, Boundary, BoundaryId, BoundaryKind, BoundaryRef, ChangeSet, Command, Crosswalk,
        CrosswalkId, EditError, EntityKind, EntityRef, GeoPoint, GeoReference, Issue, Junction,
        JunctionId, Lane, LaneId, LaneKind, Map, Neighbor, NewCrosswalk, NewLane, NewStopLine,
        NewTrafficSignal, Point2, Point3, Polygon2, Polygon3, Polyline2, Polyline3, ProjectionKind,
        RegulatoryElement, RegulatoryElementId, Road, RoadId, Rule, Severity, Side, SignalId,
        SpeedLimit, SplitAt, SplitOptions, StopLine, StopLineId, TrafficSignal, TurnDirection,
    };
}
