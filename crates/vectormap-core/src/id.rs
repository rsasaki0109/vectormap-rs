//! Strongly typed entity identifiers.
//!
//! Every entity kind has its own newtype so that, for example, a [`LaneId`]
//! cannot be passed where a [`BoundaryId`] is expected. All IDs serialize as
//! plain integers to keep the JSON representation readable.

use std::fmt;

use serde::{Deserialize, Serialize};

macro_rules! define_id {
    ($(#[$meta:meta])* $name:ident, $kind:ident, $prefix:literal) => {
        $(#[$meta])*
        #[derive(
            Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub u64);

        impl $name {
            /// Creates an ID from its raw numeric value.
            pub const fn new(raw: u64) -> Self {
                Self(raw)
            }

            /// Returns the raw numeric value.
            pub const fn get(self) -> u64 {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, concat!($prefix, ":{}"), self.0)
            }
        }

        impl From<$name> for EntityRef {
            fn from(id: $name) -> Self {
                EntityRef::$kind(id)
            }
        }
    };
}

define_id!(
    /// Identifier of a [`Lane`](crate::Lane).
    LaneId, Lane, "lane"
);
define_id!(
    /// Identifier of a [`Boundary`](crate::Boundary).
    BoundaryId, Boundary, "boundary"
);
define_id!(
    /// Identifier of a [`Road`](crate::Road).
    RoadId, Road, "road"
);
define_id!(
    /// Identifier of a [`Junction`](crate::Junction).
    JunctionId, Junction, "junction"
);
define_id!(
    /// Identifier of a [`StopLine`](crate::StopLine).
    StopLineId, StopLine, "stop_line"
);
define_id!(
    /// Identifier of a [`TrafficSignal`](crate::TrafficSignal).
    SignalId, TrafficSignal, "traffic_signal"
);
define_id!(
    /// Identifier of a [`Crosswalk`](crate::Crosswalk).
    CrosswalkId, Crosswalk, "crosswalk"
);
define_id!(
    /// Identifier of a [`RegulatoryElement`](crate::RegulatoryElement).
    RegulatoryElementId, RegulatoryElement, "regulatory_element"
);

/// The kind of an entity, without its ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EntityKind {
    /// A lane.
    Lane,
    /// A lane boundary.
    Boundary,
    /// A road (group of lanes).
    Road,
    /// A junction.
    Junction,
    /// A stop line.
    StopLine,
    /// A traffic signal head.
    TrafficSignal,
    /// A crosswalk.
    Crosswalk,
    /// A regulatory element (rule).
    RegulatoryElement,
}

impl EntityKind {
    /// Snake-case name as used in serialized data.
    pub const fn as_str(self) -> &'static str {
        match self {
            EntityKind::Lane => "lane",
            EntityKind::Boundary => "boundary",
            EntityKind::Road => "road",
            EntityKind::Junction => "junction",
            EntityKind::StopLine => "stop_line",
            EntityKind::TrafficSignal => "traffic_signal",
            EntityKind::Crosswalk => "crosswalk",
            EntityKind::RegulatoryElement => "regulatory_element",
        }
    }
}

impl fmt::Display for EntityKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A reference to any entity of a map.
///
/// Used in change sets and diagnostics. Serializes as
/// `{"kind": "lane", "id": 12}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "id", rename_all = "snake_case")]
pub enum EntityRef {
    /// A lane.
    Lane(LaneId),
    /// A boundary.
    Boundary(BoundaryId),
    /// A road.
    Road(RoadId),
    /// A junction.
    Junction(JunctionId),
    /// A stop line.
    StopLine(StopLineId),
    /// A traffic signal.
    TrafficSignal(SignalId),
    /// A crosswalk.
    Crosswalk(CrosswalkId),
    /// A regulatory element.
    RegulatoryElement(RegulatoryElementId),
}

impl EntityRef {
    /// The kind of the referenced entity.
    pub const fn kind(self) -> EntityKind {
        match self {
            EntityRef::Lane(_) => EntityKind::Lane,
            EntityRef::Boundary(_) => EntityKind::Boundary,
            EntityRef::Road(_) => EntityKind::Road,
            EntityRef::Junction(_) => EntityKind::Junction,
            EntityRef::StopLine(_) => EntityKind::StopLine,
            EntityRef::TrafficSignal(_) => EntityKind::TrafficSignal,
            EntityRef::Crosswalk(_) => EntityKind::Crosswalk,
            EntityRef::RegulatoryElement(_) => EntityKind::RegulatoryElement,
        }
    }

    /// The raw numeric ID.
    pub const fn raw(self) -> u64 {
        match self {
            EntityRef::Lane(id) => id.0,
            EntityRef::Boundary(id) => id.0,
            EntityRef::Road(id) => id.0,
            EntityRef::Junction(id) => id.0,
            EntityRef::StopLine(id) => id.0,
            EntityRef::TrafficSignal(id) => id.0,
            EntityRef::Crosswalk(id) => id.0,
            EntityRef::RegulatoryElement(id) => id.0,
        }
    }
}

impl fmt::Display for EntityRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.kind(), self.raw())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_serialize_as_integers() {
        assert_eq!(serde_json::to_string(&LaneId(7)).unwrap(), "7");
        let id: BoundaryId = serde_json::from_str("42").unwrap();
        assert_eq!(id, BoundaryId(42));
    }

    #[test]
    fn entity_ref_serializes_with_kind() {
        let r = EntityRef::from(LaneId(3));
        assert_eq!(
            serde_json::to_string(&r).unwrap(),
            r#"{"kind":"lane","id":3}"#
        );
        assert_eq!(r.to_string(), "lane:3");
        assert_eq!(r.kind(), EntityKind::Lane);
    }
}
