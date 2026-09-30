//! # vectormap
//!
//! A Rust-native vector map toolkit for autonomous driving and robotics.
//!
//! This facade crate re-exports the workspace crates:
//!
//! | Module | Crate | Purpose |
//! |---|---|---|
//! | [`core`] | `vectormap-core` | format-independent IR, geometry, topology, editing |
//! | [`validation`] | `vectormap-validation` | structured validation |
//! | [`io`] | `vectormap-io` | JSON IR, Lanelet2 adapter, Autoware profile |
//!
//! ```
//! use vectormap::prelude::*;
//! use vectormap::{io, validation};
//!
//! let (mut map, lanes) = vectormap::core::samples::intersection();
//! map.set_speed_limit(&[lanes.incoming[0]], Some(SpeedLimit::from_kmh(30.0))).unwrap();
//!
//! let report = validation::validate(&map, &Default::default());
//! assert!(!report.has_errors());
//!
//! let (osm, _warnings) = io::lanelet2::write_string(&map, &io::lanelet2::SaveOptions::autoware());
//! let reloaded = io::lanelet2::read_str(&osm, &Default::default()).unwrap().map;
//! assert_eq!(reloaded.lane_count(), map.lane_count());
//! ```

pub use vectormap_core as core;
pub use vectormap_io as io;
pub use vectormap_validation as validation;

/// Glob import of the most used IR types.
pub mod prelude {
    pub use vectormap_core::prelude::*;
}
