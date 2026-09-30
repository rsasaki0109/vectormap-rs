//! Lanelet2 tag vocabulary shared by the reader and the writer.
//!
//! The reader stores a tag as an attribute only when it differs from what
//! the writer would generate from the typed IR fields, so canonical values
//! are never duplicated and non-canonical ones survive a round trip.

use vectormap_core::{
    BoundaryKind, BulbArrow, BulbColor, LaneKind, MarkingPattern, MarkingWeight, SignalKind,
    SpeedLimit, TurnDirection,
};

pub fn lane_kind_from_subtype(subtype: &str) -> LaneKind {
    match subtype {
        "road" | "highway" | "play_street" | "exit" => LaneKind::Driving,
        "road_shoulder" => LaneKind::Shoulder,
        "bus_lane" => LaneKind::Bus,
        "bicycle_lane" => LaneKind::Bicycle,
        "walkway" | "shared_walkway" | "pedestrian_lane" | "stairs" => LaneKind::Walkway,
        "emergency_lane" => LaneKind::Emergency,
        "parking" => LaneKind::Parking,
        _ => LaneKind::Other,
    }
}

pub fn lane_subtype(kind: LaneKind) -> &'static str {
    match kind {
        LaneKind::Driving => "road",
        LaneKind::Shoulder => "road_shoulder",
        LaneKind::Bus => "bus_lane",
        LaneKind::Bicycle => "bicycle_lane",
        LaneKind::Walkway => "walkway",
        LaneKind::Parking => "parking",
        LaneKind::Emergency => "emergency_lane",
        LaneKind::Other => "unknown",
    }
}

pub fn boundary_kind_from_tags(ty: Option<&str>, subtype: Option<&str>) -> BoundaryKind {
    match ty {
        Some(t @ ("line_thin" | "line_thick")) => BoundaryKind::LaneMarking {
            pattern: match subtype {
                Some("dashed") => MarkingPattern::Dashed,
                Some("solid_solid") => MarkingPattern::SolidSolid,
                Some("solid_dashed") => MarkingPattern::SolidDashed,
                Some("dashed_solid") => MarkingPattern::DashedSolid,
                _ => MarkingPattern::Solid,
            },
            weight: if t == "line_thick" {
                MarkingWeight::Thick
            } else {
                MarkingWeight::Thin
            },
        },
        Some("virtual") => BoundaryKind::Virtual,
        Some("curbstone") => BoundaryKind::Curb,
        Some("road_border" | "guard_rail" | "wall" | "fence" | "jersey_barrier") => {
            BoundaryKind::RoadEdge
        }
        _ => BoundaryKind::Other,
    }
}

pub fn boundary_tags(kind: BoundaryKind) -> (&'static str, Option<&'static str>) {
    match kind {
        BoundaryKind::LaneMarking { pattern, weight } => (
            match weight {
                MarkingWeight::Thin => "line_thin",
                MarkingWeight::Thick => "line_thick",
            },
            Some(match pattern {
                MarkingPattern::Solid => "solid",
                MarkingPattern::Dashed => "dashed",
                MarkingPattern::SolidSolid => "solid_solid",
                MarkingPattern::SolidDashed => "solid_dashed",
                MarkingPattern::DashedSolid => "dashed_solid",
            }),
        ),
        BoundaryKind::Virtual => ("virtual", None),
        BoundaryKind::Curb => ("curbstone", Some("high")),
        BoundaryKind::RoadEdge => ("road_border", None),
        BoundaryKind::Other => ("unknown", None),
    }
}

pub fn signal_kind_from_subtype(subtype: Option<&str>) -> SignalKind {
    match subtype {
        Some("red_green") => SignalKind::Pedestrian,
        _ => SignalKind::Vehicle,
    }
}

pub fn signal_subtype(kind: SignalKind) -> &'static str {
    match kind {
        SignalKind::Vehicle => "red_yellow_green",
        SignalKind::Pedestrian => "red_green",
    }
}

pub fn parse_turn(v: &str) -> Option<TurnDirection> {
    match v {
        "straight" => Some(TurnDirection::Straight),
        "left" => Some(TurnDirection::Left),
        "right" => Some(TurnDirection::Right),
        _ => None,
    }
}

pub fn turn_str(t: TurnDirection) -> &'static str {
    match t {
        TurnDirection::Straight => "straight",
        TurnDirection::Left => "left",
        TurnDirection::Right => "right",
    }
}

pub fn parse_color(v: &str) -> Option<BulbColor> {
    match v {
        "red" => Some(BulbColor::Red),
        "yellow" | "amber" => Some(BulbColor::Yellow),
        "green" => Some(BulbColor::Green),
        "white" => Some(BulbColor::White),
        _ => None,
    }
}

pub fn color_str(c: BulbColor) -> &'static str {
    match c {
        BulbColor::Red => "red",
        BulbColor::Yellow => "yellow",
        BulbColor::Green => "green",
        BulbColor::White => "white",
    }
}

pub fn parse_arrow(v: &str) -> Option<BulbArrow> {
    match v {
        "up" => Some(BulbArrow::Up),
        "down" => Some(BulbArrow::Down),
        "left" => Some(BulbArrow::Left),
        "right" => Some(BulbArrow::Right),
        "up_left" => Some(BulbArrow::UpLeft),
        "up_right" => Some(BulbArrow::UpRight),
        "down_left" => Some(BulbArrow::DownLeft),
        "down_right" => Some(BulbArrow::DownRight),
        _ => None,
    }
}

pub fn arrow_str(a: BulbArrow) -> &'static str {
    match a {
        BulbArrow::Up => "up",
        BulbArrow::Down => "down",
        BulbArrow::Left => "left",
        BulbArrow::Right => "right",
        BulbArrow::UpLeft => "up_left",
        BulbArrow::UpRight => "up_right",
        BulbArrow::DownLeft => "down_left",
        BulbArrow::DownRight => "down_right",
    }
}

/// Parses Lanelet2 velocity strings: a number with an optional unit
/// (`km/h` default, `kmh`, `mph`, `m/s`, `mps`).
pub fn parse_speed(v: &str) -> Option<SpeedLimit> {
    let v = v.trim();
    let split = v
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+'))
        .unwrap_or(v.len());
    let (num, unit) = v.split_at(split);
    let value: f64 = num.trim().parse().ok()?;
    let limit = match unit.trim().to_ascii_lowercase().as_str() {
        "" | "km/h" | "kmh" | "kph" => SpeedLimit::from_kmh(value),
        "mph" => SpeedLimit::from_kmh(value * 1.609_344),
        "m/s" | "mps" => SpeedLimit::from_mps(value),
        _ => return None,
    };
    limit.is_valid().then_some(limit)
}

/// Formats a number with at most six decimals and no trailing zeros.
pub fn fmt_decimal(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

/// Formats a coordinate with full round-trip precision.
pub fn fmt_coord(v: f64) -> String {
    if v == 0.0 { "0".into() } else { format!("{v}") }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_parsing() {
        assert_eq!(parse_speed("30").unwrap().kmh(), 30.0);
        assert_eq!(parse_speed("30.00").unwrap().kmh(), 30.0);
        assert_eq!(parse_speed("50 km/h").unwrap().kmh(), 50.0);
        assert!((parse_speed("10m/s").unwrap().kmh() - 36.0).abs() < 1e-9);
        assert!((parse_speed("20 mph").unwrap().kmh() - 32.18688).abs() < 1e-9);
        assert!(parse_speed("fast").is_none());
        assert!(parse_speed("0").is_none());
    }

    #[test]
    fn canonical_tags_round_trip() {
        for kind in [
            BoundaryKind::SOLID,
            BoundaryKind::DASHED,
            BoundaryKind::LaneMarking {
                pattern: MarkingPattern::DashedSolid,
                weight: MarkingWeight::Thick,
            },
            BoundaryKind::Virtual,
            BoundaryKind::Curb,
            BoundaryKind::RoadEdge,
            BoundaryKind::Other,
        ] {
            let (t, s) = boundary_tags(kind);
            assert_eq!(boundary_kind_from_tags(Some(t), s), kind);
        }
        for kind in [
            LaneKind::Driving,
            LaneKind::Shoulder,
            LaneKind::Bus,
            LaneKind::Bicycle,
            LaneKind::Walkway,
            LaneKind::Parking,
            LaneKind::Emergency,
            LaneKind::Other,
        ] {
            assert_eq!(lane_kind_from_subtype(lane_subtype(kind)), kind);
        }
    }

    #[test]
    fn number_formatting() {
        assert_eq!(fmt_decimal(30.0), "30");
        assert_eq!(fmt_decimal(8.5), "8.5");
        assert_eq!(fmt_decimal(-0.0000001), "0");
        assert_eq!(fmt_coord(1.75), "1.75");
        assert_eq!(fmt_coord(-0.0), "0");
        assert_eq!(fmt_coord(3.0), "3");
    }
}
