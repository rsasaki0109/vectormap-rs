//! Autoware conventions on top of Lanelet2.
//!
//! Autoware consumes Lanelet2 maps with a set of additional conventions
//! (see `docs/autoware.md` and the Autoware "vector map requirements"). This
//! module keeps all of them out of `vectormap-core`:
//!
//! - an export profile ([`SaveOptions::autoware`]) that adds required tag
//!   defaults (`location`, `participant:*`), writes `local_x` / `local_y`
//!   and signal `height`s;
//! - [`check`], a compatibility lint with `autoware.*` issue codes;
//! - [`projector_info_yaml`] / [`save`], which write `lanelet2_map.osm`
//!   together with `map_projector_info.yaml`.

use std::path::Path;

use vectormap_core::{GeoReference, Issue, IssueCode, LaneKind, Map, ProjectionKind, Rule};

use crate::lanelet2::{SaveOptions, osm::Tags, save_lanelet2};
use crate::{IoError, write_file};

/// Housing height written for signals without one (metres).
pub const DEFAULT_SIGNAL_HEIGHT: f64 = 0.5;

/// File name Autoware expects for the vector map.
pub const MAP_FILE_NAME: &str = "lanelet2_map.osm";
/// File name Autoware expects for the projection description.
pub const PROJECTOR_INFO_FILE_NAME: &str = "map_projector_info.yaml";

/// Issue codes of the Autoware compatibility check.
pub mod codes {
    use vectormap_core::IssueCode;

    /// A vehicle lane has no speed limit (`speed_limit` is required).
    pub const MISSING_SPEED_LIMIT: IssueCode = IssueCode::new("autoware.missing_speed_limit");
    /// A junction lane has no `turn_direction`.
    pub const MISSING_TURN_DIRECTION: IssueCode = IssueCode::new("autoware.missing_turn_direction");
    /// A vehicle lane is bidirectional (`one_way=no` is not supported).
    pub const BIDIRECTIONAL_LANE: IssueCode = IssueCode::new("autoware.bidirectional_lane");
    /// A traffic light rule has no stop line.
    pub const TRAFFIC_LIGHT_WITHOUT_STOP_LINE: IssueCode =
        IssueCode::new("autoware.traffic_light_without_stop_line");
    /// A signal has no height; the default is written.
    pub const MISSING_SIGNAL_HEIGHT: IssueCode = IssueCode::new("autoware.missing_signal_height");
    /// A lane kind that Autoware's planning stack does not use.
    pub const UNSUPPORTED_LANE_KIND: IssueCode = IssueCode::new("autoware.unsupported_lane_kind");
    /// `stop_line` rules are exported as `road_marking`, which Autoware
    /// interprets as intersection guide stop lines.
    pub const STOP_LINE_AS_ROAD_MARKING: IssueCode =
        IssueCode::new("autoware.stop_line_as_road_marking");
    /// A rule kind the IR keeps as `other`; it is written without members.
    pub const UNMODELED_RULE: IssueCode = IssueCode::new("autoware.unmodeled_rule");
    /// No georeference: the `Local` projector will be used.
    pub const LOCAL_PROJECTOR: IssueCode = IssueCode::new("autoware.local_projector");
    /// MGRS origin is polar/invalid, or geometry leaves its 100 km square.
    pub const MGRS_GRID: IssueCode = IssueCode::new("autoware.mgrs_grid");
    /// A crosswalk is not crossed by any lane.
    pub const CROSSWALK_WITHOUT_LANES: IssueCode =
        IssueCode::new("autoware.crosswalk_without_lanes");
}

fn code_issue(code: IssueCode, severity: vectormap_core::Severity, msg: String) -> Issue {
    Issue::new(severity, code, msg)
}

/// Adds tags Autoware expects on road lanelets when they are absent.
pub(crate) fn apply_lanelet_defaults(tags: &mut Tags, kind: LaneKind) {
    tags.entry("location".into())
        .or_insert_with(|| "urban".into());
    if kind.is_vehicle_lane() || kind == LaneKind::Shoulder {
        tags.entry("participant:vehicle".into())
            .or_insert_with(|| "yes".into());
    }
    if kind == LaneKind::Walkway {
        tags.entry("participant:pedestrian".into())
            .or_insert_with(|| "yes".into());
    }
}

/// Adds tags Autoware expects on crosswalk lanelets when they are absent.
pub(crate) fn apply_crosswalk_defaults(tags: &mut Tags) {
    tags.entry("location".into())
        .or_insert_with(|| "urban".into());
}

/// Checks a map against Autoware's conventions.
pub fn check(map: &Map) -> Vec<Issue> {
    use vectormap_core::Severity::{Info, Warning};
    let mut issues = Vec::new();
    for lane in map.lanes() {
        if lane.kind.is_vehicle_lane() && lane.speed_limit.is_none() {
            issues.push(
                code_issue(
                    codes::MISSING_SPEED_LIMIT,
                    Warning,
                    format!(
                        "{} has no speed limit; Autoware requires `speed_limit`",
                        lane.id
                    ),
                )
                .with_entity(lane.id),
            );
        }
        if map.junction_of(lane.id).is_some() && lane.turn_direction.is_none() {
            issues.push(
                code_issue(
                    codes::MISSING_TURN_DIRECTION,
                    Warning,
                    format!("junction lane {} has no turn_direction", lane.id),
                )
                .with_entity(lane.id),
            );
        }
        if lane.kind.is_vehicle_lane() && !lane.one_way {
            issues.push(
                code_issue(
                    codes::BIDIRECTIONAL_LANE,
                    Warning,
                    format!(
                        "{} is bidirectional; Autoware only supports one_way=yes road lanes",
                        lane.id
                    ),
                )
                .with_entity(lane.id),
            );
        }
        if matches!(
            lane.kind,
            LaneKind::Bus
                | LaneKind::Bicycle
                | LaneKind::Parking
                | LaneKind::Emergency
                | LaneKind::Other
        ) {
            issues.push(
                code_issue(
                    codes::UNSUPPORTED_LANE_KIND,
                    Info,
                    format!(
                        "{} is a {:?} lane, which Autoware's planning does not use",
                        lane.id, lane.kind
                    ),
                )
                .with_entity(lane.id),
            );
        }
    }
    for s in map.traffic_signals() {
        if s.height.is_none() {
            issues.push(
                code_issue(
                    codes::MISSING_SIGNAL_HEIGHT,
                    Info,
                    format!(
                        "{} has no height; {DEFAULT_SIGNAL_HEIGHT} m will be written",
                        s.id
                    ),
                )
                .with_entity(s.id),
            );
        }
    }
    for re in map.regulatory_elements() {
        match &re.rule {
            Rule::TrafficLight {
                stop_line: None, ..
            } if re.controlled_crosswalks.is_empty()
                || !re.lanes.is_empty()
                || re.rule.signals().is_empty()
                || !re.rule.signals().iter().all(|s| {
                    map.traffic_signal(*s)
                        .is_some_and(|s| s.kind == vectormap_core::SignalKind::Pedestrian)
                })
                || !re
                    .controlled_crosswalks
                    .iter()
                    .all(|c| map.crosswalk(*c).is_some()) =>
            {
                issues.push(
                    code_issue(
                        codes::TRAFFIC_LIGHT_WITHOUT_STOP_LINE,
                        Warning,
                        format!("traffic light {} has no stop line", re.id),
                    )
                    .with_entity(re.id),
                )
            }
            Rule::StopLine { .. } => issues.push(
                code_issue(
                    codes::STOP_LINE_AS_ROAD_MARKING,
                    Info,
                    format!(
                        "{} is exported as a road_marking rule, which Autoware treats as an \
                         intersection guide stop line; use a stop_sign traffic_sign rule for \
                         regular stop lines",
                        re.id
                    ),
                )
                .with_entity(re.id),
            ),
            Rule::Other { kind } => issues.push(
                code_issue(
                    codes::UNMODELED_RULE,
                    Warning,
                    format!(
                        "{} ({kind}) is written without members and will not work in Autoware",
                        re.id
                    ),
                )
                .with_entity(re.id),
            ),
            _ => {}
        }
    }
    for c in map.crosswalks() {
        let crossed = map
            .regulatory_elements()
            .any(|r| r.rule.crosswalk() == Some(c.id) && !r.lanes.is_empty());
        if !crossed {
            issues.push(
                code_issue(
                    codes::CROSSWALK_WITHOUT_LANES,
                    Warning,
                    format!(
                        "{} is not linked to any crossing lane via a crosswalk rule",
                        c.id
                    ),
                )
                .with_entity(c.id),
            );
        }
    }
    if map.metadata().georeference.is_none() {
        issues.push(code_issue(
            codes::LOCAL_PROJECTOR,
            Info,
            "the map has no georeference; map_projector_info.yaml will use the Local projector \
             (coordinates from local_x/local_y)"
                .into(),
        ));
    }
    if let Some(g) = map
        .metadata()
        .georeference
        .filter(|g| g.projection == ProjectionKind::Mgrs)
    {
        let grid = crate::projection::mgrs_grid(g.origin);
        let projector = crate::projection::LocalProjector::new(g);
        let points = map
            .boundaries()
            .flat_map(|b| &b.geometry.points)
            .chain(map.stop_lines().flat_map(|s| &s.geometry.points))
            .chain(map.traffic_signals().flat_map(|s| &s.geometry.points))
            .copied()
            .chain(map.crosswalks().flat_map(|c| c.outline().points));
        if grid.is_none()
            || points
                .into_iter()
                .any(|p| crate::projection::mgrs_grid(projector.inverse(p)) != grid)
        {
            issues.push(Issue::error(
                codes::MGRS_GRID,
                "MGRS geometry must stay within the origin's single UTM grid square",
            ));
        }
    }
    issues
}

/// Contents of `map_projector_info.yaml` for a georeference.
///
/// - [`ProjectionKind::Utm`] → `LocalCartesianUTM`
/// - [`ProjectionKind::TransverseMercator`] → `TransverseMercator`
/// - [`ProjectionKind::Mgrs`] → `MGRS` with `mgrs_grid`
/// - no georeference → `Local`
pub fn projector_info_yaml(georeference: Option<GeoReference>) -> String {
    match georeference {
        None => "projector_type: Local\n".to_string(),
        Some(g) => {
            if g.projection == ProjectionKind::Mgrs {
                let grid = crate::projection::mgrs_grid(g.origin).unwrap_or_default();
                return format!("projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {grid}\n");
            }
            let ty = match g.projection {
                ProjectionKind::Utm => "LocalCartesianUTM",
                ProjectionKind::TransverseMercator => "TransverseMercator",
                ProjectionKind::Mgrs => unreachable!(),
            };
            format!(
                "projector_type: {ty}\nvertical_datum: WGS84\nmap_origin:\n  latitude: {:?}\n  \
                 longitude: {:?}\n  altitude: {:?}\n",
                g.origin.lat, g.origin.lon, g.origin.alt
            )
        }
    }
}

/// Writes an Autoware map directory: `lanelet2_map.osm` (Autoware profile)
/// and `map_projector_info.yaml`. Returns compatibility and export issues.
pub fn save(map: &Map, dir: impl AsRef<Path>) -> Result<Vec<Issue>, IoError> {
    let dir = dir.as_ref();
    std::fs::create_dir_all(dir).map_err(|source| IoError::File {
        path: dir.to_path_buf(),
        source,
    })?;
    let mut issues = check(map);
    issues.extend(save_lanelet2(
        map,
        dir.join(MAP_FILE_NAME),
        &SaveOptions::autoware(),
    )?);
    write_file(
        &dir.join(PROJECTOR_INFO_FILE_NAME),
        &projector_info_yaml(map.metadata().georeference),
    )?;
    Ok(issues)
}

#[cfg(test)]
mod tests {
    use super::*;
    use vectormap_core::{GeoPoint, samples};

    #[test]
    fn projector_info() {
        assert_eq!(projector_info_yaml(None), "projector_type: Local\n");
        let yaml = projector_info_yaml(Some(GeoReference {
            projection: ProjectionKind::Utm,
            origin: GeoPoint::new(35.5, 139.25),
        }));
        assert!(yaml.starts_with("projector_type: LocalCartesianUTM\n"));
        assert!(yaml.contains("  latitude: 35.5\n"));
        assert!(yaml.contains("  longitude: 139.25\n"));
    }

    #[test]
    fn sample_intersection_is_autoware_compatible() {
        let (map, _) = samples::intersection();
        let issues = check(&map);
        assert!(issues.is_empty(), "{issues:#?}");
    }

    #[test]
    fn check_reports_missing_attributes() {
        let (mut map, [a, _, _]) = samples::straight_road();
        map.set_speed_limit(&[a], None).unwrap();
        let codes: Vec<String> = check(&map).iter().map(|i| i.code.to_string()).collect();
        assert_eq!(codes, vec!["autoware.missing_speed_limit"]);
    }
}
