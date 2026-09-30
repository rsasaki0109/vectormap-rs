//! `vectormap`: inspect, validate, convert and edit vector maps.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand, ValueEnum};
use vectormap_core::{
    Command, GeoPoint, GeoReference, Issue, LaneId, Map, Point2, ProjectionKind, Severity, samples,
};
use vectormap_io::lanelet2::{self, LoadOptions, ProjectionChoice, SaveOptions};
use vectormap_io::{Format, autoware, json};
use vectormap_validation::{ValidationOptions, ValidationReport, validate};

#[derive(Parser)]
#[command(
    name = "vectormap",
    version,
    about = "A Rust-native vector map toolkit for autonomous driving and robotics.",
    long_about = None
)]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Print a summary of a map.
    Info {
        /// Map file (.osm = Lanelet2, .json = vectormap IR).
        file: PathBuf,
        #[command(flatten)]
        input: InputArgs,
        /// Print JSON instead of text.
        #[arg(long)]
        json: bool,
    },
    /// Validate a map and report structured issues. Exits with status 1 if
    /// errors were found.
    Validate {
        /// Map file.
        file: PathBuf,
        #[command(flatten)]
        input: InputArgs,
        /// Also check Autoware conventions.
        #[arg(long)]
        autoware: bool,
        /// Print JSON instead of text.
        #[arg(long)]
        json: bool,
        /// Omit informational issues.
        #[arg(long)]
        no_info: bool,
        /// Exit with status 1 on warnings too.
        #[arg(long)]
        deny_warnings: bool,
    },
    /// Convert a map between formats.
    Convert {
        /// Input file.
        input_file: PathBuf,
        /// Output file.
        output_file: PathBuf,
        #[command(flatten)]
        input: InputArgs,
        #[command(flatten)]
        output: OutputArgs,
    },
    /// Apply a JSON list of high-level commands (see `docs/commands.md`) and
    /// write the result. The batch is atomic: nothing is written if any
    /// command fails.
    Edit {
        /// Input map.
        input_file: PathBuf,
        /// JSON file with a command or an array of commands.
        commands: PathBuf,
        /// Output map.
        #[arg(short, long)]
        output_file: PathBuf,
        #[command(flatten)]
        input: InputArgs,
        #[command(flatten)]
        output: OutputArgs,
    },
    /// Print everything known about one lane as JSON.
    Lane {
        /// Map file.
        file: PathBuf,
        /// Lane ID.
        id: u64,
        #[command(flatten)]
        input: InputArgs,
    },
    /// Find the lane closest to a point (local coordinates) and print it as JSON.
    Nearest {
        /// Map file.
        file: PathBuf,
        /// X coordinate (metres).
        #[arg(allow_hyphen_values = true)]
        x: f64,
        /// Y coordinate (metres).
        #[arg(allow_hyphen_values = true)]
        y: f64,
        #[command(flatten)]
        input: InputArgs,
    },
    /// Run a Model Context Protocol (MCP) server on stdin/stdout, exposing
    /// map inspection and editing as tools for Claude Code and other MCP
    /// clients (see docs/mcp.md).
    Mcp {
        /// Map to open when the server starts.
        file: Option<PathBuf>,
        #[command(flatten)]
        input: InputArgs,
    },
    /// Write one of the built-in sample maps.
    Sample {
        /// Which sample.
        name: SampleName,
        /// Output file.
        output_file: PathBuf,
        #[command(flatten)]
        output: OutputArgs,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum SampleName {
    StraightRoad,
    TwoLaneRoad,
    Intersection,
}

#[derive(Clone, Copy, ValueEnum)]
enum FormatArg {
    Lanelet2,
    Json,
}

impl From<FormatArg> for Format {
    fn from(f: FormatArg) -> Self {
        match f {
            FormatArg::Lanelet2 => Format::Lanelet2,
            FormatArg::Json => Format::Json,
        }
    }
}

#[derive(Clone, Copy, ValueEnum)]
enum ProjectionArg {
    /// UTM zone of the origin, relative to the origin (Lanelet2 UtmProjector).
    Utm,
    /// Transverse Mercator centred on the origin.
    Tm,
    /// MGRS coordinates in the origin's 100 km UTM square.
    Mgrs,
}

#[derive(Args)]
struct InputArgs {
    /// Input format (default: from the file extension).
    #[arg(long, value_enum)]
    from: Option<FormatArg>,
    /// Lanelet2: georeference origin as `LAT,LON` (default: south-west corner
    /// of the data).
    #[arg(long, value_parser = parse_origin)]
    origin: Option<GeoPoint>,
    /// Lanelet2: projection used with `--origin`.
    #[arg(long, value_enum, default_value = "utm")]
    projection: ProjectionArg,
    /// Lanelet2: use the `local_x` / `local_y` node tags as coordinates.
    #[arg(long, conflicts_with = "origin")]
    local: bool,
}

#[derive(Args)]
struct OutputArgs {
    /// Output format (default: from the file extension).
    #[arg(long, value_enum)]
    to: Option<FormatArg>,
    /// Lanelet2: use the Autoware profile and also write
    /// `map_projector_info.yaml` next to the output file.
    #[arg(long)]
    autoware: bool,
}

fn parse_origin(s: &str) -> Result<GeoPoint, String> {
    let parts: Vec<&str> = s.split(',').map(str::trim).collect();
    let nums: Result<Vec<f64>, _> = parts.iter().map(|p| p.parse::<f64>()).collect();
    match nums.map_err(|e| e.to_string())?.as_slice() {
        [lat, lon] => Ok(GeoPoint::new(*lat, *lon)),
        [lat, lon, alt] => Ok(GeoPoint {
            lat: *lat,
            lon: *lon,
            alt: *alt,
        }),
        _ => Err("expected LAT,LON or LAT,LON,ALT".into()),
    }
}

fn format_of(path: &Path, explicit: Option<FormatArg>) -> Result<Format> {
    explicit
        .map(Format::from)
        .or_else(|| Format::from_path(path))
        .with_context(|| {
            format!(
                "cannot determine the format of {}; use --from/--to",
                path.display()
            )
        })
}

fn load(path: &Path, input: &InputArgs) -> Result<(Map, Vec<Issue>)> {
    let loaded = match format_of(path, input.from)? {
        Format::Json => json::load(path)?,
        Format::Lanelet2 => {
            let projection = if input.local {
                ProjectionChoice::LocalTags
            } else if let Some(origin) = input.origin {
                ProjectionChoice::Georeferenced(GeoReference {
                    projection: match input.projection {
                        ProjectionArg::Utm => ProjectionKind::Utm,
                        ProjectionArg::Tm => ProjectionKind::TransverseMercator,
                        ProjectionArg::Mgrs => ProjectionKind::Mgrs,
                    },
                    origin,
                })
            } else {
                ProjectionChoice::Auto
            };
            lanelet2::load_lanelet2(
                path,
                &LoadOptions {
                    projection,
                    ..Default::default()
                },
            )?
        }
    };
    Ok((loaded.map, loaded.issues))
}

fn save(map: &Map, path: &Path, output: &OutputArgs) -> Result<Vec<Issue>> {
    let format = format_of(path, output.to)?;
    match format {
        Format::Json => {
            if output.autoware {
                bail!("--autoware only applies to Lanelet2 output");
            }
            json::save(map, path)?;
            Ok(Vec::new())
        }
        Format::Lanelet2 => {
            if output.autoware {
                let mut issues = autoware::check(map);
                issues.extend(lanelet2::save_lanelet2(
                    map,
                    path,
                    &SaveOptions::autoware(),
                )?);
                let yaml = path
                    .parent()
                    .unwrap_or(Path::new("."))
                    .join(autoware::PROJECTOR_INFO_FILE_NAME);
                std::fs::write(
                    &yaml,
                    autoware::projector_info_yaml(map.metadata().georeference),
                )
                .with_context(|| format!("writing {}", yaml.display()))?;
                eprintln!("wrote {}", yaml.display());
                Ok(issues)
            } else {
                Ok(lanelet2::save_lanelet2(map, path, &SaveOptions::default())?)
            }
        }
    }
}

fn print_issues(issues: &[Issue]) {
    for i in issues {
        eprintln!("{i}");
    }
}

fn summary_text(map: &Map, load_issues: &[Issue]) -> String {
    let s = map.summary();
    let mut out = String::new();
    let _ = writeln!(out, "Map: {}", s.name.as_deref().unwrap_or("(unnamed)"));
    match s.georeference {
        Some(g) => {
            let _ = writeln!(
                out,
                "Georeference: {:?} origin ({}, {})",
                g.projection, g.origin.lat, g.origin.lon
            );
        }
        None => {
            let _ = writeln!(out, "Georeference: none (local coordinates)");
        }
    }
    let c = s.counts;
    let _ = writeln!(out, "Entities:");
    for (name, n) in [
        ("lanes", c.lanes),
        ("boundaries", c.boundaries),
        ("roads", c.roads),
        ("junctions", c.junctions),
        ("stop lines", c.stop_lines),
        ("traffic signals", c.traffic_signals),
        ("crosswalks", c.crosswalks),
        ("regulatory elements", c.regulatory_elements),
    ] {
        let _ = writeln!(out, "  {name:<20} {n}");
    }
    let join = |m: &std::collections::BTreeMap<String, usize>| {
        m.iter()
            .map(|(k, v)| format!("{k} {v}"))
            .collect::<Vec<_>>()
            .join(", ")
    };
    if !s.lane_kinds.is_empty() {
        let _ = writeln!(out, "Lane kinds: {}", join(&s.lane_kinds));
    }
    if !s.rule_types.is_empty() {
        let _ = writeln!(out, "Rules: {}", join(&s.rule_types));
    }
    let _ = writeln!(out, "Total lane length: {:.1} m", s.total_lane_length);
    if let Some(bb) = s.bounding_box {
        let _ = writeln!(
            out,
            "Bounding box: x [{:.2}, {:.2}]  y [{:.2}, {:.2}]  z [{:.2}, {:.2}]",
            bb.min.x, bb.max.x, bb.min.y, bb.max.y, bb.min.z, bb.max.z
        );
    }
    let t = s.topology;
    let _ = writeln!(
        out,
        "Topology: {} links, {} neighbor relations, {} entry / {} exit / {} isolated lanes",
        t.links, t.neighbor_relations, t.entry_lanes, t.exit_lanes, t.isolated_lanes
    );
    let notable = load_issues
        .iter()
        .filter(|i| i.severity >= Severity::Warning)
        .count();
    if notable > 0 {
        let _ = writeln!(
            out,
            "Load issues: {notable} warning(s)/error(s); run `vectormap validate` for details"
        );
    }
    out
}

fn report_text(report: &ValidationReport) -> String {
    let mut out = String::new();
    for i in &report.issues {
        let _ = writeln!(out, "{i}");
    }
    let c = report.counts;
    let _ = writeln!(
        out,
        "{} error(s), {} warning(s), {} info",
        c.errors, c.warnings, c.infos
    );
    out
}

fn read_commands(path: &Path) -> Result<Vec<Command>> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let value: serde_json::Value = serde_json::from_str(&text)
        .with_context(|| format!("{} is not valid JSON", path.display()))?;
    let commands = match value {
        serde_json::Value::Array(_) => serde_json::from_value(value)?,
        other => vec![serde_json::from_value(other)?],
    };
    Ok(commands)
}

fn run(cli: Cli) -> Result<ExitCode> {
    match cli.command {
        Cmd::Info { file, input, json } => {
            let (map, issues) = load(&file, &input)?;
            if json {
                let value = serde_json::json!({
                    "summary": map.summary(),
                    "load_issues": issues,
                });
                println!("{}", json::to_pretty_string(&value));
            } else {
                print!("{}", summary_text(&map, &issues));
            }
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Validate {
            file,
            input,
            autoware: check_autoware,
            json,
            no_info,
            deny_warnings,
        } => {
            let (map, load_issues) = load(&file, &input)?;
            let options = ValidationOptions {
                include_info: !no_info,
                ..Default::default()
            };
            let mut extra = load_issues;
            if check_autoware {
                extra.extend(autoware::check(&map));
            }
            if no_info {
                extra.retain(|i| i.severity > Severity::Info);
            }
            let report = validate(&map, &options).merge(extra);
            if json {
                println!("{}", json::to_pretty_string(&report));
            } else {
                print!("{}", report_text(&report));
            }
            let failed = report.has_errors() || (deny_warnings && report.counts.warnings > 0);
            Ok(if failed {
                ExitCode::FAILURE
            } else {
                ExitCode::SUCCESS
            })
        }
        Cmd::Convert {
            input_file,
            output_file,
            input,
            output,
        } => {
            let (map, load_issues) = load(&input_file, &input)?;
            print_issues(
                &load_issues
                    .into_iter()
                    .filter(|i| i.severity >= Severity::Warning)
                    .collect::<Vec<_>>(),
            );
            let issues = save(&map, &output_file, &output)?;
            print_issues(&issues);
            eprintln!(
                "wrote {} ({} lanes)",
                output_file.display(),
                map.lane_count()
            );
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Edit {
            input_file,
            commands,
            output_file,
            input,
            output,
        } => {
            let (mut map, _) = load(&input_file, &input)?;
            let commands = read_commands(&commands)?;
            let changes = match map.apply_all(&commands) {
                Ok(c) => c,
                Err(e) => {
                    println!("{}", json::to_pretty_string(&e));
                    bail!("{e}");
                }
            };
            println!("{}", json::to_pretty_string(&changes));
            let issues = save(&map, &output_file, &output)?;
            print_issues(&issues);
            eprintln!("wrote {}", output_file.display());
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Lane { file, id, input } => {
            let (map, _) = load(&file, &input)?;
            let info = map
                .lane_info(LaneId(id))
                .with_context(|| format!("lane {id} does not exist"))?;
            println!("{}", json::to_pretty_string(&info));
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Nearest { file, x, y, input } => {
            let (map, _) = load(&file, &input)?;
            let nearest = map
                .find_nearest_lane(Point2::new(x, y))
                .context("the map has no lanes")?;
            println!("{}", json::to_pretty_string(&nearest));
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Mcp { file, input } => {
            let mut session = vectormap_mcp::Session::new();
            if let Some(path) = file {
                let format = format_of(&path, input.from)?;
                let (map, issues) = load(&path, &input)?;
                print_issues(
                    &issues
                        .into_iter()
                        .filter(|i| i.severity >= Severity::Warning)
                        .collect::<Vec<_>>(),
                );
                eprintln!("vectormap mcp: opened {}", path.display());
                session.open(map, Some((path, format)));
            }
            eprintln!("vectormap mcp: serving on stdio");
            vectormap_mcp::serve_stdio(&mut vectormap_mcp::Server::with_session(session))?;
            Ok(ExitCode::SUCCESS)
        }
        Cmd::Sample {
            name,
            output_file,
            output,
        } => {
            let map = match name {
                SampleName::StraightRoad => samples::straight_road().0,
                SampleName::TwoLaneRoad => samples::two_lane_road().0,
                SampleName::Intersection => samples::intersection().0,
            };
            let issues = save(&map, &output_file, &output)?;
            print_issues(&issues);
            eprintln!("wrote {}", output_file.display());
            Ok(ExitCode::SUCCESS)
        }
    }
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::from(2)
        }
    }
}
