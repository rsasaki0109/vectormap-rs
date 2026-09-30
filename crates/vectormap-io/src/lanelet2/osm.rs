//! A minimal, lossless-enough model of OSM XML as used by Lanelet2, with a
//! parser (based on `roxmltree`) and a deterministic writer.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use vectormap_core::Issue;

use super::codes;
use crate::IoError;

/// Ordered tag map.
pub type Tags = BTreeMap<String, String>;

/// An OSM node.
#[derive(Debug, Clone, PartialEq)]
pub struct OsmNode {
    /// Node ID.
    pub id: i64,
    /// Latitude (degrees).
    pub lat: f64,
    /// Longitude (degrees).
    pub lon: f64,
    /// Tags.
    pub tags: Tags,
}

/// An OSM way.
#[derive(Debug, Clone, PartialEq)]
pub struct OsmWay {
    /// Way ID.
    pub id: i64,
    /// Node references in order.
    pub nodes: Vec<i64>,
    /// Tags.
    pub tags: Tags,
}

/// Type of a relation member.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MemberType {
    /// A node.
    Node,
    /// A way.
    Way,
    /// A relation.
    Relation,
}

impl MemberType {
    fn as_str(self) -> &'static str {
        match self {
            MemberType::Node => "node",
            MemberType::Way => "way",
            MemberType::Relation => "relation",
        }
    }
}

/// A relation member.
#[derive(Debug, Clone, PartialEq)]
pub struct OsmMember {
    /// Member type.
    pub kind: MemberType,
    /// Referenced ID.
    pub reference: i64,
    /// Role.
    pub role: String,
}

/// An OSM relation.
#[derive(Debug, Clone, PartialEq)]
pub struct OsmRelation {
    /// Relation ID.
    pub id: i64,
    /// Members in order.
    pub members: Vec<OsmMember>,
    /// Tags.
    pub tags: Tags,
}

impl OsmRelation {
    /// Members with the given type and role.
    pub fn members<'a>(
        &'a self,
        kind: MemberType,
        role: &'a str,
    ) -> impl Iterator<Item = i64> + 'a {
        self.members
            .iter()
            .filter(move |m| m.kind == kind && m.role == role)
            .map(|m| m.reference)
    }
}

/// The content of an OSM file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct OsmData {
    /// Nodes by ID.
    pub nodes: BTreeMap<i64, OsmNode>,
    /// Ways by ID.
    pub ways: BTreeMap<i64, OsmWay>,
    /// Relations by ID.
    pub relations: BTreeMap<i64, OsmRelation>,
}

fn attr<'a>(node: roxmltree::Node<'a, '_>, name: &str) -> Result<&'a str, IoError> {
    node.attribute(name).ok_or_else(|| {
        IoError::Format(format!(
            "<{}> element without `{name}` attribute",
            node.tag_name().name()
        ))
    })
}

fn parse_num<T: std::str::FromStr>(
    node: roxmltree::Node<'_, '_>,
    name: &str,
) -> Result<T, IoError> {
    let v = attr(node, name)?;
    v.trim().parse().map_err(|_| {
        IoError::Format(format!(
            "invalid `{name}` value {v:?} on <{}>",
            node.tag_name().name()
        ))
    })
}

fn tags_of(node: roxmltree::Node<'_, '_>) -> Result<Tags, IoError> {
    let mut tags = Tags::new();
    for t in node.children().filter(|c| c.has_tag_name("tag")) {
        tags.insert(attr(t, "k")?.to_string(), attr(t, "v")?.to_string());
    }
    Ok(tags)
}

fn duplicate(kind: &str, id: i64) -> Issue {
    Issue::error(
        codes::DUPLICATE_ID,
        format!("OSM {kind} {id} is defined more than once; later definitions were dropped"),
    )
}

fn insert_unique<T>(
    map: &mut BTreeMap<i64, T>,
    id: i64,
    value: T,
    kind: &str,
    issues: &mut Vec<Issue>,
) {
    match map.entry(id) {
        std::collections::btree_map::Entry::Vacant(e) => {
            e.insert(value);
        }
        std::collections::btree_map::Entry::Occupied(_) => issues.push(duplicate(kind, id)),
    }
}

/// Parses OSM XML. Elements with `action="delete"` are skipped; duplicate
/// IDs are reported and the first definition is kept.
pub fn parse(text: &str) -> Result<(OsmData, Vec<Issue>), IoError> {
    let doc = roxmltree::Document::parse(text)?;
    let root = doc.root_element();
    if !root.has_tag_name("osm") {
        return Err(IoError::Format(format!(
            "expected <osm> root element, found <{}>",
            root.tag_name().name()
        )));
    }
    let mut data = OsmData::default();
    let mut issues = Vec::new();
    for el in root.children().filter(roxmltree::Node::is_element) {
        if el.attribute("action") == Some("delete") {
            continue;
        }
        match el.tag_name().name() {
            "node" => {
                let id = parse_num(el, "id")?;
                let node = OsmNode {
                    id,
                    lat: parse_num(el, "lat")?,
                    lon: parse_num(el, "lon")?,
                    tags: tags_of(el)?,
                };
                insert_unique(&mut data.nodes, id, node, "node", &mut issues);
            }
            "way" => {
                let id = parse_num(el, "id")?;
                let mut nodes = Vec::new();
                for nd in el.children().filter(|c| c.has_tag_name("nd")) {
                    nodes.push(parse_num(nd, "ref")?);
                }
                let way = OsmWay {
                    id,
                    nodes,
                    tags: tags_of(el)?,
                };
                insert_unique(&mut data.ways, id, way, "way", &mut issues);
            }
            "relation" => {
                let id = parse_num(el, "id")?;
                let mut members = Vec::new();
                for m in el.children().filter(|c| c.has_tag_name("member")) {
                    let kind = match attr(m, "type")? {
                        "node" => MemberType::Node,
                        "way" => MemberType::Way,
                        "relation" => MemberType::Relation,
                        other => {
                            return Err(IoError::Format(format!(
                                "relation {id}: unknown member type {other:?}"
                            )));
                        }
                    };
                    members.push(OsmMember {
                        kind,
                        reference: parse_num(m, "ref")?,
                        role: m.attribute("role").unwrap_or("").to_string(),
                    });
                }
                let rel = OsmRelation {
                    id,
                    members,
                    tags: tags_of(el)?,
                };
                insert_unique(&mut data.relations, id, rel, "relation", &mut issues);
            }
            // <bounds>, <MetaInfo> and other elements carry no map content.
            _ => {}
        }
    }
    Ok((data, issues))
}

/// Escapes a string for use in an XML attribute value.
fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            '\n' => out.push_str("&#10;"),
            '\t' => out.push_str("&#9;"),
            c => out.push(c),
        }
    }
    out
}

fn write_tags(out: &mut String, tags: &Tags) {
    for (k, v) in tags {
        let _ = writeln!(out, "    <tag k=\"{}\" v=\"{}\"/>", escape(k), escape(v));
    }
}

/// Writes OSM XML deterministically: nodes, ways, relations, each sorted by
/// ID, tags sorted by key.
pub fn write(data: &OsmData, generator: &str) -> String {
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    let _ = writeln!(
        out,
        "<osm version=\"0.6\" upload=\"false\" generator=\"{}\">",
        escape(generator)
    );
    for n in data.nodes.values() {
        let _ = write!(
            out,
            "  <node id=\"{}\" visible=\"true\" version=\"1\" lat=\"{:.11}\" lon=\"{:.11}\"",
            n.id, n.lat, n.lon
        );
        if n.tags.is_empty() {
            out.push_str("/>\n");
        } else {
            out.push_str(">\n");
            write_tags(&mut out, &n.tags);
            out.push_str("  </node>\n");
        }
    }
    for w in data.ways.values() {
        let _ = writeln!(
            out,
            "  <way id=\"{}\" visible=\"true\" version=\"1\">",
            w.id
        );
        for r in &w.nodes {
            let _ = writeln!(out, "    <nd ref=\"{r}\"/>");
        }
        write_tags(&mut out, &w.tags);
        out.push_str("  </way>\n");
    }
    for r in data.relations.values() {
        let _ = writeln!(
            out,
            "  <relation id=\"{}\" visible=\"true\" version=\"1\">",
            r.id
        );
        for m in &r.members {
            let _ = writeln!(
                out,
                "    <member type=\"{}\" ref=\"{}\" role=\"{}\"/>",
                m.kind.as_str(),
                m.reference,
                escape(&m.role)
            );
        }
        write_tags(&mut out, &r.tags);
        out.push_str("  </relation>\n");
    }
    out.push_str("</osm>\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"<?xml version="1.0"?>
<osm version="0.6" generator="JOSM">
  <bounds minlat="0" minlon="0" maxlat="1" maxlon="1"/>
  <node id="-1" lat="49.0" lon="8.4"><tag k="ele" v="1.5"/></node>
  <node id="-2" lat="49.0001" lon="8.4"/>
  <node id="-3" lat="49.0002" lon="8.4" action="delete"/>
  <way id="-10"><nd ref="-1"/><nd ref="-2"/><tag k="type" v="line_thin"/><tag k="note" v="a &amp; b"/></way>
  <relation id="-20"><member type="way" ref="-10" role="left"/><tag k="type" v="lanelet"/></relation>
</osm>"#;

    #[test]
    fn parse_josm_style() {
        let (data, issues) = parse(SAMPLE).unwrap();
        assert!(issues.is_empty());
        assert_eq!(data.nodes.len(), 2, "deleted node skipped");
        assert_eq!(data.nodes[&-1].tags["ele"], "1.5");
        assert_eq!(data.ways[&-10].nodes, vec![-1, -2]);
        assert_eq!(data.ways[&-10].tags["note"], "a & b");
        let rel = &data.relations[&-20];
        assert_eq!(
            rel.members(MemberType::Way, "left").collect::<Vec<_>>(),
            vec![-10]
        );
    }

    #[test]
    fn write_then_parse() {
        let (data, _) = parse(SAMPLE).unwrap();
        let text = write(&data, "test");
        let (again, _) = parse(&text).unwrap();
        assert_eq!(data, again);
        assert!(text.contains("generator=\"test\""));
        assert!(text.contains("v=\"a &amp; b\""));
    }

    #[test]
    fn duplicates_are_reported() {
        let text = r#"<osm><node id="1" lat="0" lon="0"/><node id="1" lat="1" lon="1"/></osm>"#;
        let (data, issues) = parse(text).unwrap();
        assert_eq!(data.nodes[&1].lat, 0.0);
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].code, "duplicate_id");
    }

    #[test]
    fn rejects_non_osm() {
        assert!(parse("<gpx/>").is_err());
        assert!(parse("<osm><node id=\"x\" lat=\"0\" lon=\"0\"/></osm>").is_err());
    }
}
