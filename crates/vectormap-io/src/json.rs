//! The JSON form of the IR (`vectormap-ir`).
//!
//! Output is pretty-printed, deterministic (entities sorted by ID) and ends
//! with a newline, so that it diffs well and is easy for humans and language
//! models to read.

use std::io;
use std::path::Path;

use serde::Serialize;
use serde_json::ser::Formatter;
use vectormap_core::{Map, MapDocument};

use crate::{IoError, Loaded, read_file, write_file};

/// Serializes a map to pretty JSON.
pub fn to_string(map: &Map) -> String {
    let mut s = to_pretty_string(&map.to_document());
    s.push('\n');
    s
}

/// Pretty-prints any value like `serde_json::to_string_pretty`, but keeps
/// arrays of scalars (points, ID lists) on a single line, so a polyline
/// takes one line per point instead of one line per coordinate.
pub fn to_pretty_string<T: Serialize + ?Sized>(value: &T) -> String {
    let mut out = Vec::new();
    let mut ser = serde_json::Serializer::with_formatter(&mut out, CompactLeafFormatter::default());
    value.serialize(&mut ser).expect("value serializes");
    String::from_utf8(out).expect("serde_json emits UTF-8")
}

#[derive(Clone, Copy, PartialEq)]
enum ArrayMode {
    Undecided,
    Inline,
    Multiline,
}

/// A pretty formatter that decides per array, from its first element,
/// whether to print it inline (scalars) or one element per line.
#[derive(Default)]
struct CompactLeafFormatter {
    indent: usize,
    arrays: Vec<ArrayMode>,
    /// An array element is about to start.
    pending: bool,
    /// The pending element is the first of its array.
    pending_first: bool,
    /// Whether each open object has at least one key.
    object_has_value: Vec<bool>,
}

impl CompactLeafFormatter {
    fn newline<W: ?Sized + io::Write>(&self, w: &mut W, indent: usize) -> io::Result<()> {
        w.write_all(b"\n")?;
        for _ in 0..indent {
            w.write_all(b"  ")?;
        }
        Ok(())
    }

    /// Called before any value (scalar or container) is written.
    fn before_value<W: ?Sized + io::Write>(
        &mut self,
        w: &mut W,
        container: bool,
    ) -> io::Result<()> {
        if !self.pending {
            return Ok(());
        }
        self.pending = false;
        let Some(mode) = self.arrays.last_mut() else {
            return Ok(());
        };
        if *mode == ArrayMode::Undecided {
            *mode = if container {
                ArrayMode::Multiline
            } else {
                ArrayMode::Inline
            };
        }
        match *mode {
            ArrayMode::Multiline => self.newline(w, self.indent),
            _ if !self.pending_first => w.write_all(b" "),
            _ => Ok(()),
        }
    }
}

macro_rules! scalar {
    ($($name:ident: $t:ty),* $(,)?) => {
        $(
            fn $name<W: ?Sized + io::Write>(&mut self, w: &mut W, value: $t) -> io::Result<()> {
                self.before_value(w, false)?;
                serde_json::ser::CompactFormatter.$name(w, value)
            }
        )*
    };
}

impl Formatter for CompactLeafFormatter {
    scalar!(
        write_bool: bool, write_i8: i8, write_i16: i16, write_i32: i32, write_i64: i64,
        write_i128: i128, write_u8: u8, write_u16: u16, write_u32: u32, write_u64: u64,
        write_u128: u128, write_f32: f32, write_f64: f64, write_number_str: &str,
    );

    fn write_null<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.before_value(w, false)?;
        w.write_all(b"null")
    }

    fn begin_string<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.before_value(w, false)?;
        w.write_all(b"\"")
    }

    fn begin_array<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.before_value(w, true)?;
        self.indent += 1;
        self.arrays.push(ArrayMode::Undecided);
        w.write_all(b"[")
    }

    fn end_array<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.indent -= 1;
        if self.arrays.pop() == Some(ArrayMode::Multiline) {
            self.newline(w, self.indent)?;
        }
        w.write_all(b"]")
    }

    fn begin_array_value<W: ?Sized + io::Write>(
        &mut self,
        w: &mut W,
        first: bool,
    ) -> io::Result<()> {
        if !first {
            w.write_all(b",")?;
        }
        self.pending = true;
        self.pending_first = first;
        Ok(())
    }

    fn end_array_value<W: ?Sized + io::Write>(&mut self, _w: &mut W) -> io::Result<()> {
        Ok(())
    }

    fn begin_object<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.before_value(w, true)?;
        self.indent += 1;
        self.object_has_value.push(false);
        w.write_all(b"{")
    }

    fn end_object<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        self.indent -= 1;
        if self.object_has_value.pop() == Some(true) {
            self.newline(w, self.indent)?;
        }
        w.write_all(b"}")
    }

    fn begin_object_key<W: ?Sized + io::Write>(
        &mut self,
        w: &mut W,
        first: bool,
    ) -> io::Result<()> {
        if !first {
            w.write_all(b",")?;
        }
        if let Some(v) = self.object_has_value.last_mut() {
            *v = true;
        }
        self.newline(w, self.indent)
    }

    fn begin_object_value<W: ?Sized + io::Write>(&mut self, w: &mut W) -> io::Result<()> {
        w.write_all(b": ")
    }
}

/// Parses JSON produced by [`to_string`] (or written by hand).
///
/// Duplicate IDs do not fail the load; they are reported as `duplicate_id`
/// issues and the first definition wins.
pub fn from_str(text: &str) -> Result<Loaded, IoError> {
    let doc: MapDocument = serde_json::from_str(text)?;
    let (map, issues) = doc.into_map()?;
    Ok(Loaded { map, issues })
}

/// Loads a JSON file.
pub fn load(path: impl AsRef<Path>) -> Result<Loaded, IoError> {
    from_str(&read_file(path.as_ref())?)
}

/// Saves a map as JSON.
pub fn save(map: &Map, path: impl AsRef<Path>) -> Result<(), IoError> {
    write_file(path.as_ref(), &to_string(map))
}

#[cfg(test)]
mod tests {
    use super::*;
    use vectormap_core::samples;

    #[test]
    fn round_trip() {
        let (map, _) = samples::intersection();
        let text = to_string(&map);
        let loaded = from_str(&text).unwrap();
        assert!(loaded.issues.is_empty());
        assert_eq!(loaded.map, map);
        assert_eq!(to_string(&loaded.map), text);
    }

    #[test]
    fn scalar_arrays_are_inline() {
        let v = serde_json::json!({"a": [[1.0, 2.5], [3, 4]], "b": [], "c": {"d": ["x", "y"]}, "e": [{"f": 1}]});
        let text = to_pretty_string(&v);
        assert_eq!(
            text,
            "{
  \"a\": [
    [1.0, 2.5],
    [3, 4]
  ],
  \"b\": [],
  \"c\": {
    \"d\": [\"x\", \"y\"]
  },
  \"e\": [
    {
      \"f\": 1
    }
  ]
}"
        );
        let back: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(back, v);
    }

    #[test]
    fn hand_written_minimal_document() {
        let text = r#"{
            "format": "vectormap-ir",
            "version": 1,
            "boundaries": [
                {"id": 1, "kind": {"type": "lane_marking", "pattern": "solid"}, "geometry": [[0,1.5,0],[10,1.5,0]]},
                {"id": 2, "kind": {"type": "curb"}, "geometry": [[0,-1.5,0],[10,-1.5,0]]}
            ],
            "lanes": [{"id": 3, "left": 1, "right": 2, "speed_limit": {"kmh": 30}}]
        }"#;
        let loaded = from_str(text).unwrap();
        let lane = loaded.map.lane(vectormap_core::LaneId(3)).unwrap();
        assert_eq!(lane.speed_limit.unwrap().kmh(), 30.0);
        assert!((loaded.map.lane_length(lane.id).unwrap() - 10.0).abs() < 1e-12);
    }

    #[test]
    fn rejects_foreign_documents() {
        assert!(matches!(
            from_str(r#"{"format": "geojson"}"#),
            Err(IoError::Document(_))
        ));
        assert!(matches!(from_str("{not json"), Err(IoError::Json(_))));
    }
}
