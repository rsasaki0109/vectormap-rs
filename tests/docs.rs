//! Keeps the documentation honest: every command example in
//! `docs/commands.md` must parse as a `Command`, and the examples must apply
//! to the intersection sample where their IDs exist.

use vectormap::core::{Command, EditError};

/// Extracts every `{"op": ...}` JSON object (possibly spanning lines).
fn command_examples(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(start) = rest.find(r#"{"op""#) {
        let bytes = &rest.as_bytes()[start..];
        let mut depth = 0usize;
        let mut end = None;
        for (i, b) in bytes.iter().enumerate() {
            match b {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        end = Some(i + 1);
                        break;
                    }
                }
                _ => {}
            }
        }
        let end = end.expect("balanced braces");
        let example = &rest[start..start + end];
        // Skip illustrative placeholders such as `{"op": "split_lane", ...}`.
        if !example.contains("...") {
            out.push(example.to_string());
        }
        rest = &rest[start + end..];
    }
    out
}

#[test]
fn command_examples_parse() {
    let text =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/docs/commands.md")).unwrap();
    let examples = command_examples(&text);
    assert!(
        examples.len() >= 25,
        "found only {} examples",
        examples.len()
    );
    let (map, _) = vectormap::core::samples::intersection();
    for ex in examples {
        let cmd: Command = serde_json::from_str(&ex)
            .unwrap_or_else(|e| panic!("example does not parse: {ex}\n{e}"));
        // Examples use made-up IDs; they must either apply or fail with a
        // structured error, never panic.
        let mut scratch = map.clone();
        match scratch.apply(&cmd) {
            Ok(_) | Err(EditError::NotFound { .. }) => {}
            Err(e) => {
                let json = serde_json::to_value(&e).unwrap();
                assert!(json.get("code").is_some(), "{ex}: {e}");
            }
        }
    }
}
