//! # vectormap-mcp
//!
//! A [Model Context Protocol](https://modelcontextprotocol.io) server that
//! lets an AI assistant (e.g. Claude Code) inspect and edit vector maps
//! through **high-level tools** — `split_lane`, `add_traffic_light`,
//! `validate_map`, `export_lanelet2`, ... The assistant never edits OSM XML:
//! every change goes through the deterministic editing API of
//! `vectormap-core` and is answered with a structured change set.
//!
//! Transport: JSON-RPC 2.0 over stdio, one message per line
//! ([`serve_stdio`]). The server holds one map per session with undo
//! history ([`Session`]).
//!
//! ```
//! use serde_json::json;
//! use vectormap_mcp::Server;
//!
//! let mut server = Server::new();
//! let reply = server
//!     .handle(json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
//!                    "params": {"name": "new_map", "arguments": {"sample": "straight_road"}}}))
//!     .unwrap();
//! assert_eq!(reply["result"]["isError"], false);
//! ```

mod session;
mod tools;

use std::io::{self, BufRead, Write};

use serde_json::{Value, json};

pub use session::{MAX_UNDO, Session, ToolError};
pub use tools::{ToolDef, definitions};

/// Protocol versions this server can speak, newest first. The client's
/// requested version is echoed if supported, otherwise the newest is offered.
pub const SUPPORTED_PROTOCOL_VERSIONS: &[&str] =
    &["2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05"];

/// Server name reported in `initialize`.
pub const SERVER_NAME: &str = "vectormap";

/// Guidance sent to the client in `initialize`.
pub const INSTRUCTIONS: &str = "\
vectormap edits HD vector maps (lanes, boundaries, junctions, stop lines, traffic lights, \
crosswalks) through high-level tools. Workflow: open_map (or new_map) → inspect with \
get_map_summary / list_lanes / get_lane / find_nearest_lane → edit with create_lane, split_lane, \
merge_lanes, connect_lanes, add_stop_line, add_traffic_light, add_crosswalk, set_speed_limit or \
apply_commands → validate_map → save_map / export_lanelet2 (autoware=true for Autoware). \
Coordinates are metres in the map's local frame (x east, y north). IDs are integers. Every edit is \
atomic and returns created/modified/deleted entities plus warnings; undo reverts the last edit. \
Validation issues may carry a `fix` command that apply_commands accepts.";

/// JSON-RPC error codes.
mod rpc {
    pub const PARSE_ERROR: i64 = -32700;
    pub const INVALID_REQUEST: i64 = -32600;
    pub const METHOD_NOT_FOUND: i64 = -32601;
    pub const INVALID_PARAMS: i64 = -32602;
}

/// An MCP server holding one editing [`Session`].
#[derive(Debug, Default)]
pub struct Server {
    session: Session,
}

fn error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message.into()}})
}

fn success(id: Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

fn tool_result(result: Result<Value, ToolError>) -> Value {
    let (value, is_error) = match result {
        Ok(v) => (v, false),
        Err(e) => (e.to_json(), true),
    };
    let text = serde_json::to_string_pretty(&value).unwrap_or_default();
    let mut out = json!({
        "content": [{"type": "text", "text": text}],
        "isError": is_error,
    });
    if value.is_object() {
        out["structuredContent"] = value;
    }
    out
}

impl Server {
    /// A server without an open map.
    pub fn new() -> Self {
        Self::default()
    }

    /// A server whose session starts with `map` open.
    pub fn with_session(session: Session) -> Self {
        Self { session }
    }

    /// The editing session.
    pub fn session(&self) -> &Session {
        &self.session
    }

    /// Handles one raw line; returns the serialized reply, if any.
    pub fn handle_line(&mut self, line: &str) -> Option<String> {
        let reply = match serde_json::from_str::<Value>(line) {
            Ok(Value::Array(batch)) => {
                let replies: Vec<Value> =
                    batch.into_iter().filter_map(|m| self.handle(m)).collect();
                (!replies.is_empty()).then_some(Value::Array(replies))
            }
            Ok(msg) => self.handle(msg),
            Err(e) => Some(error(
                Value::Null,
                rpc::PARSE_ERROR,
                format!("parse error: {e}"),
            )),
        };
        reply.map(|r| r.to_string())
    }

    /// Handles one JSON-RPC message; returns the reply for requests and
    /// `None` for notifications and responses.
    pub fn handle(&mut self, msg: Value) -> Option<Value> {
        let Some(obj) = msg.as_object() else {
            return Some(error(
                Value::Null,
                rpc::INVALID_REQUEST,
                "expected a JSON object",
            ));
        };
        let id = obj.get("id").cloned();
        let Some(method) = obj.get("method").and_then(Value::as_str) else {
            // A response from the client (we never send requests): ignore.
            return match id {
                Some(id) if !obj.contains_key("result") && !obj.contains_key("error") => {
                    Some(error(id, rpc::INVALID_REQUEST, "missing method"))
                }
                _ => None,
            };
        };
        let params = obj.get("params").cloned().unwrap_or(Value::Null);
        let id = id?; // notifications (no id) get no reply
        Some(self.dispatch(id, method, params))
    }

    fn dispatch(&mut self, id: Value, method: &str, params: Value) -> Value {
        match method {
            "initialize" => {
                let requested = params["protocolVersion"].as_str().unwrap_or_default();
                let version = SUPPORTED_PROTOCOL_VERSIONS
                    .iter()
                    .find(|v| **v == requested)
                    .unwrap_or(&SUPPORTED_PROTOCOL_VERSIONS[0]);
                success(
                    id,
                    json!({
                        "protocolVersion": version,
                        "capabilities": {"tools": {"listChanged": false}},
                        "serverInfo": {
                            "name": SERVER_NAME,
                            "title": "vectormap-rs",
                            "version": env!("CARGO_PKG_VERSION"),
                        },
                        "instructions": INSTRUCTIONS,
                    }),
                )
            }
            "ping" => success(id, json!({})),
            "tools/list" => {
                let tools: Vec<Value> = definitions().iter().map(ToolDef::to_json).collect();
                success(id, json!({"tools": tools}))
            }
            "tools/call" => {
                let Some(name) = params["name"].as_str() else {
                    return error(id, rpc::INVALID_PARAMS, "tools/call needs a `name`");
                };
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                match tools::call(&mut self.session, name, args) {
                    tools::CallOutcome::Done(result) => success(id, tool_result(result)),
                    tools::CallOutcome::UnknownTool => {
                        error(id, rpc::INVALID_PARAMS, format!("unknown tool: {name}"))
                    }
                }
            }
            other => error(
                id,
                rpc::METHOD_NOT_FOUND,
                format!("method not found: {other}"),
            ),
        }
    }
}

/// Serves MCP over a line-based transport until `input` is exhausted.
pub fn serve<R: BufRead, W: Write>(server: &mut Server, input: R, mut output: W) -> io::Result<()> {
    for line in input.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        if let Some(reply) = server.handle_line(&line) {
            writeln!(output, "{reply}")?;
            output.flush()?;
        }
    }
    Ok(())
}

/// Serves MCP over stdin / stdout. Nothing else may write to stdout.
pub fn serve_stdio(server: &mut Server) -> io::Result<()> {
    let stdin = io::stdin();
    let stdout = io::stdout();
    serve(server, stdin.lock(), stdout.lock())
}
