//! Helpers shared by the integration tests.

#![allow(dead_code)]

use serde_json::Value;
use vectormap::core::Map;

/// Asserts that two maps are equal, allowing numbers to differ by `tol`
/// (projection round trips through lat/lon are not bit-exact).
pub fn assert_maps_close(a: &Map, b: &Map, tol: f64) {
    let va = serde_json::to_value(a).unwrap();
    let vb = serde_json::to_value(b).unwrap();
    if let Err(path) = close(&va, &vb, tol, "$") {
        panic!("maps differ at {path}");
    }
}

fn close(a: &Value, b: &Value, tol: f64, path: &str) -> Result<(), String> {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            if (x - y).abs() <= tol {
                Ok(())
            } else {
                Err(format!("{path}: {x} != {y}"))
            }
        }
        (Value::Array(x), Value::Array(y)) => {
            if x.len() != y.len() {
                return Err(format!("{path}: length {} != {}", x.len(), y.len()));
            }
            for (i, (p, q)) in x.iter().zip(y).enumerate() {
                close(p, q, tol, &format!("{path}[{i}]"))?;
            }
            Ok(())
        }
        (Value::Object(x), Value::Object(y)) => {
            let kx: Vec<_> = x.keys().collect();
            let ky: Vec<_> = y.keys().collect();
            if kx != ky {
                return Err(format!("{path}: keys {kx:?} != {ky:?}"));
            }
            for (k, v) in x {
                close(v, &y[k], tol, &format!("{path}.{k}"))?;
            }
            Ok(())
        }
        _ if a == b => Ok(()),
        _ => Err(format!("{path}: {a} != {b}")),
    }
}

/// Path of a fixture file in `tests/data`.
pub fn fixture(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
        .join(name)
}
