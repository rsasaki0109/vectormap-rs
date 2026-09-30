//! Free-form key/value attributes: the IR's extension mechanism.
//!
//! Importers keep source-format tags that have no typed counterpart in the IR
//! under a format prefix, e.g. `lanelet2:location = urban`. Exporters write
//! back keys carrying their own prefix, ignore other prefixes, and write
//! un-prefixed (generic) keys as-is.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Ordered string key/value map attached to every entity.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Attributes(pub BTreeMap<String, String>);

impl Attributes {
    /// Creates an empty attribute map.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the value for `key`.
    pub fn get(&self, key: &str) -> Option<&str> {
        self.0.get(key).map(String::as_str)
    }

    /// Sets `key` to `value`, returning the previous value.
    pub fn insert(&mut self, key: impl Into<String>, value: impl Into<String>) -> Option<String> {
        self.0.insert(key.into(), value.into())
    }

    /// Removes `key`, returning its value.
    pub fn remove(&mut self, key: &str) -> Option<String> {
        self.0.remove(key)
    }

    /// `true` if there are no attributes.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Number of attributes.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Iterates over `(key, value)` pairs in key order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.0.iter().map(|(k, v)| (k.as_str(), v.as_str()))
    }

    /// Iterates over attributes stored under `prefix:` with the prefix
    /// stripped, e.g. `with_prefix("lanelet2")` yields `("location", "urban")`
    /// for `lanelet2:location = urban`.
    pub fn with_prefix<'a>(&'a self, prefix: &'a str) -> impl Iterator<Item = (&'a str, &'a str)> {
        self.0.iter().filter_map(move |(k, v)| {
            k.strip_prefix(prefix)
                .and_then(|rest| rest.strip_prefix(':'))
                .map(|rest| (rest, v.as_str()))
        })
    }

    /// Value of `prefix:key`.
    pub fn get_prefixed(&self, prefix: &str, key: &str) -> Option<&str> {
        self.get(&format!("{prefix}:{key}"))
    }

    /// Attributes without any `namespace:` prefix. A key counts as prefixed
    /// when the part before its first `:` is one of `known_prefixes`.
    pub fn generic<'a>(
        &'a self,
        known_prefixes: &'a [&'a str],
    ) -> impl Iterator<Item = (&'a str, &'a str)> {
        self.iter().filter(move |(k, _)| {
            k.split_once(':')
                .is_none_or(|(ns, _)| !known_prefixes.contains(&ns))
        })
    }
}

impl<K: Into<String>, V: Into<String>> FromIterator<(K, V)> for Attributes {
    fn from_iter<T: IntoIterator<Item = (K, V)>>(iter: T) -> Self {
        Self(
            iter.into_iter()
                .map(|(k, v)| (k.into(), v.into()))
                .collect(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefixes() {
        let attrs: Attributes = [
            ("lanelet2:location", "urban"),
            ("lanelet2:participant:vehicle", "yes"),
            ("opendrive:foo", "bar"),
            ("note", "hello"),
            ("custom:thing", "1"),
        ]
        .into_iter()
        .collect();
        let l2: Vec<_> = attrs.with_prefix("lanelet2").collect();
        assert_eq!(
            l2,
            vec![("location", "urban"), ("participant:vehicle", "yes")]
        );
        let generic: Vec<_> = attrs.generic(&["lanelet2", "opendrive"]).collect();
        assert_eq!(generic, vec![("custom:thing", "1"), ("note", "hello")]);
        assert_eq!(attrs.get_prefixed("opendrive", "foo"), Some("bar"));
    }
}
