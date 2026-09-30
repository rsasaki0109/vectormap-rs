//! Minimal geometry toolkit for vector maps.
//!
//! Coordinates are metres in a local Cartesian frame (x east, y north, z up).
//! The toolkit intentionally covers only what vector-map processing needs:
//! lengths, distances, nearest points, bounding boxes, interpolation and a few
//! 2D predicates. Points serialize as `[x, y]` / `[x, y, z]` arrays.

mod polygon;
mod polyline;

use std::ops::{Add, Mul, Sub};

use serde::{Deserialize, Serialize};

pub use polygon::{Polygon2, Polygon3};
pub use polyline::{NearestPoint, Polyline2, Polyline3};

/// Tolerance used for "same point" decisions throughout the crate (metres).
pub const EPSILON: f64 = 1e-9;

/// A 2D point / vector.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(from = "[f64; 2]", into = "[f64; 2]")]
pub struct Point2 {
    /// X coordinate (metres).
    pub x: f64,
    /// Y coordinate (metres).
    pub y: f64,
}

/// A 3D point / vector.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(from = "[f64; 3]", into = "[f64; 3]")]
pub struct Point3 {
    /// X coordinate (metres).
    pub x: f64,
    /// Y coordinate (metres).
    pub y: f64,
    /// Z coordinate (metres).
    pub z: f64,
}

impl Point2 {
    /// Creates a point.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Euclidean distance to `other`.
    pub fn distance(self, other: Point2) -> f64 {
        (self - other).norm()
    }

    /// Vector length.
    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    /// Dot product.
    pub fn dot(self, other: Point2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// 2D cross product (z component of the 3D cross product).
    pub fn cross(self, other: Point2) -> f64 {
        self.x * other.y - self.y * other.x
    }

    /// Unit vector in the same direction, or `None` for a zero vector.
    pub fn normalized(self) -> Option<Point2> {
        let n = self.norm();
        (n > EPSILON).then(|| Point2::new(self.x / n, self.y / n))
    }

    /// The vector rotated by +90° (pointing to the left of `self`).
    pub fn perp(self) -> Point2 {
        Point2::new(-self.y, self.x)
    }

    /// Linear interpolation between `self` (t = 0) and `other` (t = 1).
    pub fn lerp(self, other: Point2, t: f64) -> Point2 {
        self + (other - self) * t
    }

    /// Lifts the point to 3D with the given elevation.
    pub const fn with_z(self, z: f64) -> Point3 {
        Point3::new(self.x, self.y, z)
    }

    /// `true` if both coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Point3 {
    /// Creates a point.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Euclidean 3D distance to `other`.
    pub fn distance(self, other: Point3) -> f64 {
        (self - other).norm()
    }

    /// Distance in the XY plane, ignoring elevation.
    pub fn distance_2d(self, other: Point3) -> f64 {
        self.xy().distance(other.xy())
    }

    /// Vector length.
    pub fn norm(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Dot product.
    pub fn dot(self, other: Point3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Linear interpolation between `self` (t = 0) and `other` (t = 1).
    pub fn lerp(self, other: Point3, t: f64) -> Point3 {
        self + (other - self) * t
    }

    /// Projection onto the XY plane.
    pub const fn xy(self) -> Point2 {
        Point2::new(self.x, self.y)
    }

    /// `true` if all coordinates are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl From<[f64; 2]> for Point2 {
    fn from([x, y]: [f64; 2]) -> Self {
        Point2::new(x, y)
    }
}

impl From<Point2> for [f64; 2] {
    fn from(p: Point2) -> Self {
        [p.x, p.y]
    }
}

impl From<[f64; 3]> for Point3 {
    fn from([x, y, z]: [f64; 3]) -> Self {
        Point3::new(x, y, z)
    }
}

impl From<Point3> for [f64; 3] {
    fn from(p: Point3) -> Self {
        [p.x, p.y, p.z]
    }
}

impl From<Point2> for Point3 {
    fn from(p: Point2) -> Self {
        p.with_z(0.0)
    }
}

macro_rules! impl_vector_ops {
    ($t:ident { $($f:ident),+ }) => {
        impl Add for $t {
            type Output = $t;
            fn add(self, rhs: $t) -> $t {
                $t { $($f: self.$f + rhs.$f),+ }
            }
        }
        impl Sub for $t {
            type Output = $t;
            fn sub(self, rhs: $t) -> $t {
                $t { $($f: self.$f - rhs.$f),+ }
            }
        }
        impl Mul<f64> for $t {
            type Output = $t;
            fn mul(self, rhs: f64) -> $t {
                $t { $($f: self.$f * rhs),+ }
            }
        }
    };
}

impl_vector_ops!(Point2 { x, y });
impl_vector_ops!(Point3 { x, y, z });

/// An axis-aligned bounding box in 3D.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    /// Minimum corner.
    pub min: Point3,
    /// Maximum corner.
    pub max: Point3,
}

impl BoundingBox {
    /// Bounding box of a single point.
    pub const fn from_point(p: Point3) -> Self {
        Self { min: p, max: p }
    }

    /// Bounding box of a set of points; `None` if the iterator is empty.
    pub fn from_points<I: IntoIterator<Item = Point3>>(points: I) -> Option<Self> {
        let mut it = points.into_iter();
        let first = it.next()?;
        let mut bb = BoundingBox::from_point(first);
        for p in it {
            bb.include_point(p);
        }
        Some(bb)
    }

    /// Grows the box to include `p`.
    pub fn include_point(&mut self, p: Point3) {
        self.min = Point3::new(
            self.min.x.min(p.x),
            self.min.y.min(p.y),
            self.min.z.min(p.z),
        );
        self.max = Point3::new(
            self.max.x.max(p.x),
            self.max.y.max(p.y),
            self.max.z.max(p.z),
        );
    }

    /// Returns the smallest box containing both boxes.
    pub fn union(self, other: BoundingBox) -> BoundingBox {
        let mut bb = self;
        bb.include_point(other.min);
        bb.include_point(other.max);
        bb
    }

    /// Box grown by `margin` in every direction.
    pub fn expanded(self, margin: f64) -> BoundingBox {
        let m = Point3::new(margin, margin, margin);
        BoundingBox {
            min: self.min - m,
            max: self.max + m,
        }
    }

    /// `true` if the XY footprint contains `p`.
    pub fn contains_2d(&self, p: Point2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    /// `true` if the XY footprints of both boxes overlap.
    pub fn intersects_2d(&self, other: &BoundingBox) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Distance from `p` to the XY footprint (0 inside).
    pub fn distance_2d(&self, p: Point2) -> f64 {
        let dx = (self.min.x - p.x).max(0.0).max(p.x - self.max.x);
        let dy = (self.min.y - p.y).max(0.0).max(p.y - self.max.y);
        dx.hypot(dy)
    }

    /// Size along each axis.
    pub fn extent(&self) -> Point3 {
        self.max - self.min
    }

    /// Centre point.
    pub fn center(&self) -> Point3 {
        self.min.lerp(self.max, 0.5)
    }
}

/// Intersection point of the 2D segments `a0-a1` and `b0-b1`.
///
/// Returns the parameters `(t, u)` along each segment (both in `[0, 1]`) or
/// `None` if the segments do not intersect or are parallel.
pub fn segment_intersection(a0: Point2, a1: Point2, b0: Point2, b1: Point2) -> Option<(f64, f64)> {
    let r = a1 - a0;
    let s = b1 - b0;
    let denom = r.cross(s);
    if denom.abs() < EPSILON {
        return None;
    }
    let qp = b0 - a0;
    let t = qp.cross(s) / denom;
    let u = qp.cross(r) / denom;
    let range = -EPSILON..=1.0 + EPSILON;
    (range.contains(&t) && range.contains(&u)).then_some((t.clamp(0.0, 1.0), u.clamp(0.0, 1.0)))
}

/// Closest point to `p` on the segment `a-b`, returned as the parameter
/// `t ∈ [0, 1]` (XY plane only).
pub fn project_on_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let len2 = ab.dot(ab);
    if len2 < EPSILON * EPSILON {
        return 0.0;
    }
    ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_serialization_is_compact() {
        let p = Point3::new(1.0, 2.5, -3.0);
        assert_eq!(serde_json::to_string(&p).unwrap(), "[1.0,2.5,-3.0]");
        let q: Point2 = serde_json::from_str("[4,5]").unwrap();
        assert_eq!(q, Point2::new(4.0, 5.0));
    }

    #[test]
    fn distances() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(3.0, 4.0, 12.0);
        assert!((a.distance(b) - 13.0).abs() < 1e-12);
        assert!((a.distance_2d(b) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn bounding_box_ops() {
        let bb = BoundingBox::from_points([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, -2.0, 1.0),
            Point3::new(5.0, 3.0, -1.0),
        ])
        .unwrap();
        assert_eq!(bb.min, Point3::new(0.0, -2.0, -1.0));
        assert_eq!(bb.max, Point3::new(10.0, 3.0, 1.0));
        assert!(bb.contains_2d(Point2::new(5.0, 0.0)));
        assert!(!bb.contains_2d(Point2::new(11.0, 0.0)));
        assert!((bb.distance_2d(Point2::new(13.0, 7.0)) - 5.0).abs() < 1e-12);
        assert!(BoundingBox::from_points(std::iter::empty()).is_none());
    }

    #[test]
    fn segments_intersect() {
        let (t, u) = segment_intersection(
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
        )
        .unwrap();
        assert!((t - 0.5).abs() < 1e-12 && (u - 0.5).abs() < 1e-12);
        assert!(
            segment_intersection(
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(2.0, -1.0),
                Point2::new(2.0, 1.0),
            )
            .is_none()
        );
    }
}
