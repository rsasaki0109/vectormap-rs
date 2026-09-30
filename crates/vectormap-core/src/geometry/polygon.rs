//! Simple polygons (no holes). The ring is implicitly closed: the last vertex
//! is connected to the first and is **not** repeated.

use serde::{Deserialize, Serialize};

use super::{BoundingBox, Point2, Point3, Polyline3, segment_intersection};

/// A simple 2D polygon.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Polygon2 {
    /// Ring vertices (not closed explicitly).
    pub points: Vec<Point2>,
}

/// A simple polygon with 3D vertices (e.g. an area with elevation).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Polygon3 {
    /// Ring vertices (not closed explicitly).
    pub points: Vec<Point3>,
}

impl Polygon2 {
    /// Creates a polygon, dropping a repeated closing vertex if present.
    pub fn new(mut points: Vec<Point2>) -> Self {
        if points.len() > 1 && points.first() == points.last() {
            points.pop();
        }
        Self { points }
    }

    /// Signed area (positive for counter-clockwise rings).
    pub fn signed_area(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        (0..n)
            .map(|i| self.points[i].cross(self.points[(i + 1) % n]))
            .sum::<f64>()
            * 0.5
    }

    /// Absolute area.
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    /// Perimeter length.
    pub fn perimeter(&self) -> f64 {
        let n = self.points.len();
        if n < 2 {
            return 0.0;
        }
        (0..n)
            .map(|i| self.points[i].distance(self.points[(i + 1) % n]))
            .sum()
    }

    /// Point-in-polygon test (even-odd rule). Points on the edge may be
    /// reported either way.
    pub fn contains(&self, p: Point2) -> bool {
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (self.points[i], self.points[j]);
            if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Edges of the ring as `(start, end)` pairs.
    pub fn edges(&self) -> impl Iterator<Item = (Point2, Point2)> + '_ {
        let n = self.points.len();
        (0..n).map(move |i| (self.points[i], self.points[(i + 1) % n]))
    }

    /// `true` if the 2D projection of `line` crosses or lies inside the polygon.
    pub fn intersects_polyline(&self, line: &Polyline3) -> bool {
        if line.points.iter().any(|p| self.contains(p.xy())) {
            return true;
        }
        line.points.windows(2).any(|w| {
            self.edges()
                .any(|(a, b)| segment_intersection(w[0].xy(), w[1].xy(), a, b).is_some())
        })
    }

    /// Bounding box (z = 0).
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_points(self.points.iter().map(|p| p.with_z(0.0)))
    }
}

impl Polygon3 {
    /// Creates a polygon, dropping a repeated closing vertex if present.
    pub fn new(mut points: Vec<Point3>) -> Self {
        if points.len() > 1 && points.first() == points.last() {
            points.pop();
        }
        Self { points }
    }

    /// Projection onto the XY plane.
    pub fn to_2d(&self) -> Polygon2 {
        Polygon2 {
            points: self.points.iter().map(|p| p.xy()).collect(),
        }
    }

    /// Area of the XY projection.
    pub fn area_2d(&self) -> f64 {
        self.to_2d().area()
    }

    /// Bounding box.
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_points(self.points.iter().copied())
    }

    /// Polygon enclosed by two roughly parallel polylines oriented in the same
    /// direction (e.g. the left and right boundary of a lane).
    pub fn from_sides(left: &Polyline3, right: &Polyline3) -> Polygon3 {
        let mut points = right.points.clone();
        points.extend(left.points.iter().rev());
        Polygon3::new(points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Polygon2 {
        Polygon2::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(4.0, 4.0),
            Point2::new(0.0, 4.0),
            Point2::new(0.0, 0.0),
        ])
    }

    #[test]
    fn closing_vertex_is_dropped() {
        assert_eq!(square().points.len(), 4);
    }

    #[test]
    fn area_and_containment() {
        let sq = square();
        assert!((sq.area() - 16.0).abs() < 1e-12);
        assert!(sq.signed_area() > 0.0);
        assert!((sq.perimeter() - 16.0).abs() < 1e-12);
        assert!(sq.contains(Point2::new(2.0, 2.0)));
        assert!(!sq.contains(Point2::new(5.0, 2.0)));
    }

    #[test]
    fn polyline_intersection() {
        let sq = square();
        let crossing = Polyline3::from_xy(&[[-1.0, 2.0], [5.0, 2.0]]);
        let outside = Polyline3::from_xy(&[[-1.0, 5.0], [5.0, 5.0]]);
        assert!(sq.intersects_polyline(&crossing));
        assert!(!sq.intersects_polyline(&outside));
    }

    #[test]
    fn polygon_from_sides() {
        let left = Polyline3::from_xy(&[[0.0, 1.0], [10.0, 1.0]]);
        let right = Polyline3::from_xy(&[[0.0, -1.0], [10.0, -1.0]]);
        let poly = Polygon3::from_sides(&left, &right);
        assert_eq!(poly.points.len(), 4);
        assert!((poly.area_2d() - 20.0).abs() < 1e-12);
        assert!(poly.to_2d().signed_area() > 0.0, "counter-clockwise ring");
    }
}
