//! Polylines (linestrings) in 2D and 3D.

use serde::{Deserialize, Serialize};

use super::{BoundingBox, EPSILON, Point2, Point3, project_on_segment};

/// A 2D polyline.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Polyline2 {
    /// Vertices in order.
    pub points: Vec<Point2>,
}

/// A 3D polyline. The primary geometry type of the IR.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Polyline3 {
    /// Vertices in order.
    pub points: Vec<Point3>,
}

/// Result of a nearest-point query on a polyline.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NearestPoint {
    /// The closest point on the polyline.
    pub point: Point3,
    /// Distance in the XY plane from the query point to [`Self::point`].
    pub distance: f64,
    /// Arc length (station) of [`Self::point`] along the polyline, measured in 3D.
    pub station: f64,
    /// Index of the segment containing the point (`points[i]..points[i+1]`).
    pub segment: usize,
    /// Signed lateral offset of the query point: positive on the left side.
    pub lateral: f64,
}

impl Polyline2 {
    /// Creates a polyline from points.
    pub fn new(points: Vec<Point2>) -> Self {
        Self { points }
    }

    /// Total length.
    pub fn length(&self) -> f64 {
        self.points.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    /// Lifts to 3D with the given constant elevation.
    pub fn with_z(&self, z: f64) -> Polyline3 {
        Polyline3::new(self.points.iter().map(|p| p.with_z(z)).collect())
    }
}

impl From<Vec<Point2>> for Polyline2 {
    fn from(points: Vec<Point2>) -> Self {
        Self { points }
    }
}

impl From<Vec<Point3>> for Polyline3 {
    fn from(points: Vec<Point3>) -> Self {
        Self { points }
    }
}

impl Polyline3 {
    /// Creates a polyline from points.
    pub fn new(points: Vec<Point3>) -> Self {
        Self { points }
    }

    /// Convenience constructor from `[x, y, z]` triples.
    pub fn from_xyz(points: &[[f64; 3]]) -> Self {
        Self::new(points.iter().map(|&p| Point3::from(p)).collect())
    }

    /// Convenience constructor from `[x, y]` pairs at elevation 0.
    pub fn from_xy(points: &[[f64; 2]]) -> Self {
        Self::new(
            points
                .iter()
                .map(|&[x, y]| Point3::new(x, y, 0.0))
                .collect(),
        )
    }

    /// Number of vertices.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// `true` if the polyline has no vertices.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// First vertex.
    pub fn first(&self) -> Option<Point3> {
        self.points.first().copied()
    }

    /// Last vertex.
    pub fn last(&self) -> Option<Point3> {
        self.points.last().copied()
    }

    /// Total 3D length.
    pub fn length(&self) -> f64 {
        self.points.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    /// Total length in the XY plane.
    pub fn length_2d(&self) -> f64 {
        self.points.windows(2).map(|w| w[0].distance_2d(w[1])).sum()
    }

    /// Projection onto the XY plane.
    pub fn to_2d(&self) -> Polyline2 {
        Polyline2::new(self.points.iter().map(|p| p.xy()).collect())
    }

    /// Bounding box, or `None` if empty.
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_points(self.points.iter().copied())
    }

    /// The same polyline traversed in the opposite direction.
    pub fn reversed(&self) -> Polyline3 {
        let mut points = self.points.clone();
        points.reverse();
        Polyline3::new(points)
    }

    /// `true` if the polyline has at least two vertices and a positive length.
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2 && self.length() > EPSILON
    }

    /// Cumulative 3D arc length at each vertex (first entry is 0).
    pub fn stations(&self) -> Vec<f64> {
        let mut acc = 0.0;
        let mut out = Vec::with_capacity(self.points.len());
        for (i, p) in self.points.iter().enumerate() {
            if i > 0 {
                acc += self.points[i - 1].distance(*p);
            }
            out.push(acc);
        }
        out
    }

    /// Point at arc length `station` (clamped to `[0, length]`).
    pub fn point_at(&self, station: f64) -> Option<Point3> {
        let (i, t) = self.locate(station)?;
        if i + 1 >= self.points.len() {
            return self.last();
        }
        Some(self.points[i].lerp(self.points[i + 1], t))
    }

    /// Point at `fraction ∈ [0, 1]` of the total length.
    pub fn point_at_fraction(&self, fraction: f64) -> Option<Point3> {
        self.point_at(fraction.clamp(0.0, 1.0) * self.length())
    }

    /// Unit direction (XY plane) of the segment at `station`.
    pub fn direction_at(&self, station: f64) -> Option<Point2> {
        let (i, _) = self.locate(station)?;
        let n = self.points.len();
        if n < 2 {
            return None;
        }
        // Search forward, then backward, for a non-degenerate segment.
        let start = i.min(n - 2);
        (start..n - 1)
            .chain((0..start).rev())
            .find_map(|j| (self.points[j + 1].xy() - self.points[j].xy()).normalized())
    }

    /// Finds the segment index and local parameter for a station.
    fn locate(&self, station: f64) -> Option<(usize, f64)> {
        if self.points.is_empty() {
            return None;
        }
        if self.points.len() == 1 || station <= 0.0 {
            return Some((0, 0.0));
        }
        let mut acc = 0.0;
        for i in 0..self.points.len() - 1 {
            let seg = self.points[i].distance(self.points[i + 1]);
            if acc + seg >= station && seg > 0.0 {
                return Some((i, ((station - acc) / seg).clamp(0.0, 1.0)));
            }
            acc += seg;
        }
        Some((self.points.len() - 2, 1.0))
    }

    /// Nearest point on the polyline to `p`, measured in the XY plane.
    ///
    /// Ties are resolved in favour of the earliest segment, which keeps the
    /// result deterministic.
    pub fn nearest_point(&self, p: Point2) -> Option<NearestPoint> {
        match self.points.len() {
            0 => None,
            1 => {
                let q = self.points[0];
                Some(NearestPoint {
                    point: q,
                    distance: q.xy().distance(p),
                    station: 0.0,
                    segment: 0,
                    lateral: 0.0,
                })
            }
            _ => {
                let mut best: Option<NearestPoint> = None;
                let mut acc = 0.0;
                for i in 0..self.points.len() - 1 {
                    let (a, b) = (self.points[i], self.points[i + 1]);
                    let t = project_on_segment(p, a.xy(), b.xy());
                    let q = a.lerp(b, t);
                    let d = q.xy().distance(p);
                    let seg_len = a.distance(b);
                    if best.is_none_or(|bst| d < bst.distance - EPSILON) {
                        let dir = b.xy() - a.xy();
                        best = Some(NearestPoint {
                            point: q,
                            distance: d,
                            station: acc + t * seg_len,
                            segment: i,
                            lateral: dir.normalized().map_or(0.0, |u| u.cross(p - q.xy())),
                        });
                    }
                    acc += seg_len;
                }
                best
            }
        }
    }

    /// Distance in the XY plane from `p` to the polyline.
    pub fn distance_to(&self, p: Point2) -> Option<f64> {
        self.nearest_point(p).map(|n| n.distance)
    }

    /// Sub-polyline between two stations (`from < to`, both clamped).
    pub fn slice(&self, from: f64, to: f64) -> Polyline3 {
        let len = self.length();
        let (from, to) = (from.clamp(0.0, len), to.clamp(0.0, len));
        if self.points.len() < 2 || to <= from {
            return Polyline3::new(self.point_at(from).into_iter().collect());
        }
        let stations = self.stations();
        let mut out = Vec::new();
        out.extend(self.point_at(from));
        for (p, s) in self.points.iter().zip(&stations) {
            if *s > from + EPSILON && *s < to - EPSILON {
                out.push(*p);
            }
        }
        out.extend(self.point_at(to));
        Polyline3::new(out)
    }

    /// Splits the polyline at `station` into two pieces that share the split
    /// point exactly.
    pub fn split_at(&self, station: f64) -> (Polyline3, Polyline3) {
        let len = self.length();
        (self.slice(0.0, station), self.slice(station, len))
    }

    /// Resamples the polyline to `count ≥ 2` points evenly spaced by arc length.
    pub fn resample_count(&self, count: usize) -> Polyline3 {
        let count = count.max(2);
        let len = self.length();
        let pts = (0..count)
            .filter_map(|i| self.point_at(len * i as f64 / (count - 1) as f64))
            .collect();
        Polyline3::new(pts)
    }

    /// Resamples the polyline with approximately `step` metres between points,
    /// always keeping both end points.
    pub fn resample(&self, step: f64) -> Polyline3 {
        let len = self.length();
        if step <= 0.0 || len <= 0.0 {
            return self.clone();
        }
        let count = ((len / step).ceil() as usize + 1).max(2);
        self.resample_count(count)
    }

    /// Polyline offset laterally by `distance` metres in the XY plane
    /// (positive = left). Vertices are displaced along the averaged normal of
    /// the adjacent segments (miter joint, clamped for very sharp corners).
    pub fn offset(&self, distance: f64) -> Polyline3 {
        let n = self.points.len();
        if n < 2 {
            return self.clone();
        }
        let seg_normals: Vec<Option<Point2>> = self
            .points
            .windows(2)
            .map(|w| (w[1].xy() - w[0].xy()).normalized().map(Point2::perp))
            .collect();
        let pick = |i: usize| -> Point2 {
            // Nearest non-degenerate segment normal.
            (i..seg_normals.len())
                .chain((0..i).rev())
                .find_map(|j| seg_normals[j])
                .unwrap_or(Point2::new(0.0, 0.0))
        };
        let pts = (0..n)
            .map(|i| {
                let normal = if i == 0 {
                    pick(0)
                } else if i == n - 1 {
                    pick(n - 2)
                } else {
                    let (a, b) = (pick(i - 1), pick(i));
                    let sum = a + b;
                    match sum.normalized() {
                        Some(m) => {
                            // Miter length compensation, clamped to 4x.
                            let cos_half = m.dot(a).max(0.25);
                            m * (1.0 / cos_half)
                        }
                        None => a,
                    }
                };
                let p = self.points[i];
                Point3::new(p.x + normal.x * distance, p.y + normal.y * distance, p.z)
            })
            .collect();
        Polyline3::new(pts)
    }

    /// Removes consecutive duplicate vertices (closer than `tolerance`).
    pub fn dedup(&self, tolerance: f64) -> Polyline3 {
        let mut out: Vec<Point3> = Vec::with_capacity(self.points.len());
        for &p in &self.points {
            if out.last().is_none_or(|q| q.distance(p) > tolerance) {
                out.push(p);
            }
        }
        Polyline3::new(out)
    }

    /// Appends `other`, skipping its first vertex if it coincides with the
    /// last vertex of `self`.
    pub fn concat(&self, other: &Polyline3) -> Polyline3 {
        let mut points = self.points.clone();
        let skip = match (self.last(), other.first()) {
            (Some(a), Some(b)) if a.distance(b) < 1e-6 => 1,
            _ => 0,
        };
        points.extend(other.points.iter().skip(skip));
        Polyline3::new(points)
    }

    /// Centreline between two polylines oriented in the same direction.
    ///
    /// Both are resampled to the same number of points by normalized arc
    /// length and averaged.
    pub fn centerline(left: &Polyline3, right: &Polyline3) -> Option<Polyline3> {
        if left.is_empty() || right.is_empty() {
            return None;
        }
        let count = left.len().max(right.len()).max(2);
        let (l, r) = (left.resample_count(count), right.resample_count(count));
        Some(Polyline3::new(
            l.points
                .iter()
                .zip(&r.points)
                .map(|(a, b)| a.lerp(*b, 0.5))
                .collect(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn l_shape() -> Polyline3 {
        Polyline3::from_xy(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    }

    #[test]
    fn length_and_interpolation() {
        let pl = l_shape();
        assert!((pl.length() - 20.0).abs() < 1e-12);
        assert_eq!(pl.point_at(5.0).unwrap(), Point3::new(5.0, 0.0, 0.0));
        assert_eq!(pl.point_at(15.0).unwrap(), Point3::new(10.0, 5.0, 0.0));
        assert_eq!(pl.point_at(100.0).unwrap(), Point3::new(10.0, 10.0, 0.0));
        assert_eq!(pl.point_at(-1.0).unwrap(), Point3::new(0.0, 0.0, 0.0));
        assert_eq!(
            pl.point_at_fraction(0.5).unwrap(),
            Point3::new(10.0, 0.0, 0.0)
        );
    }

    #[test]
    fn nearest_point_reports_station_and_side() {
        let pl = l_shape();
        let n = pl.nearest_point(Point2::new(4.0, 2.0)).unwrap();
        assert_eq!(n.point, Point3::new(4.0, 0.0, 0.0));
        assert!((n.distance - 2.0).abs() < 1e-12);
        assert!((n.station - 4.0).abs() < 1e-12);
        assert_eq!(n.segment, 0);
        assert!(n.lateral > 0.0, "point is on the left side");
        let n = pl.nearest_point(Point2::new(12.0, 5.0)).unwrap();
        assert_eq!(n.segment, 1);
        assert!(n.lateral < 0.0);
    }

    #[test]
    fn split_shares_point() {
        let pl = l_shape();
        let (a, b) = pl.split_at(12.0);
        assert_eq!(a.last(), b.first());
        assert_eq!(a.last().unwrap(), Point3::new(10.0, 2.0, 0.0));
        assert!((a.length() - 12.0).abs() < 1e-12);
        assert!((b.length() - 8.0).abs() < 1e-12);
        assert_eq!(a.len(), 3);
        assert_eq!(b.len(), 2);
    }

    #[test]
    fn resample_and_offset() {
        let pl = Polyline3::from_xy(&[[0.0, 0.0], [10.0, 0.0]]);
        let r = pl.resample(2.5);
        assert_eq!(r.len(), 5);
        let left = pl.offset(1.5);
        assert_eq!(left.points[0], Point3::new(0.0, 1.5, 0.0));
        assert_eq!(left.points[1], Point3::new(10.0, 1.5, 0.0));
        // Miter keeps a constant distance around a corner.
        let off = l_shape().offset(-1.0);
        assert!((off.points[1].x - 11.0).abs() < 1e-9);
        assert!((off.points[1].y + 1.0).abs() < 1e-9);
    }

    #[test]
    fn centerline_of_parallel_lines() {
        let left = Polyline3::from_xy(&[[0.0, 2.0], [10.0, 2.0]]);
        let right = Polyline3::from_xy(&[[0.0, -2.0], [5.0, -2.0], [10.0, -2.0]]);
        let c = Polyline3::centerline(&left, &right).unwrap();
        assert_eq!(c.len(), 3);
        for p in &c.points {
            assert!(p.y.abs() < 1e-12);
        }
    }

    #[test]
    fn concat_and_dedup() {
        let a = Polyline3::from_xy(&[[0.0, 0.0], [1.0, 0.0]]);
        let b = Polyline3::from_xy(&[[1.0, 0.0], [2.0, 0.0]]);
        assert_eq!(a.concat(&b).len(), 3);
        let d = Polyline3::from_xy(&[[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]]).dedup(1e-9);
        assert_eq!(d.len(), 2);
    }
}
