//! WGS84 ⇄ local metric coordinates.
//!
//! Implements the transverse Mercator projection with Krüger's series (as
//! used by GeographicLib, to third order in the flattening), which is
//! accurate to well below a millimetre within a UTM zone. On top of it,
//! [`LocalProjector`] maps between geographic coordinates and the map's local
//! frame according to a [`GeoReference`]:
//!
//! - [`ProjectionKind::Utm`]: UTM zone of the origin, relative to the origin
//!   (Lanelet2 `UtmProjector`, Autoware `LocalCartesianUTM`);
//! - [`ProjectionKind::TransverseMercator`]: central meridian through the
//!   origin, origin at (0, 0) (Autoware `TransverseMercator`).
//! - [`ProjectionKind::Mgrs`]: UTM coordinates within the 100 km square
//!   containing the origin (Autoware `MGRS`); only single UTM grids are supported.
//!
//! Elevation is passed through unchanged (`z = ele`), as Lanelet2 does.

use vectormap_core::{GeoPoint, GeoReference, Point3, ProjectionKind};

/// WGS84 semi-major axis (metres).
pub const WGS84_A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// UTM scale factor on the central meridian.
pub const UTM_K0: f64 = 0.9996;

/// A transverse Mercator projection on the WGS84 ellipsoid (no false
/// easting / northing).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransverseMercator {
    lon0: f64,
    k0: f64,
    a_hat: f64,
    e: f64,
    alpha: [f64; 3],
    beta: [f64; 3],
    delta: [f64; 3],
}

impl TransverseMercator {
    /// Projection with central meridian `lon0_deg` and scale factor `k0`.
    pub fn new(lon0_deg: f64, k0: f64) -> Self {
        let f = WGS84_F;
        let n = f / (2.0 - f);
        let (n2, n3, n4) = (n * n, n * n * n, n * n * n * n);
        let a_hat = WGS84_A / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0);
        Self {
            lon0: lon0_deg.to_radians(),
            k0,
            a_hat,
            e: (f * (2.0 - f)).sqrt(),
            alpha: [
                n / 2.0 - 2.0 / 3.0 * n2 + 5.0 / 16.0 * n3,
                13.0 / 48.0 * n2 - 3.0 / 5.0 * n3,
                61.0 / 240.0 * n3,
            ],
            beta: [
                n / 2.0 - 2.0 / 3.0 * n2 + 37.0 / 96.0 * n3,
                1.0 / 48.0 * n2 + 1.0 / 15.0 * n3,
                17.0 / 480.0 * n3,
            ],
            delta: [
                2.0 * n - 2.0 / 3.0 * n2 - 2.0 * n3,
                7.0 / 3.0 * n2 - 8.0 / 5.0 * n3,
                56.0 / 15.0 * n3,
            ],
        }
    }

    /// Rectifying radius times π/2: the length of a quarter meridian.
    pub fn quarter_meridian(&self) -> f64 {
        self.a_hat * std::f64::consts::FRAC_PI_2
    }

    /// Geographic (degrees) → projected `(easting, northing)` in metres.
    pub fn forward(&self, lat_deg: f64, lon_deg: f64) -> (f64, f64) {
        let phi = lat_deg.to_radians();
        let lam = lon_deg.to_radians() - self.lon0;
        let s = phi.sin();
        let t = (s.atanh() - self.e * (self.e * s).atanh()).sinh();
        let xi_p = t.atan2(lam.cos());
        let eta_p = (lam.sin() / (1.0 + t * t).sqrt()).atanh();
        let mut xi = xi_p;
        let mut eta = eta_p;
        for (j, a) in self.alpha.iter().enumerate() {
            let k = 2.0 * (j + 1) as f64;
            xi += a * (k * xi_p).sin() * (k * eta_p).cosh();
            eta += a * (k * xi_p).cos() * (k * eta_p).sinh();
        }
        (self.k0 * self.a_hat * eta, self.k0 * self.a_hat * xi)
    }

    /// Projected `(easting, northing)` → geographic `(lat, lon)` in degrees.
    ///
    /// The series inverse is refined with Newton steps against
    /// [`forward`](Self::forward), so `forward(inverse(x)) == x` to well
    /// below a micrometre and repeated round trips do not drift.
    pub fn inverse(&self, easting: f64, northing: f64) -> (f64, f64) {
        let (mut lat, mut lon) = self.inverse_series(easting, northing);
        for _ in 0..4 {
            let (e0, n0) = self.forward(lat, lon);
            let (de, dn) = (easting - e0, northing - n0);
            if de.abs() < 1e-10 && dn.abs() < 1e-10 {
                break;
            }
            let h = 1e-6;
            let (e_lat, n_lat) = self.forward(lat + h, lon);
            let (e_lon, n_lon) = self.forward(lat, lon + h);
            let (a, b) = ((e_lat - e0) / h, (e_lon - e0) / h);
            let (c, d) = ((n_lat - n0) / h, (n_lon - n0) / h);
            let det = a * d - b * c;
            if det.abs() < 1e-12 {
                break;
            }
            lat += (d * de - b * dn) / det;
            lon += (a * dn - c * de) / det;
        }
        (lat, lon)
    }

    fn inverse_series(&self, easting: f64, northing: f64) -> (f64, f64) {
        let xi = northing / (self.k0 * self.a_hat);
        let eta = easting / (self.k0 * self.a_hat);
        let mut xi_p = xi;
        let mut eta_p = eta;
        for (j, b) in self.beta.iter().enumerate() {
            let k = 2.0 * (j + 1) as f64;
            xi_p -= b * (k * xi).sin() * (k * eta).cosh();
            eta_p -= b * (k * xi).cos() * (k * eta).sinh();
        }
        let chi = (xi_p.sin() / eta_p.cosh()).asin();
        let mut phi = chi;
        for (j, d) in self.delta.iter().enumerate() {
            let k = 2.0 * (j + 1) as f64;
            phi += d * (k * chi).sin();
        }
        let lam = eta_p.sinh().atan2(xi_p.cos());
        (phi.to_degrees(), (self.lon0 + lam).to_degrees())
    }
}

/// Standard UTM zone number of a position, including the Norway and
/// Svalbard exceptions.
pub fn utm_zone(lat: f64, lon: f64) -> u8 {
    let lon = (lon + 180.0).rem_euclid(360.0) - 180.0;
    let mut zone = ((lon + 180.0) / 6.0).floor() as i32 + 1;
    if (56.0..64.0).contains(&lat) && (3.0..12.0).contains(&lon) {
        zone = 32;
    }
    if (72.0..84.0).contains(&lat) {
        zone = match lon {
            l if (0.0..9.0).contains(&l) => 31,
            l if (9.0..21.0).contains(&l) => 33,
            l if (21.0..33.0).contains(&l) => 35,
            l if (33.0..42.0).contains(&l) => 37,
            _ => zone,
        };
    }
    zone.clamp(1, 60) as u8
}

/// Central meridian (degrees) of a UTM zone.
pub fn utm_central_meridian(zone: u8) -> f64 {
    f64::from(zone) * 6.0 - 183.0
}

/// The UTM-based MGRS 100 km grid identifier of a geographic position.
/// Returns `None` for non-finite coordinates or polar UPS positions.
pub fn mgrs_grid(p: GeoPoint) -> Option<String> {
    if !p.lat.is_finite()
        || !p.lon.is_finite()
        || !(-80.0..84.0).contains(&p.lat)
        || !(-180.0..=180.0).contains(&p.lon)
    {
        return None;
    }
    let zone = utm_zone(p.lat, p.lon);
    let tm = TransverseMercator::new(utm_central_meridian(zone), UTM_K0);
    let (e, n) = tm.forward(p.lat, p.lon);
    let column = ((e + 500_000.0) / 100_000.0).floor() as usize;
    if !(1..=8).contains(&column) {
        return None;
    }
    let northing = n + if p.lat < 0.0 { 10_000_000.0 } else { 0.0 };
    let row = (northing / 100_000.0).floor() as usize;
    let columns = [b"ABCDEFGH", b"JKLMNPQR", b"STUVWXYZ"];
    let rows = b"ABCDEFGHJKLMNPQRSTUV";
    let bands = b"CDEFGHJKLMNPQRSTUVWX";
    let band = (((p.lat + 80.0) / 8.0).floor() as usize).min(19);
    Some(format!(
        "{zone:02}{}{}{}",
        bands[band] as char,
        columns[usize::from((zone - 1) % 3)][column - 1] as char,
        rows[(row + if zone & 1 == 0 { 5 } else { 0 }) % 20] as char
    ))
}

/// Maps between WGS84 and a map's local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalProjector {
    tm: TransverseMercator,
    offset: (f64, f64),
    georeference: GeoReference,
}

impl LocalProjector {
    /// Creates the projector described by `georeference`.
    pub fn new(georeference: GeoReference) -> Self {
        let o = georeference.origin;
        let lon0 = match georeference.projection {
            ProjectionKind::Utm | ProjectionKind::Mgrs => {
                utm_central_meridian(utm_zone(o.lat, o.lon))
            }
            ProjectionKind::TransverseMercator => o.lon,
        };
        let tm = TransverseMercator::new(lon0, UTM_K0);
        let (e, n) = tm.forward(o.lat, o.lon);
        let offset = if georeference.projection == ProjectionKind::Mgrs {
            // UTM false easting and southern false northing are whole grid tiles.
            (
                (e + 500_000.0).div_euclid(100_000.0) * 100_000.0 - 500_000.0,
                n.div_euclid(100_000.0) * 100_000.0,
            )
        } else {
            (e, n)
        };
        Self {
            tm,
            offset,
            georeference,
        }
    }

    /// The georeference this projector implements.
    pub fn georeference(&self) -> GeoReference {
        self.georeference
    }

    /// Geographic → local.
    pub fn forward(&self, p: GeoPoint) -> Point3 {
        let (e, n) = self.tm.forward(p.lat, p.lon);
        Point3::new(e - self.offset.0, n - self.offset.1, p.alt)
    }

    /// Local → geographic.
    pub fn inverse(&self, p: Point3) -> GeoPoint {
        let (lat, lon) = self.tm.inverse(p.x + self.offset.0, p.y + self.offset.1);
        GeoPoint { lat, lon, alt: p.z }
    }
}

/// Rounds a recovered origin coordinate to 1e-9 degrees (~0.1 mm) so that
/// the small errors of lat/lon written with finite precision do not change
/// the origin from one round trip to the next.
fn snap(deg: f64) -> f64 {
    (deg * 1e9).round() / 1e9
}

/// Recovers the georeference of a map whose nodes carry both lat/lon and
/// local coordinates.
///
/// `pairs` are `(lat, lon, local_x, local_y)`. Two hypotheses are tested:
/// UTM relative to an origin (solved in closed form from the first node)
/// and transverse Mercator centred on the origin (solved with Newton's
/// method). A hypothesis is accepted if it reproduces the local coordinates
/// of every node within `tolerance` metres. For Autoware MGRS maps the
/// recovered UTM origin is the corner of the MGRS grid square.
pub fn recover_georeference(
    pairs: &[(f64, f64, f64, f64)],
    tolerance: f64,
) -> Option<GeoReference> {
    let &(lat0, lon0, x0, y0) = pairs.first()?;
    let consistent = |g: GeoReference| {
        let p = LocalProjector::new(g);
        pairs.iter().all(|&(lat, lon, x, y)| {
            let q = p.forward(GeoPoint::new(lat, lon));
            (q.x - x).abs() <= tolerance && (q.y - y).abs() <= tolerance
        })
    };

    // UTM: local = UTM(p) - UTM(origin).
    let tm = TransverseMercator::new(utm_central_meridian(utm_zone(lat0, lon0)), UTM_K0);
    let (e0, n0) = tm.forward(lat0, lon0);
    let (olat, olon) = tm.inverse(e0 - x0, n0 - y0);
    let utm = GeoReference {
        projection: ProjectionKind::Utm,
        origin: GeoPoint::new(snap(olat), snap(olon)),
    };
    // Only accept it if the origin lies in the same UTM zone (otherwise the
    // projector would use a different zone).
    if utm_zone(olat, olon) == utm_zone(lat0, lon0) && consistent(utm) {
        return Some(utm);
    }

    // Transverse Mercator centred on the origin: solve for (lat, lon).
    let residual = |lat: f64, lon: f64| {
        let tm = TransverseMercator::new(lon, UTM_K0);
        let (e, n) = tm.forward(lat0, lon0);
        let (eo, no) = tm.forward(lat, lon);
        (e - eo - x0, n - no - y0)
    };
    let (mut lat, mut lon) = (olat, olon);
    for _ in 0..30 {
        let (r1, r2) = residual(lat, lon);
        if r1.abs() < 1e-9 && r2.abs() < 1e-9 {
            break;
        }
        let h = 1e-7;
        let (a1, a2) = residual(lat + h, lon);
        let (b1, b2) = residual(lat, lon + h);
        let (j11, j12, j21, j22) = ((a1 - r1) / h, (b1 - r1) / h, (a2 - r2) / h, (b2 - r2) / h);
        let det = j11 * j22 - j12 * j21;
        if !det.is_finite() || det.abs() < 1e-12 {
            return None;
        }
        lat -= (j22 * r1 - j12 * r2) / det;
        lon -= (j11 * r2 - j21 * r1) / det;
    }
    let tm = GeoReference {
        projection: ProjectionKind::TransverseMercator,
        origin: GeoPoint::new(snap(lat), snap(lon)),
    };
    (lat.is_finite() && lon.is_finite() && consistent(tm)).then_some(tm)
}

#[cfg(test)]
mod tests {
    #[test]
    fn mgrs_matches_independent_utm_reference_values() {
        // WGS84 UTM coordinates from PROJ, reduced to the 100 km square.
        for (lat, lon, grid, x, y) in [
            (
                35.90204788913,
                139.93223216702,
                "54SVE",
                3643.9996002302,
                73610.756300225,
            ),
            (
                -33.8688,
                151.2093,
                "56HLH",
                34368.633648097,
                50948.345385009,
            ),
        ] {
            let origin = GeoPoint::new(lat, lon);
            assert_eq!(mgrs_grid(origin).as_deref(), Some(grid));
            let p = LocalProjector::new(GeoReference {
                projection: ProjectionKind::Mgrs,
                origin,
            });
            let q = p.forward(origin);
            assert!((q.x - x).abs() < 0.001 && (q.y - y).abs() < 0.001, "{q:?}");
            let back = p.inverse(Point3::new(x, y, 19.5));
            assert!((back.lat - lat).abs() < 1e-8 && (back.lon - lon).abs() < 1e-8);
            assert_eq!(back.alt, 19.5);
        }
        for lat in [-90.0, 84.0, f64::NAN] {
            assert!(mgrs_grid(GeoPoint::new(lat, 0.0)).is_none());
        }
    }

    #[test]
    fn georeference_recovery() {
        for projection in [ProjectionKind::Utm, ProjectionKind::TransverseMercator] {
            let g = GeoReference {
                projection,
                origin: GeoPoint::new(35.681236, 139.767125),
            };
            let p = LocalProjector::new(g);
            let pairs: Vec<_> = [(0.0, 0.0), (250.0, -40.0), (-120.0, 380.0), (900.0, 700.0)]
                .iter()
                .map(|&(x, y)| {
                    let q = p.inverse(Point3::new(x, y, 0.0));
                    (q.lat, q.lon, x, y)
                })
                .collect();
            let r = recover_georeference(&pairs, 1e-3).unwrap();
            assert_eq!(r.projection, projection);
            assert!((r.origin.lat - g.origin.lat).abs() < 1e-9);
            assert!((r.origin.lon - g.origin.lon).abs() < 1e-9);
        }
        // Inconsistent data is rejected.
        let bad = [(35.0, 139.0, 0.0, 0.0), (35.001, 139.0, 0.0, 0.0)];
        assert!(recover_georeference(&bad, 0.05).is_none());
    }

    use super::*;

    #[test]
    fn quarter_meridian_matches_wgs84() {
        // Known value: 10 001 965.729 m.
        let tm = TransverseMercator::new(0.0, 1.0);
        assert!((tm.quarter_meridian() - 10_001_965.729).abs() < 0.01);
    }

    #[test]
    fn central_meridian_is_scaled_meridian_arc() {
        let tm = TransverseMercator::new(3.0, UTM_K0);
        let (e, n) = tm.forward(0.0, 3.0);
        assert!(e.abs() < 1e-9 && n.abs() < 1e-9);
        // Meridian arc to 45°N: 4 984 944.378 m.
        let (e, n) = tm.forward(45.0, 3.0);
        assert!(e.abs() < 1e-6);
        assert!((n - UTM_K0 * 4_984_944.378).abs() < 0.01, "{n}");
    }

    #[test]
    fn forward_inverse_round_trip() {
        let tm = TransverseMercator::new(141.0, UTM_K0);
        for &(lat, lon) in &[
            (35.681236, 139.767125),
            (-33.9, 138.2),
            (60.0, 143.9),
            (0.5, 141.0),
        ] {
            let (e, n) = tm.forward(lat, lon);
            let (lat2, lon2) = tm.inverse(e, n);
            assert!(
                (lat - lat2).abs() < 1e-11 && (lon - lon2).abs() < 1e-11,
                "{lat} {lat2} {lon} {lon2}"
            );
        }
    }

    #[test]
    fn utm_zones() {
        assert_eq!(utm_zone(35.68, 139.77), 54);
        assert_eq!(utm_zone(49.0, 8.4), 32);
        assert_eq!(utm_zone(60.0, 5.0), 32); // Norway exception
        assert_eq!(utm_zone(78.0, 15.0), 33); // Svalbard exception
        assert_eq!(utm_zone(0.0, -180.0), 1);
        assert_eq!(utm_central_meridian(54), 141.0);
    }

    #[test]
    fn local_projector_is_relative_to_origin() {
        for projection in [ProjectionKind::Utm, ProjectionKind::TransverseMercator] {
            let origin = GeoPoint::new(35.681236, 139.767125);
            let p = LocalProjector::new(GeoReference { projection, origin });
            let o = p.forward(origin);
            assert!(o.x.abs() < 1e-9 && o.y.abs() < 1e-9);
            // ~111 m north.
            let north = p.forward(GeoPoint::new(35.682236, 139.767125));
            assert!((north.y - 110.9).abs() < 0.2, "{north:?}");
            let back = p.inverse(Point3::new(123.4, -56.7, 8.0));
            let again = p.forward(back);
            assert!((again.x - 123.4).abs() < 1e-6 && (again.y + 56.7).abs() < 1e-6);
            assert_eq!(again.z, 8.0);
        }
    }
}
