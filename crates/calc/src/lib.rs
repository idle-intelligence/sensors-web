//! Speed and heading derived from raw device readings.
//! Every input may be NaN when the browser does not provide it; every output is NaN while unknown.

use wasm_bindgen::prelude::*;

/// Acceleration magnitude (m/s^2) below which the device counts as still.
const STILL_ACCEL: f64 = 0.3;
/// How long the device must stay still before the speed is reset (ms).
const STILL_MS: f64 = 1000.0;
/// Longest time step integrated in one reading (s); a longer gap is a pause, not motion.
const MAX_DT: f64 = 0.5;

const POINTS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

#[wasm_bindgen]
pub struct Calc {
    speed: f64,
    velocity: [f64; 3],
    attitude: Option<[f64; 3]>,
    last_motion: Option<f64>,
    still_since: Option<f64>,
    heading: f64,
    compass_seen: bool,
    last_gyro: Option<f64>,
}

impl Default for Calc {
    fn default() -> Self {
        Self::new()
    }
}

/// Rotates a device-frame vector into the earth frame (x east, y north, z up)
/// from alpha, beta, gamma in degrees (Z-X'-Y'' rotation order).
fn to_world(att: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let (a, b, g) = (att[0].to_radians(), att[1].to_radians(), att[2].to_radians());
    let (ca, sa, cb, sb, cg, sg) = (a.cos(), a.sin(), b.cos(), b.sin(), g.cos(), g.sin());
    [
        (ca * cg - sa * sb * sg) * v[0] - sa * cb * v[1] + (ca * sg + sa * sb * cg) * v[2],
        (sa * cg + ca * sb * sg) * v[0] + ca * cb * v[1] + (sa * sg - ca * sb * cg) * v[2],
        -cb * sg * v[0] + sb * v[1] + cb * cg * v[2],
    ]
}

fn wrap(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

#[wasm_bindgen]
impl Calc {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Calc {
        Calc {
            speed: f64::NAN,
            velocity: [0.0; 3],
            attitude: None,
            last_motion: None,
            still_since: None,
            heading: f64::NAN,
            compass_seen: false,
            last_gyro: None,
        }
    }

    /// One devicemotion reading: time in ms, acceleration without gravity (m/s^2),
    /// and the rotation rate about z (deg/s, counter-clockwise positive).
    pub fn push_motion(&mut self, t_ms: f64, ax: f64, ay: f64, az: f64, rot_z: f64) {
        if ax.is_finite() && ay.is_finite() && az.is_finite() {
            let dt = self.last_motion.map_or(0.0, |p| ((t_ms - p) / 1000.0).clamp(0.0, MAX_DT));
            self.last_motion = Some(t_ms);
            let mag = (ax * ax + ay * ay + az * az).sqrt();
            if mag < STILL_ACCEL {
                let since = *self.still_since.get_or_insert(t_ms);
                if t_ms - since >= STILL_MS {
                    self.velocity = [0.0; 3];
                }
            } else {
                self.still_since = None;
                let a = match self.attitude {
                    Some(att) => to_world(att, [ax, ay, az]),
                    None => [ax, ay, az],
                };
                for (v, a) in self.velocity.iter_mut().zip(a) {
                    *v += a * dt;
                }
            }
            let [vx, vy, vz] = self.velocity;
            // With an orientation the vertical axis is known and left out; without one the full vector is reported.
            self.speed = if self.attitude.is_some() { vx.hypot(vy) } else { (vx * vx + vy * vy + vz * vz).sqrt() };
        }
        if !self.compass_seen && rot_z.is_finite() {
            if let Some(p) = self.last_gyro {
                let dt = ((t_ms - p) / 1000.0).clamp(0.0, MAX_DT);
                self.heading = wrap(self.heading - rot_z * dt);
            } else {
                self.heading = 0.0;
            }
            self.last_gyro = Some(t_ms);
        }
    }

    /// One deviceorientation reading in degrees; used to express acceleration in the earth frame.
    pub fn push_attitude(&mut self, alpha: f64, beta: f64, gamma: f64) {
        if alpha.is_finite() && beta.is_finite() && gamma.is_finite() {
            if self.attitude.is_none() {
                self.velocity = [0.0; 3];
            }
            self.attitude = Some([alpha, beta, gamma]);
        }
    }

    /// One orientation reading: `compass` is webkitCompassHeading (deg clockwise from north),
    /// `absolute_alpha` is the alpha of deviceorientationabsolute (deg counter-clockwise from north).
    pub fn push_orientation(&mut self, compass: f64, absolute_alpha: f64) {
        if compass.is_finite() {
            self.compass_seen = true;
            self.heading = wrap(compass);
        } else if absolute_alpha.is_finite() {
            self.compass_seen = true;
            self.heading = wrap(360.0 - absolute_alpha);
        }
    }

    /// Speed in m/s, NaN until an acceleration reading arrived.
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Heading in degrees, 0 to 360, NaN until a compass or gyro reading arrived.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// Compass letter for the heading, empty while unknown.
    pub fn heading_letter(&self) -> String {
        letter(self.heading).to_string()
    }
}

pub fn letter(deg: f64) -> &'static str {
    if !deg.is_finite() {
        return "";
    }
    POINTS[(wrap(deg) / 45.0).round() as usize % 8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_is_nan() {
        let c = Calc::new();
        assert!(c.speed().is_nan() && c.heading().is_nan());
        assert_eq!(c.heading_letter(), "");
    }

    #[test]
    fn integrates_acceleration() {
        let mut c = Calc::new();
        for i in 0..11 {
            c.push_motion(i as f64 * 100.0, 2.0, 0.0, 0.0, f64::NAN);
        }
        assert!((c.speed() - 2.0).abs() < 1e-9);
    }

    fn back_and_forth(c: &mut Calc) {
        c.push_motion(0.0, 0.0, 0.0, 0.0, f64::NAN);
        for i in 1..=5 {
            c.push_motion(i as f64 * 100.0, 2.0, 1.0, 0.5, f64::NAN);
        }
        assert!(c.speed() > 1.0);
        for i in 6..=10 {
            c.push_motion(i as f64 * 100.0, -2.0, -1.0, -0.5, f64::NAN);
        }
    }

    #[test]
    fn symmetric_shake_returns_to_zero() {
        let mut c = Calc::new();
        back_and_forth(&mut c);
        assert!(c.speed().abs() < 1e-9);
        let mut c = Calc::new();
        c.push_attitude(30.0, 20.0, 10.0);
        back_and_forth(&mut c);
        assert!(c.speed().abs() < 1e-9);
    }

    #[test]
    fn world_frame_horizontal_speed() {
        // Device flat, alpha 90: device x acceleration points north; speed is the same as unrotated.
        let mut c = Calc::new();
        c.push_attitude(90.0, 0.0, 0.0);
        c.push_motion(0.0, 0.0, 0.0, 0.0, f64::NAN);
        c.push_motion(500.0, 2.0, 0.0, 0.0, f64::NAN);
        assert!((c.speed() - 1.0).abs() < 1e-9);
        // Device standing on its end (beta 90): device y acceleration is vertical, so no horizontal speed.
        let mut c = Calc::new();
        c.push_attitude(0.0, 90.0, 0.0);
        c.push_motion(0.0, 0.0, 0.0, 0.0, f64::NAN);
        c.push_motion(500.0, 0.0, 2.0, 0.0, f64::NAN);
        assert!(c.speed().abs() < 1e-9);
    }

    #[test]
    fn still_resets_after_a_second() {
        let mut c = Calc::new();
        for i in 0..6 {
            c.push_motion(i as f64 * 100.0, 3.0, 0.0, 0.0, f64::NAN);
        }
        assert!(c.speed() > 1.0);
        for i in 6..16 {
            c.push_motion(i as f64 * 100.0, 0.0, 0.1, 0.0, f64::NAN);
        }
        assert!(c.speed() > 1.0, "not yet a full second of rest");
        c.push_motion(1700.0, 0.0, 0.0, 0.0, f64::NAN);
        assert_eq!(c.speed(), 0.0);
    }

    #[test]
    fn heading_wraps() {
        let mut c = Calc::new();
        c.push_orientation(f64::NAN, 10.0);
        assert!((c.heading() - 350.0).abs() < 1e-9);
        c.push_orientation(725.0, f64::NAN);
        assert!((c.heading() - 5.0).abs() < 1e-9);
        c.push_orientation(f64::NAN, 360.0);
        assert!(c.heading().abs() < 1e-9);
    }

    #[test]
    fn gyro_fallback_starts_at_zero_and_wraps() {
        let mut c = Calc::new();
        c.push_motion(0.0, f64::NAN, f64::NAN, f64::NAN, 10.0);
        assert_eq!(c.heading(), 0.0);
        c.push_motion(500.0, f64::NAN, f64::NAN, f64::NAN, 10.0);
        assert!((c.heading() - 355.0).abs() < 1e-9);
        c.push_orientation(90.0, f64::NAN);
        c.push_motion(1000.0, f64::NAN, f64::NAN, f64::NAN, 10.0);
        assert_eq!(c.heading(), 90.0);
    }

    #[test]
    fn letters() {
        assert_eq!(letter(0.0), "N");
        assert_eq!(letter(359.0), "N");
        assert_eq!(letter(44.0), "NE");
        assert_eq!(letter(180.0), "S");
        assert_eq!(letter(300.0), "NW");
    }
}
