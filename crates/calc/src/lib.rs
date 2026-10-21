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

fn wrap(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

#[wasm_bindgen]
impl Calc {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Calc {
        Calc {
            speed: f64::NAN,
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
            if self.speed.is_nan() {
                self.speed = 0.0;
            }
            if mag < STILL_ACCEL {
                let since = *self.still_since.get_or_insert(t_ms);
                if t_ms - since >= STILL_MS {
                    self.speed = 0.0;
                }
            } else {
                self.still_since = None;
                self.speed += mag * dt;
            }
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
