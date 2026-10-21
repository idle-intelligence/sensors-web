//! Speed and heading from the inertial sensors only (accelerometer, gyroscope, compass).
//! Every input may be NaN when the browser does not provide it; every output is NaN while unknown.
//!
//! Speed: stillness detection, accelerometer bias estimation, zero-velocity updates and a leaky
//! integrator, in the earth frame. Heading: a complementary filter of the gyro and the compass.

use wasm_bindgen::prelude::*;

/// Window over which stillness is judged (ms), and the least span of data needed to judge it.
const WINDOW_MS: f64 = 500.0;
const MIN_SPAN_MS: f64 = 400.0;
/// Still: variance of |a| and |w| over the window below these, and means below the caps.
const STILL_VAR_A: f64 = 0.02; // (m/s^2)^2
const STILL_VAR_W: f64 = 4.0; // (deg/s)^2
const STILL_MEAN_A: f64 = 0.5; // m/s^2
const STILL_MEAN_W: f64 = 10.0; // deg/s
/// Time constant of the running mean that estimates the accelerometer bias while still (s).
const BIAS_TAU: f64 = 2.0;
/// Time constant of the velocity leak (s): residual bias decays instead of growing without bound.
const LEAK_TAU: f64 = 5.0;
/// Time constant with which the heading is pulled toward the compass (s).
const COMPASS_TAU: f64 = 2.0;
/// Longest time step integrated in one reading (s); a longer gap is a pause, not motion.
const MAX_DT: f64 = 0.5;

const POINTS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

#[wasm_bindgen]
pub struct Calc {
    speed: f64,
    velocity: [f64; 3],
    bias: [f64; 3],
    attitude: Option<[f64; 3]>,
    last_motion: Option<f64>,
    window: Vec<(f64, f64, f64)>, // (t_ms, |a|, |w|; NaN when the gyro is missing)
    still: bool,
    compass: f64,
    heading: f64,
    compass_seen: bool,
    last_gyro: Option<f64>,
    last_compass: Option<f64>,
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

/// Signed shortest angular difference `to - from`, in (-180, 180].
fn angle_diff(to: f64, from: f64) -> f64 {
    let d = wrap(to - from);
    if d > 180.0 {
        d - 360.0
    } else {
        d
    }
}

fn mean_var(xs: &[f64]) -> (f64, f64) {
    let m = xs.iter().sum::<f64>() / xs.len() as f64;
    (m, xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / xs.len() as f64)
}

#[wasm_bindgen]
impl Calc {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Calc {
        Calc {
            speed: f64::NAN,
            velocity: [0.0; 3],
            bias: [0.0; 3],
            attitude: None,
            last_motion: None,
            window: Vec::new(),
            still: false,
            compass: f64::NAN,
            heading: f64::NAN,
            compass_seen: false,
            last_gyro: None,
            last_compass: None,
        }
    }

    /// One devicemotion reading: time in ms, acceleration without gravity (m/s^2) and the
    /// rotation rates (deg/s): `rot_x` = beta, `rot_y` = gamma, `rot_z` = alpha of rotationRate.
    #[allow(clippy::too_many_arguments)]
    pub fn push_motion(&mut self, t_ms: f64, ax: f64, ay: f64, az: f64, rot_x: f64, rot_y: f64, rot_z: f64) {
        if ax.is_finite() && ay.is_finite() && az.is_finite() {
            self.step_speed(t_ms, [ax, ay, az], [rot_x, rot_y, rot_z]);
        }
        if rot_z.is_finite() {
            self.step_heading(t_ms, [rot_x, rot_y, rot_z]);
        }
    }

    fn step_speed(&mut self, t: f64, a: [f64; 3], w: [f64; 3]) {
        let dt = self.last_motion.map_or(0.0, |p| ((t - p) / 1000.0).clamp(0.0, MAX_DT));
        self.last_motion = Some(t);
        let amag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        let wmag = if w.iter().all(|x| x.is_finite()) {
            (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt()
        } else if w[2].is_finite() {
            w[2].abs()
        } else {
            f64::NAN
        };
        self.window.push((t, amag, wmag));
        self.window.retain(|s| t - s.0 <= WINDOW_MS);

        self.still = self.is_still();
        if self.still {
            // Zero-velocity update, and learn the bias: the acceleration while still is the sensor's offset.
            self.velocity = [0.0; 3];
            let k = 1.0 - (-dt / BIAS_TAU).exp();
            for (b, a) in self.bias.iter_mut().zip(a) {
                *b += (a - *b) * k;
            }
        } else {
            let c = [a[0] - self.bias[0], a[1] - self.bias[1], a[2] - self.bias[2]];
            let c = match self.attitude {
                Some(att) => to_world(att, c),
                None => c,
            };
            let leak = (-dt / LEAK_TAU).exp();
            for (v, c) in self.velocity.iter_mut().zip(c) {
                *v = *v * leak + c * dt;
            }
        }
        let [vx, vy, vz] = self.velocity;
        // With an orientation the vertical axis is known and left out; without one the full vector is reported.
        self.speed = if self.attitude.is_some() { vx.hypot(vy) } else { (vx * vx + vy * vy + vz * vz).sqrt() };
    }

    /// Still when, over the last half second, both |a| and |w| barely vary and are small.
    fn is_still(&self) -> bool {
        let (Some(first), Some(last)) = (self.window.first(), self.window.last()) else {
            return false;
        };
        if last.0 - first.0 < MIN_SPAN_MS {
            return false;
        }
        let a: Vec<f64> = self.window.iter().map(|s| s.1).collect();
        let (ma, va) = mean_var(&a);
        if ma > STILL_MEAN_A || va > STILL_VAR_A {
            return false;
        }
        let w: Vec<f64> = self.window.iter().map(|s| s.2).filter(|x| x.is_finite()).collect();
        if w.is_empty() {
            return true;
        }
        let (mw, vw) = mean_var(&w);
        mw <= STILL_MEAN_W && vw <= STILL_VAR_W
    }

    /// Short-term heading: integrate the earth-frame vertical rotation rate (counter-clockwise positive).
    fn step_heading(&mut self, t: f64, w: [f64; 3]) {
        let rate = match self.attitude {
            Some(att) if w.iter().all(|x| x.is_finite()) => {
                let b = att[1].to_radians();
                let g = att[2].to_radians();
                -b.cos() * g.sin() * w[0] + b.sin() * w[1] + b.cos() * g.cos() * w[2]
            }
            _ => w[2],
        };
        match self.last_gyro {
            Some(p) if self.heading.is_finite() => {
                let dt = ((t - p) / 1000.0).clamp(0.0, MAX_DT);
                self.heading = wrap(self.heading - rate * dt);
            }
            _ if !self.heading.is_finite() => self.heading = 0.0,
            _ => {}
        }
        self.last_gyro = Some(t);
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

    /// One compass reading at time `t_ms`: `compass` is webkitCompassHeading (deg clockwise from north),
    /// `absolute_alpha` is the alpha of deviceorientationabsolute (deg counter-clockwise from north).
    /// The heading is pulled toward it along the shortest way round.
    pub fn push_orientation(&mut self, t_ms: f64, compass: f64, absolute_alpha: f64) {
        let target = if compass.is_finite() {
            wrap(compass)
        } else if absolute_alpha.is_finite() {
            wrap(360.0 - absolute_alpha)
        } else {
            return;
        };
        self.compass = target;
        match (self.compass_seen, self.last_compass) {
            (true, Some(p)) if self.heading.is_finite() => {
                let dt = ((t_ms - p) / 1000.0).max(0.0);
                let k = 1.0 - (-dt / COMPASS_TAU).exp();
                self.heading = wrap(self.heading + angle_diff(target, self.heading) * k);
            }
            _ => self.heading = target,
        }
        self.compass_seen = true;
        self.last_compass = Some(t_ms);
    }

    /// Speed in m/s, NaN until an acceleration reading arrived.
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Heading in degrees, 0 to 360, NaN until a compass or gyro reading arrived.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// Whether the last acceleration reading was judged still (the moments the speed is zeroed).
    pub fn still(&self) -> bool {
        self.still
    }

    /// The last raw compass heading in degrees, 0 to 360, NaN until one arrived.
    pub fn compass(&self) -> f64 {
        self.compass
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

    const N: f64 = f64::NAN;

    /// Feeds `secs` seconds of the same acceleration at 20 Hz starting at `t0`; returns the end time.
    fn feed(c: &mut Calc, t0: f64, secs: f64, a: [f64; 3]) -> f64 {
        let steps = (secs * 20.0) as usize;
        for i in 1..=steps {
            c.push_motion(t0 + i as f64 * 50.0, a[0], a[1], a[2], 0.0, 0.0, 0.0);
        }
        t0 + steps as f64 * 50.0
    }

    #[test]
    fn unknown_is_nan() {
        let c = Calc::new();
        assert!(c.speed().is_nan() && c.heading().is_nan());
        assert_eq!(c.heading_letter(), "");
    }

    #[test]
    fn integrates_motion() {
        let mut c = Calc::new();
        c.push_motion(0.0, 0.0, 0.0, 0.0, N, N, N);
        feed(&mut c, 0.0, 0.5, [2.0, 0.0, 0.0]);
        assert!((c.speed() - 1.0).abs() < 0.05, "{}", c.speed());
    }

    #[test]
    fn symmetric_shake_returns_near_zero() {
        for att in [None, Some([30.0, 20.0, 10.0])] {
            let mut c = Calc::new();
            if let Some(a) = att {
                c.push_attitude(a[0], a[1], a[2]);
            }
            c.push_motion(0.0, 0.0, 0.0, 0.0, N, N, N);
            let t = feed(&mut c, 0.0, 0.5, [2.0, 1.0, 0.5]);
            assert!(c.speed() > 0.5);
            feed(&mut c, t, 0.5, [-2.0, -1.0, -0.5]);
            assert!(c.speed() < 0.15, "{}", c.speed());
        }
    }

    #[test]
    fn world_frame_horizontal_speed() {
        let mut c = Calc::new();
        c.push_attitude(90.0, 0.0, 0.0);
        c.push_motion(0.0, 0.0, 0.0, 0.0, N, N, N);
        feed(&mut c, 0.0, 0.5, [2.0, 0.0, 0.0]);
        assert!((c.speed() - 1.0).abs() < 0.05);
        let mut c = Calc::new();
        c.push_attitude(0.0, 90.0, 0.0);
        c.push_motion(0.0, 0.0, 0.0, 0.0, N, N, N);
        feed(&mut c, 0.0, 0.5, [0.0, 2.0, 0.0]);
        assert!(c.speed() < 1e-9);
    }

    #[test]
    fn zupt_zeroes_velocity_when_still() {
        let mut c = Calc::new();
        c.push_motion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let t = feed(&mut c, 0.0, 1.0, [3.0, 0.0, 0.0]);
        assert!(c.speed() > 1.0);
        feed(&mut c, t, 1.0, [0.0, 0.0, 0.0]);
        assert_eq!(c.speed(), 0.0);
    }

    #[test]
    fn rotation_alone_is_not_still() {
        let mut c = Calc::new();
        for i in 0..40 {
            let w = if i % 2 == 0 { 0.0 } else { 40.0 };
            c.push_motion(i as f64 * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, w);
        }
        assert!(!c.is_still());
    }

    #[test]
    fn bias_is_learned_while_still_and_removed_after() {
        let mut c = Calc::new();
        c.push_motion(0.0, 0.2, -0.1, 0.1, 0.0, 0.0, 0.0);
        let t = feed(&mut c, 0.0, 8.0, [0.2, -0.1, 0.1]);
        assert_eq!(c.speed(), 0.0);
        for (b, e) in c.bias.iter().zip([0.2, -0.1, 0.1]) {
            assert!((b - e).abs() < 0.01, "{:?}", c.bias);
        }
        // A real push on top of the same offset: only the push integrates.
        feed(&mut c, t, 0.5, [2.2, -0.1, 0.1]);
        assert!((c.speed() - 1.0).abs() < 0.1, "{}", c.speed());
    }

    #[test]
    fn leak_bounds_drift_under_constant_bias() {
        // A constant 0.6 m/s^2 offset is above the still cap, so it integrates, but the leak caps it at bias * tau.
        let mut c = Calc::new();
        c.push_motion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        feed(&mut c, 0.0, 60.0, [0.6, 0.0, 0.0]);
        assert!(c.speed() < 0.6 * LEAK_TAU * 1.05, "{}", c.speed());
        assert!(c.speed() > 0.6 * LEAK_TAU * 0.9, "{}", c.speed());
    }

    #[test]
    fn heading_converges_to_compass() {
        let mut c = Calc::new();
        c.push_orientation(0.0, 0.0, N);
        for i in 1..=200 {
            c.push_orientation(i as f64 * 50.0, 90.0, N);
        }
        assert!((c.heading() - 90.0).abs() < 2.0, "{}", c.heading());
    }

    #[test]
    fn heading_smooths_a_noisy_compass() {
        let mut c = Calc::new();
        c.push_orientation(0.0, 100.0, N);
        let mut worst: f64 = 0.0;
        for i in 1..=200 {
            let noisy = if i % 2 == 0 { 110.0 } else { 90.0 };
            c.push_orientation(i as f64 * 50.0, noisy, N);
            worst = worst.max((c.heading() - 100.0).abs());
        }
        assert!(worst < 1.0, "{worst}");
    }

    #[test]
    fn heading_wraps_the_short_way() {
        let mut c = Calc::new();
        c.push_orientation(0.0, 350.0, N);
        for i in 1..=100 {
            c.push_orientation(i as f64 * 50.0, 10.0, N);
            let h = c.heading();
            assert!(h >= 350.0 || h <= 10.5, "went the long way: {h}");
        }
        assert!(c.heading() < 11.0 || c.heading() > 359.0);
        let mut c = Calc::new();
        c.push_orientation(0.0, N, 10.0);
        assert!((c.heading() - 350.0).abs() < 1e-9);
        c.push_orientation(100_000.0, 725.0, N);
        assert!((c.heading() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn gyro_smooths_and_wraps_without_compass() {
        let mut c = Calc::new();
        c.push_motion(0.0, N, N, N, 0.0, 0.0, 10.0);
        assert_eq!(c.heading(), 0.0);
        c.push_motion(500.0, N, N, N, 0.0, 0.0, 10.0);
        assert!((c.heading() - 355.0).abs() < 1e-9);
    }

    #[test]
    fn gyro_carries_heading_between_compass_readings() {
        let mut c = Calc::new();
        c.push_orientation(0.0, 90.0, N);
        c.push_motion(0.0, N, N, N, 0.0, 0.0, 0.0);
        c.push_motion(500.0, N, N, N, 0.0, 0.0, -20.0);
        assert!((c.heading() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn exposes_still_and_raw_compass() {
        let mut c = Calc::new();
        assert!(!c.still() && c.compass().is_nan());
        c.push_motion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        feed(&mut c, 0.0, 1.0, [0.0, 0.0, 0.0]);
        assert!(c.still());
        feed(&mut c, 1000.0, 0.2, [3.0, 0.0, 0.0]);
        assert!(!c.still());
        c.push_orientation(0.0, 725.0, N);
        assert!((c.compass() - 5.0).abs() < 1e-9);
        c.push_orientation(50.0, 100.0, N);
        assert!((c.compass() - 100.0).abs() < 1e-9);
        assert!(c.heading() < 10.0, "filtered heading lags the raw compass");
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
