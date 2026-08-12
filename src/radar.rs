//! RADAR — 77GHz Automotive Radar simulation (Bosch SRR2+ / Continental ARS540 style).
//! Models: front long-range (250m), rear short-range (80m), corner radars (120m).
//! Implements: target detection, Kalman tracking, Doppler velocity, RCS model.

use std::f64::consts::PI;

// ── Radar sensor position ──────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RadarPosition {
    FrontCenter,
    RearCenter,
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

impl std::fmt::Display for RadarPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadarPosition::FrontCenter => write!(f, "F-CTR"),
            RadarPosition::RearCenter => write!(f, "R-CTR"),
            RadarPosition::FrontLeft => write!(f, "F-LFT"),
            RadarPosition::FrontRight => write!(f, "F-RGT"),
            RadarPosition::RearLeft => write!(f, "R-LFT"),
            RadarPosition::RearRight => write!(f, "R-RGT"),
        }
    }
}

// ── Object type classification ─────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RadarObjectClass {
    Car,
    Truck,
    Motorcycle,
    Pedestrian,
    Bicycle,
    Stationary,
    Unknown,
}

impl std::fmt::Display for RadarObjectClass {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadarObjectClass::Car => write!(f, "Car        "),
            RadarObjectClass::Truck => write!(f, "Truck      "),
            RadarObjectClass::Motorcycle => write!(f, "Motorcycle "),
            RadarObjectClass::Pedestrian => write!(f, "Pedestrian "),
            RadarObjectClass::Bicycle => write!(f, "Bicycle    "),
            RadarObjectClass::Stationary => write!(f, "Stationary "),
            RadarObjectClass::Unknown => write!(f, "Unknown    "),
        }
    }
}

// ── Tracked target ────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct RadarTarget {
    pub id: u8,
    pub range_m: f64,       // slant range to target
    pub azimuth_deg: f64,   // bearing (left=-45..right=+45)
    pub elevation_deg: f64, // elevation angle
    pub range_rate_ms: f64, // radial velocity (negative = approaching)
    pub lat_vel_ms: f64,    // lateral velocity estimate
    pub rcs_dbsm: f64,      // Radar Cross Section (car ~10dBsm, truck ~25dBsm)
    pub confidence: f64,    // 0-1 detection confidence
    pub object_class: RadarObjectClass,
    pub tracked_cycles: u32, // how many cycles this target has been tracked
    pub ttc_s: f64,          // Time-To-Collision (seconds, ∞ if receding)

    // Kalman state [range, range_rate, lateral_pos, lateral_vel]
    kf_x: [f64; 4],
    kf_p: [f64; 4], // diagonal covariance
}

impl RadarTarget {
    fn new(id: u8, range: f64, azimuth: f64, range_rate: f64, rcs: f64) -> Self {
        let cls = if rcs > 20.0 {
            RadarObjectClass::Truck
        } else if rcs > 5.0 {
            RadarObjectClass::Car
        } else if rcs > -5.0 {
            RadarObjectClass::Motorcycle
        } else {
            RadarObjectClass::Pedestrian
        };
        let ttc = if range_rate < -0.1 {
            -range / range_rate
        } else {
            f64::INFINITY
        };
        RadarTarget {
            id,
            range_m: range,
            azimuth_deg: azimuth,
            elevation_deg: 0.0,
            range_rate_ms: range_rate,
            lat_vel_ms: 0.0,
            rcs_dbsm: rcs,
            confidence: 0.9,
            object_class: cls,
            tracked_cycles: 1,
            ttc_s: ttc,
            kf_x: [range, range_rate, azimuth * range * PI / 180.0, 0.0],
            kf_p: [5.0, 1.0, 2.0, 0.5],
        }
    }

    fn kalman_predict(&mut self, dt: f64) {
        // State transition: range += range_rate*dt, lateral_pos += lat_vel*dt
        self.kf_x[0] += self.kf_x[1] * dt;
        self.kf_x[2] += self.kf_x[3] * dt;
        // Process noise
        self.kf_p[0] += self.kf_p[1] * dt + 0.1 * dt * dt;
        self.kf_p[1] += 0.5 * dt;
        self.kf_p[2] += self.kf_p[3] * dt + 0.1 * dt * dt;
        self.kf_p[3] += 0.3 * dt;
    }

    fn kalman_update(&mut self, z_range: f64, z_rate: f64, z_azimuth: f64) {
        let r_cov = 1.0; // measurement noise
        let k0 = self.kf_p[0] / (self.kf_p[0] + r_cov);
        let k1 = self.kf_p[1] / (self.kf_p[1] + r_cov);
        self.kf_x[0] += k0 * (z_range - self.kf_x[0]);
        self.kf_x[1] += k1 * (z_rate - self.kf_x[1]);
        self.kf_p[0] *= 1.0 - k0;
        self.kf_p[1] *= 1.0 - k1;
        // Update derived quantities
        self.range_m = self.kf_x[0].max(0.0);
        self.range_rate_ms = self.kf_x[1];
        self.azimuth_deg = z_azimuth;
        self.ttc_s = if self.range_rate_ms < -0.1 {
            -self.range_m / self.range_rate_ms
        } else {
            f64::INFINITY
        };
    }
}

// ── Single radar sensor ────────────────────────────────────────────────────────
pub struct RadarSensor {
    pub position: RadarPosition,
    pub targets: Vec<RadarTarget>,
    pub max_range_m: f64,
    pub fov_deg: f64, // horizontal FOV
    pub update_rate_hz: f64,
    pub active: bool,
    next_id: u8,
    update_timer: f64,
    noise_t: f64,
}

impl RadarSensor {
    pub fn new(pos: RadarPosition, max_range: f64, fov: f64) -> Self {
        RadarSensor {
            position: pos,
            targets: Vec::new(),
            max_range_m: max_range,
            fov_deg: fov,
            update_rate_hz: 50.0,
            active: true,
            next_id: 0,
            update_timer: 0.0,
            noise_t: 0.0,
        }
    }

    pub fn update(&mut self, ego_speed_ms: f64, traffic_objects: &[SimTrafficObj], dt: f64) {
        self.noise_t += dt;
        self.update_timer += dt;
        if self.update_timer < 1.0 / self.update_rate_hz {
            return;
        }
        self.update_timer = 0.0;

        let n = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        let nt = self.noise_t;

        // ── Kalman predict all existing targets ───────────────────────────────
        for t in &mut self.targets {
            t.kalman_predict(1.0 / self.update_rate_hz);
        }

        // ── Generate detections from traffic simulation ───────────────────────
        let mut updated_ids: Vec<u8> = Vec::new();

        for obj in traffic_objects {
            let (r, az, rr) = self.compute_detection(obj, ego_speed_ms);
            if r > self.max_range_m || az.abs() > self.fov_deg / 2.0 {
                continue;
            }

            // Detection probability based on RCS and range
            let snr_db = obj.rcs_dbsm - 20.0 * (r / 100.0).log10().max(0.0) * 4.0;
            if snr_db < 5.0 {
                continue;
            } // Below detection threshold

            // Add measurement noise
            let r_noisy = r + n(nt * 1.1 + obj.id as f64) * (0.1 + r * 0.005);
            let az_noisy = az + n(nt * 2.3 + obj.id as f64) * 0.3;
            let rr_noisy = rr + n(nt * 3.7 + obj.id as f64) * 0.15;

            // Find existing target or create new
            if let Some(t) = self
                .targets
                .iter_mut()
                .find(|t| (t.range_m - r).abs() < 5.0 && (t.azimuth_deg - az).abs() < 5.0)
            {
                t.kalman_update(r_noisy, rr_noisy, az_noisy);
                t.tracked_cycles += 1;
                t.confidence = (t.confidence + 0.05).min(1.0);
                updated_ids.push(t.id);
            } else {
                let id = self.next_id;
                self.next_id = self.next_id.wrapping_add(1);
                self.targets.push(RadarTarget::new(
                    id,
                    r_noisy,
                    az_noisy,
                    rr_noisy,
                    obj.rcs_dbsm,
                ));
                updated_ids.push(id);
            }
        }

        // ── Remove stale targets ──────────────────────────────────────────────
        self.targets.retain(|t| {
            updated_ids.contains(&t.id) || {
                // Decay confidence for unupdated targets
                let conf = t.confidence;
                // Can't mutate while in retain... filter by threshold
                conf > 0.3 && t.range_m > 0.0 && t.range_m < self.max_range_m
            }
        });
        for t in &mut self.targets {
            if !updated_ids.contains(&t.id) {
                t.confidence -= 0.1;
            }
        }
        self.targets.truncate(64); // Hard limit: 64 objects per sensor
    }

    fn compute_detection(&self, obj: &SimTrafficObj, ego_speed_ms: f64) -> (f64, f64, f64) {
        let range = obj.distance_m;
        let azimuth = obj.lateral_offset_m.atan2(range) * 180.0 / PI;
        let range_rate = ego_speed_ms - obj.speed_ms; // positive = ego closing
        (range, azimuth, -range_rate) // negative = approaching
    }

    /// Closest in-path target
    pub fn closest_inpath(&self) -> Option<&RadarTarget> {
        self.targets
            .iter()
            .filter(|t| t.azimuth_deg.abs() < 5.0 && t.range_m > 0.5)
            .min_by(|a, b| a.range_m.partial_cmp(&b.range_m).unwrap())
    }
}

// ── Traffic object for radar detection ────────────────────────────────────────
pub struct SimTrafficObj {
    pub id: u8,
    pub distance_m: f64,       // longitudinal distance ahead
    pub lateral_offset_m: f64, // lateral offset from ego path
    pub speed_ms: f64,
    pub rcs_dbsm: f64,
    pub object_type: RadarObjectClass,
}

// ── Full radar suite ──────────────────────────────────────────────────────────
pub struct RadarSuite {
    pub front_center: RadarSensor, // 250m, ±9° (ACC, AEB)
    pub rear_center: RadarSensor,  // 80m, ±20° (rear traffic)
    pub front_left: RadarSensor,   // 120m, ±75° (BSM, junction)
    pub front_right: RadarSensor,
    pub rear_left: RadarSensor, // 120m, ±75° (lane change)
    pub rear_right: RadarSensor,

    // Aggregated metrics
    pub ttc_front: f64, // Time to collision front
    pub closest_front_m: f64,
    pub bsm_left: bool,
    pub bsm_right: bool,
    pub cross_traffic_left: bool,
    pub cross_traffic_right: bool,
}

impl RadarSuite {
    pub fn new() -> Self {
        RadarSuite {
            front_center: RadarSensor::new(RadarPosition::FrontCenter, 250.0, 18.0),
            rear_center: RadarSensor::new(RadarPosition::RearCenter, 80.0, 40.0),
            front_left: RadarSensor::new(RadarPosition::FrontLeft, 120.0, 150.0),
            front_right: RadarSensor::new(RadarPosition::FrontRight, 120.0, 150.0),
            rear_left: RadarSensor::new(RadarPosition::RearLeft, 120.0, 150.0),
            rear_right: RadarSensor::new(RadarPosition::RearRight, 120.0, 150.0),
            ttc_front: f64::INFINITY,
            closest_front_m: f64::INFINITY,
            bsm_left: false,
            bsm_right: false,
            cross_traffic_left: false,
            cross_traffic_right: false,
        }
    }

    pub fn update(&mut self, ego_speed_ms: f64, traffic: &[SimTrafficObj], dt: f64) {
        self.front_center.update(ego_speed_ms, traffic, dt);
        self.rear_center.update(ego_speed_ms, traffic, dt);
        self.front_left.update(ego_speed_ms, traffic, dt);
        self.front_right.update(ego_speed_ms, traffic, dt);
        self.rear_left.update(ego_speed_ms, traffic, dt);
        self.rear_right.update(ego_speed_ms, traffic, dt);

        // Aggregate
        if let Some(t) = self.front_center.closest_inpath() {
            self.ttc_front = t.ttc_s;
            self.closest_front_m = t.range_m;
        } else {
            self.ttc_front = f64::INFINITY;
            self.closest_front_m = f64::INFINITY;
        }
        self.bsm_left = self
            .rear_left
            .targets
            .iter()
            .any(|t| t.range_m < 80.0 && t.azimuth_deg.abs() < 45.0);
        self.bsm_right = self
            .rear_right
            .targets
            .iter()
            .any(|t| t.range_m < 80.0 && t.azimuth_deg.abs() < 45.0);
        self.cross_traffic_left = self.rear_left.targets.iter().any(|t| t.range_m < 40.0);
        self.cross_traffic_right = self.rear_right.targets.iter().any(|t| t.range_m < 40.0);
    }

    pub fn total_targets(&self) -> usize {
        self.front_center.targets.len()
            + self.rear_center.targets.len()
            + self.front_left.targets.len()
            + self.front_right.targets.len()
            + self.rear_left.targets.len()
            + self.rear_right.targets.len()
    }
}
