//! Camera-based perception — simulates automotive vision system.
#![allow(dead_code)]
//! Models: forward camera (ADAS), surround-view, traffic sign recognition,
//! object detection (YOLOv8-style outputs), lane detection with Hough transform.

// ── Lane mark type ────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MarkType {
    Solid,
    Dashed,
    DoubleSolid,
    DottedShort,
    GuardRail,
    None,
}

impl std::fmt::Display for MarkType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MarkType::Solid => "Solid",
            MarkType::Dashed => "Dashed",
            MarkType::DoubleSolid => "Dbl-Solid",
            MarkType::DottedShort => "Dotted",
            MarkType::GuardRail => "GuardRail",
            MarkType::None => "None",
        }
        .fmt(f)
    }
}

// ── Traffic sign ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SignType {
    SpeedLimit(u32),
    StopSign,
    GiveWay,
    NoOvertaking,
    RoadWorks,
    PedestrianCrossing,
    Roundabout,
    HazardWarning,
    Unknown,
}

impl std::fmt::Display for SignType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SignType::SpeedLimit(v) => write!(f, "Speed {v} km/h"),
            SignType::StopSign => write!(f, "STOP"),
            SignType::GiveWay => write!(f, "Give Way"),
            SignType::NoOvertaking => write!(f, "No Overtaking"),
            SignType::RoadWorks => write!(f, "Road Works"),
            SignType::PedestrianCrossing => write!(f, "Ped Crossing"),
            SignType::Roundabout => write!(f, "Roundabout"),
            SignType::HazardWarning => write!(f, "Hazard"),
            SignType::Unknown => write!(f, "Unknown"),
        }
    }
}

// ── Detected sign ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct DetectedSign {
    pub sign_type: SignType,
    pub distance_m: f64,
    pub confidence: f64,
    pub pixel_x: u16,
    pub pixel_y: u16,
    pub pixel_width: u16,
}

// ── Object detection result ───────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObjectClass {
    Car,
    Truck,
    Motorcycle,
    Bus,
    Pedestrian,
    Cyclist,
    Cone,
    Barrier,
    Animal,
    Unknown,
}

impl std::fmt::Display for ObjectClass {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ObjectClass::Car => write!(f, "Car       "),
            ObjectClass::Truck => write!(f, "Truck     "),
            ObjectClass::Motorcycle => write!(f, "Moto      "),
            ObjectClass::Bus => write!(f, "Bus       "),
            ObjectClass::Pedestrian => write!(f, "Pedestrian"),
            ObjectClass::Cyclist => write!(f, "Cyclist   "),
            ObjectClass::Cone => write!(f, "Cone      "),
            ObjectClass::Barrier => write!(f, "Barrier   "),
            ObjectClass::Animal => write!(f, "Animal    "),
            ObjectClass::Unknown => write!(f, "Unknown   "),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DetectedObject {
    pub id: u32,
    pub class: ObjectClass,
    pub confidence: f64, // 0-1 (YOLOv8-style score)
    pub distance_m: f64,
    pub azimuth_deg: f64, // ±45° from forward
    pub width_m: f64,     // estimated physical width
    pub height_m: f64,
    pub bbox_x: u16, // bounding box pixel coords
    pub bbox_y: u16,
    pub bbox_w: u16,
    pub bbox_h: u16,
    pub velocity_ms: Option<f64>, // optical flow estimate
}

// ── Lane detection ────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct LaneDetection {
    pub detected: bool,
    pub confidence: f64,
    pub left_offset_m: f64,  // distance to left lane mark (metres)
    pub right_offset_m: f64, // distance to right lane mark
    pub lane_width_m: f64,
    pub ego_offset_m: f64,      // lateral offset from centre (+right)
    pub heading_error_deg: f64, // vehicle heading vs lane heading
    pub curvature_inv_m: f64,   // 1/R (0 = straight)
    pub left_type: MarkType,
    pub right_type: MarkType,
    pub adjacent_left: bool,
    pub adjacent_right: bool,
    pub departure_left: bool,
    pub departure_right: bool,
    /// Predicted time to lane departure (seconds, if not corrected)
    pub ttld_s: f64,
}

impl LaneDetection {
    pub fn new() -> Self {
        LaneDetection {
            detected: false,
            confidence: 0.0,
            left_offset_m: 1.75,
            right_offset_m: 1.75,
            lane_width_m: 3.5,
            ego_offset_m: 0.0,
            heading_error_deg: 0.0,
            curvature_inv_m: 0.0,
            left_type: MarkType::Dashed,
            right_type: MarkType::Solid,
            adjacent_left: true,
            adjacent_right: false,
            departure_left: false,
            departure_right: false,
            ttld_s: f64::INFINITY,
        }
    }
}

// ── Camera specification ──────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub struct CameraSpec {
    pub resolution_w: u16, // pixels
    pub resolution_h: u16,
    pub fov_h_deg: f64, // horizontal field of view
    pub fov_v_deg: f64,
    pub fps: f64,
    pub focal_length_mm: f64,
    pub pixel_size_um: f64,
}

impl CameraSpec {
    pub fn forward_adas() -> Self {
        CameraSpec {
            resolution_w: 1920,
            resolution_h: 1080,
            fov_h_deg: 100.0,
            fov_v_deg: 55.0,
            fps: 30.0,
            focal_length_mm: 7.7,
            pixel_size_um: 3.45,
        }
    }
    pub fn surround_wide() -> Self {
        CameraSpec {
            resolution_w: 1280,
            resolution_h: 720,
            fov_h_deg: 190.0,
            fov_v_deg: 120.0,
            fps: 25.0,
            focal_length_mm: 1.8,
            pixel_size_um: 4.0,
        }
    }
}

// ── Vision system ─────────────────────────────────────────────────────────────
pub struct CameraSystem {
    pub forward_spec: CameraSpec,
    pub rear_spec: CameraSpec,

    // ─ Lane detection output (from Hough transform + polynomial fit) ──────────
    pub lane: LaneDetection,

    // ─ Object detection (YOLOv8-style outputs) ───────────────────────────────
    pub objects_front: Vec<DetectedObject>,
    pub objects_rear: Vec<DetectedObject>,

    // ─ Traffic sign recognition ───────────────────────────────────────────────
    pub signs: Vec<DetectedSign>,
    pub active_speed_limit: Option<u32>,

    // ─ Surround view / blind spot (4× wide-angle cameras) ────────────────────
    pub front_clear: bool,
    pub rear_clear: bool,
    pub left_clear: bool,
    pub right_clear: bool,

    // ─ Processing stats ───────────────────────────────────────────────────────
    pub inference_ms: f64, // simulated neural network inference time
    pub frames_processed: u64,
    pub detection_fps: f64,

    // ─ System health ─────────────────────────────────────────────────────────
    pub forward_cam_fault: bool,
    pub rear_cam_fault: bool,
    pub obscured: bool, // camera obscured (mud, frost, sun glare)
    pub calibration_valid: bool,

    // ─ Internal ──────────────────────────────────────────────────────────────
    update_timer: f64,
    noise_t: f64,
    obj_id_counter: u32,
}

impl CameraSystem {
    pub fn new() -> Self {
        CameraSystem {
            forward_spec: CameraSpec::forward_adas(),
            rear_spec: CameraSpec::surround_wide(),
            lane: LaneDetection::new(),
            objects_front: Vec::new(),
            objects_rear: Vec::new(),
            signs: Vec::new(),
            active_speed_limit: Some(80),
            front_clear: true,
            rear_clear: true,
            left_clear: true,
            right_clear: true,
            inference_ms: 12.0,
            frames_processed: 0,
            detection_fps: 30.0,
            forward_cam_fault: false,
            rear_cam_fault: false,
            obscured: false,
            calibration_valid: true,
            update_timer: 0.0,
            noise_t: 0.0,
            obj_id_counter: 0,
        }
    }

    /// Update camera system — called each simulation tick.
    pub fn update(
        &mut self,
        ego_speed_kmh: f64,
        _ego_heading: f64,
        radar_ttc: f64,
        elapsed: f64,
        dt: f64,
    ) {
        self.noise_t += dt;
        self.update_timer += dt;

        // Forward camera runs at 30 fps
        if self.update_timer < 1.0 / self.forward_spec.fps {
            return;
        }
        self.update_timer = 0.0;
        self.frames_processed += 1;

        let n = self.noise_t;
        let ns = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5).fract() - 0.5;

        // ─ Lane detection (simulates Hough + polynomial fitting) ─────────────
        let speed_factor = (ego_speed_kmh / 60.0).clamp(0.0, 1.0);
        self.lane.detected = ego_speed_kmh > 3.0 && !self.obscured;
        self.lane.confidence = if self.lane.detected {
            (0.92 + ns(n * 1.1) * 0.05).clamp(0.5, 0.99)
        } else {
            0.0
        };
        self.lane.ego_offset_m = ns(n * 0.3) * 0.15 * speed_factor;
        self.lane.heading_error_deg = ns(n * 0.5) * 1.5 * speed_factor;
        self.lane.curvature_inv_m = ns(n * 0.01) * 0.002;
        let lo = 1.75 + self.lane.ego_offset_m;
        let ro = 1.75 - self.lane.ego_offset_m;
        self.lane.left_offset_m = lo;
        self.lane.right_offset_m = ro;
        self.lane.lane_width_m = lo + ro;
        self.lane.departure_left = lo < 0.5;
        self.lane.departure_right = ro < 0.5;
        self.lane.ttld_s = if self.lane.departure_left || self.lane.departure_right {
            lo.min(ro) / (ego_speed_kmh / 3.6).max(0.1) * 5.0
        } else {
            f64::INFINITY
        };

        // ─ Object detection (simulated neural network outputs) ─────────────
        self.objects_front.clear();
        // Lead vehicle simulation
        if radar_ttc < 30.0 {
            let dist = (ego_speed_kmh / 3.6 * radar_ttc.min(10.0)).max(5.0);
            self.objects_front.push(DetectedObject {
                id: 1,
                class: ObjectClass::Car,
                confidence: 0.94 + ns(n * 2.1) * 0.04,
                distance_m: dist,
                azimuth_deg: ns(n * 3.1) * 2.0,
                width_m: 1.9,
                height_m: 1.5,
                bbox_x: 760,
                bbox_y: 400,
                bbox_w: 120,
                bbox_h: 80,
                velocity_ms: Some((ego_speed_kmh - 5.0) / 3.6),
            });
        }
        // Periodic additional objects
        let t_mod = elapsed % 20.0;
        if t_mod > 5.0 && t_mod < 15.0 {
            self.objects_front.push(DetectedObject {
                id: 2,
                class: if (t_mod * 0.5).sin() > 0.0 {
                    ObjectClass::Pedestrian
                } else {
                    ObjectClass::Cyclist
                },
                confidence: 0.72 + ns(n * 4.1) * 0.1,
                distance_m: 18.0 + ns(n * 5.1) * 5.0,
                azimuth_deg: 8.0 + ns(n * 6.1) * 3.0,
                width_m: 0.5,
                height_m: 1.8,
                bbox_x: 1100,
                bbox_y: 350,
                bbox_w: 40,
                bbox_h: 90,
                velocity_ms: None,
            });
        }

        // ─ Traffic sign recognition ──────────────────────────────────────────
        self.signs.clear();
        // Speed limit sign simulation
        let sign_cycle = (elapsed / 45.0) as u32;
        let speed_lim = if sign_cycle % 2 == 0 { 80 } else { 50 };
        self.active_speed_limit = Some(speed_lim);
        if elapsed % 45.0 < 5.0 {
            self.signs.push(DetectedSign {
                sign_type: SignType::SpeedLimit(speed_lim),
                distance_m: 60.0 - (elapsed % 45.0) * 10.0,
                confidence: 0.97,
                pixel_x: 1600,
                pixel_y: 300,
                pixel_width: 60,
            });
        }
        if (elapsed % 60.0) < 3.0 {
            self.signs.push(DetectedSign {
                sign_type: SignType::RoadWorks,
                distance_m: 180.0,
                confidence: 0.88,
                pixel_x: 1700,
                pixel_y: 280,
                pixel_width: 50,
            });
        }

        // ─ Surround-view / blind spots ───────────────────────────────────────
        let lateral_t = elapsed * 0.13;
        self.left_clear = (lateral_t.sin()).abs() < 0.8;
        self.right_clear = (lateral_t + 1.2).cos().abs() < 0.85;

        // ─ Inference time simulation (with variability) ──────────────────────
        self.inference_ms = 10.0 + ns(n * 8.1).abs() * 5.0;

        // ─ Camera health ─────────────────────────────────────────────────────
        self.calibration_valid = true;
        self.obscured = false;
    }

    pub fn closest_object(&self) -> Option<&DetectedObject> {
        self.objects_front
            .iter()
            .min_by(|a, b| a.distance_m.partial_cmp(&b.distance_m).unwrap())
    }
}

// ── Sensor Fusion ─────────────────────────────────────────────────────────────
/// Fuses radar + camera + LIDAR into a unified object list using track-level fusion.
/// Implements a simplified Joint Probabilistic Data Association (JPDA) approach.

#[derive(Debug, Clone)]
pub struct FusedObject {
    pub id: u32,
    pub class: ObjectClass,
    pub distance_m: f64,
    pub azimuth_deg: f64,
    pub speed_ms: f64,
    pub lateral_pos_m: f64,
    pub confidence: f64,
    pub sensor_sources: u8, // bitmask: bit0=radar, bit1=camera, bit2=lidar
    pub ttc_s: f64,
    pub in_path: bool,
    /// Kalman state: [x, y, vx, vy]
    kf_x: [f64; 4],
    kf_p: [f64; 4], // diagonal P
}

impl FusedObject {
    fn new(id: u32, class: ObjectClass, dist: f64, az: f64, speed: f64) -> Self {
        let ttc = if speed > 0.1 {
            dist / speed
        } else {
            f64::INFINITY
        };
        FusedObject {
            id,
            class,
            distance_m: dist,
            azimuth_deg: az,
            speed_ms: speed,
            lateral_pos_m: az.to_radians().sin() * dist,
            confidence: 0.7,
            sensor_sources: 0,
            ttc_s: ttc,
            in_path: az.abs() < 5.0,
            kf_x: [dist, 0.0, speed, 0.0],
            kf_p: [5.0, 5.0, 2.0, 2.0],
        }
    }

    fn predict(&mut self, dt: f64) {
        self.kf_x[0] += self.kf_x[2] * dt;
        self.kf_x[1] += self.kf_x[3] * dt;
        self.kf_p[0] += 0.5 * dt;
        self.kf_p[1] += 0.5 * dt;
        self.kf_p[2] += 0.3 * dt;
        self.kf_p[3] += 0.3 * dt;
    }

    fn update_with_measurement(&mut self, dist: f64, az: f64, spd: f64, src_bit: u8) {
        let r = 1.5;
        let k = [
            self.kf_p[0] / (self.kf_p[0] + r),
            self.kf_p[2] / (self.kf_p[2] + r),
        ];
        self.kf_x[0] += k[0] * (dist - self.kf_x[0]);
        self.kf_x[2] += k[1] * (spd - self.kf_x[2]);
        self.kf_p[0] *= 1.0 - k[0];
        self.kf_p[2] *= 1.0 - k[1];
        self.distance_m = self.kf_x[0].max(0.0);
        self.speed_ms = self.kf_x[2];
        self.azimuth_deg = az;
        self.lateral_pos_m = az.to_radians().sin() * self.distance_m;
        self.in_path = self.azimuth_deg.abs() < 5.0;
        self.ttc_s = if self.speed_ms > 0.5 {
            self.distance_m / self.speed_ms
        } else {
            f64::INFINITY
        };
        self.sensor_sources |= src_bit;
        self.confidence = (0.5 + self.sensor_sources.count_ones() as f64 * 0.15).min(0.99);
    }
}

pub struct SensorFusion {
    pub objects: Vec<FusedObject>,
    pub ttc_critical: f64, // minimum TTC in path
    pub lead_dist_m: f64,
    id_counter: u32,
}

impl SensorFusion {
    pub fn new() -> Self {
        SensorFusion {
            objects: Vec::new(),
            ttc_critical: f64::INFINITY,
            lead_dist_m: f64::INFINITY,
            id_counter: 0,
        }
    }

    pub fn fuse(
        &mut self,
        camera: &CameraSystem,
        radar_front_targets: &[crate::radar::RadarTarget],
        ego_speed_ms: f64,
        dt: f64,
    ) {
        // Predict all existing tracks
        for obj in &mut self.objects {
            obj.predict(dt);
        }

        // ─ Associate and update from RADAR (source bit 0) ─────────────────────
        for rt in radar_front_targets {
            let closing = if rt.range_rate_ms < 0.0 {
                -rt.range_rate_ms
            } else {
                0.0
            };
            if let Some(o) = self
                .objects
                .iter_mut()
                .find(|o| (o.distance_m - rt.range_m).abs() < 8.0)
            {
                o.update_with_measurement(rt.range_m, rt.azimuth_deg, closing, 0x01);
            } else {
                let mut o = FusedObject::new(
                    self.id_counter,
                    ObjectClass::Car,
                    rt.range_m,
                    rt.azimuth_deg,
                    closing,
                );
                o.sensor_sources = 0x01;
                self.objects.push(o);
                self.id_counter += 1;
            }
        }

        // ─ Associate and update from CAMERA (source bit 1) ───────────────────
        for co in &camera.objects_front {
            if let Some(o) = self.objects.iter_mut().find(|o| {
                (o.distance_m - co.distance_m).abs() < 5.0
                    && (o.azimuth_deg - co.azimuth_deg).abs() < 8.0
            }) {
                let spd = co
                    .velocity_ms
                    .map(|v| (ego_speed_ms - v).max(0.0))
                    .unwrap_or(o.speed_ms);
                o.update_with_measurement(co.distance_m, co.azimuth_deg, spd, 0x02);
            } else {
                let spd = co
                    .velocity_ms
                    .map(|v| (ego_speed_ms - v).max(0.0))
                    .unwrap_or(5.0);
                let mut o = FusedObject::new(
                    self.id_counter,
                    co.class,
                    co.distance_m,
                    co.azimuth_deg,
                    spd,
                );
                o.sensor_sources = 0x02;
                self.objects.push(o);
                self.id_counter += 1;
            }
        }

        // Remove stale / out-of-range objects
        self.objects
            .retain(|o| o.distance_m < 200.0 && o.confidence > 0.2);
        self.objects.truncate(64);

        // Aggregate
        self.ttc_critical = self
            .objects
            .iter()
            .filter(|o| o.in_path && o.speed_ms > 0.1)
            .map(|o| o.ttc_s)
            .fold(f64::INFINITY, f64::min);
        self.lead_dist_m = self
            .objects
            .iter()
            .filter(|o| o.in_path)
            .map(|o| o.distance_m)
            .fold(f64::INFINITY, f64::min);
    }
}
