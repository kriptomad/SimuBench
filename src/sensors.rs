use crate::vehicle::TrafficObject;

/// A radar return from the forward/rear 77 GHz radar
#[derive(Debug, Clone)]
pub struct RadarTarget {
    pub distance: f64,       // m
    pub relative_speed: f64, // km/h (positive = approaching)
    pub angle_deg: f64,      // degrees from forward axis
    pub is_in_path: bool,
}

/// Complete sensor suite (all simulated)
#[derive(Debug, Clone)]
pub struct SensorSuite {
    // 77 GHz RADAR
    pub forward_targets: Vec<RadarTarget>,
    pub rear_targets: Vec<RadarTarget>,

    // Mono camera — lane detection
    pub lane_offset: f64,     // m from lane center (+ = right)
    pub lane_width: f64,      // m
    pub lane_curvature: f64,  // 1/m
    pub lane_confidence: f64, // 0-1
    pub lane_departure_left: bool,
    pub lane_departure_right: bool,

    // Blind spot monitoring (short-range radar)
    pub bsm_left: bool,
    pub bsm_right: bool,

    // Ultrasonic parking sensors: [FL,FC,FR, BL,BC,BR + 2 corners]
    pub ultrasonic: [f64; 8],

    // Traffic Sign Recognition (camera-based)
    pub speed_limit_kmh: Option<u32>,
    pub stop_sign_ahead: bool,

    // IMU
    pub longitudinal_g: f64,
    pub lateral_g: f64,
    pub yaw_rate_imu: f64, // deg/s

    // GPS (simplified)
    pub gps_speed: f64,   // km/h
    pub gps_heading: f64, // degrees
}

impl SensorSuite {
    pub fn new() -> Self {
        Self {
            forward_targets: Vec::new(),
            rear_targets: Vec::new(),
            lane_offset: 0.0,
            lane_width: 3.5,
            lane_curvature: 0.0,
            lane_confidence: 0.92,
            lane_departure_left: false,
            lane_departure_right: false,
            bsm_left: false,
            bsm_right: false,
            ultrasonic: [8.0; 8],
            speed_limit_kmh: Some(80),
            stop_sign_ahead: false,
            longitudinal_g: 0.0,
            lateral_g: 0.0,
            yaw_rate_imu: 0.0,
            gps_speed: 0.0,
            gps_heading: 0.0,
        }
    }

    /// Update all sensor readings from vehicle state and traffic
    pub fn update(
        &mut self,
        vehicle_speed: f64,
        steering_angle: f64,
        acceleration: f64,
        heading: f64,
        traffic: &[TrafficObject],
        elapsed: f64,
        dt: f64,
    ) {
        // --- Forward RADAR -----------------------------------------------
        self.forward_targets.clear();
        for obj in traffic {
            if obj.distance > 0.0 && obj.distance < 200.0 {
                self.forward_targets.push(RadarTarget {
                    distance: obj.distance,
                    relative_speed: obj.closing_speed,
                    angle_deg: (obj.lateral_offset / obj.distance.max(1.0))
                        .atan()
                        .to_degrees(),
                    is_in_path: obj.in_path(),
                });
            }
        }

        // --- Rear RADAR --------------------------------------------------
        self.rear_targets.clear();
        for obj in traffic {
            if obj.distance < 0.0 && obj.distance > -50.0 {
                self.rear_targets.push(RadarTarget {
                    distance: -obj.distance,
                    relative_speed: -obj.closing_speed,
                    angle_deg: 0.0,
                    is_in_path: obj.lateral_offset.abs() < 1.5,
                });
            }
        }

        // --- Lane detection ----------------------------------------------
        // Drift lane_offset based on steering (simplified)
        self.lane_offset += steering_angle * 0.001 * dt * vehicle_speed / 3.6;
        self.lane_offset = self.lane_offset.clamp(-2.0, 2.0);
        self.lane_confidence = (0.92 + noise(elapsed * 7.3) * 0.04).clamp(0.6, 1.0);
        self.lane_departure_left = self.lane_offset < -1.1;
        self.lane_departure_right = self.lane_offset > 1.1;

        // --- BSM (side radar, 70m range) ---------------------------------
        // Simulate periodic vehicle on right side
        self.bsm_right = (elapsed / 15.0).sin() > 0.7;
        self.bsm_left = (elapsed / 22.0).sin() > 0.85;

        // --- Ultrasonic (parking) ----------------------------------------
        // Front center sensor reacts to nearest forward target
        if let Some(nearest) = self
            .forward_targets
            .iter()
            .filter(|t| t.is_in_path)
            .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())
        {
            self.ultrasonic[1] = nearest.distance.min(8.0); // front center
        } else {
            self.ultrasonic[1] = 8.0;
        }
        self.ultrasonic[0] = (self.ultrasonic[1] + 0.3 + noise(elapsed * 5.0) * 0.1).min(8.0);
        self.ultrasonic[2] = (self.ultrasonic[1] + 0.2 + noise(elapsed * 6.0) * 0.1).min(8.0);
        // Rear static
        for i in 3..8 {
            self.ultrasonic[i] =
                (5.0 + noise(elapsed * f64::from(i as u8) * 3.1) * 0.5).clamp(0.1, 8.0);
        }

        // --- TSR ---------------------------------------------------------
        // Change speed limit sign periodically
        if ((elapsed / 30.0) as u32) % 2 == 0 {
            self.speed_limit_kmh = Some(80);
        } else {
            self.speed_limit_kmh = Some(120);
        }

        // --- IMU ---------------------------------------------------------
        self.longitudinal_g = acceleration / 9.81;
        self.lateral_g = steering_angle * vehicle_speed / 3.6 / 9.81 * 0.08;
        self.yaw_rate_imu = steering_angle * vehicle_speed / 3.6 / 2.7;

        // --- GPS ---------------------------------------------------------
        self.gps_speed = vehicle_speed + noise(elapsed * 2.1) * 0.3;
        self.gps_heading = heading;
    }
}

/// Cheap pseudo-random noise in [-1, 1]
fn noise(seed: f64) -> f64 {
    let x = (seed * 127.1 + 311.7).sin() * 43758.545;
    x - x.floor() - 0.5
}
