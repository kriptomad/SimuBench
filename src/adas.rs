use crate::sensors::SensorSuite;

/// ADAS domain controller (SAE Level 2)
#[derive(Debug, Clone)]
pub struct AdasModule {
    // Adaptive Cruise Control
    pub acc_enabled: bool,
    pub acc_set_speed: f64,    // km/h
    pub acc_headway_time: f64, // s (gap to lead)
    pub acc_throttle_out: f64,
    pub acc_brake_out: f64,
    acc_speed_error_integral: f64,

    // Autonomous Emergency Braking
    pub aeb_enabled: bool,
    pub aeb_pre_charge: bool, // brakes pre-charged
    pub aeb_active: bool,
    pub aeb_ttc: f64, // seconds to collision

    // Lane Keeping Assist / Lane Departure Warning
    pub lka_enabled: bool,
    pub lka_active: bool,
    pub lka_steering_correction: f64,
    pub ldw_warning: bool,

    // Blind Spot Monitoring
    pub bsm_enabled: bool,
    pub bsm_left_warning: bool,
    pub bsm_right_warning: bool,

    // Park Assist (front/rear ultrasonic guidance)
    pub park_assist_enabled: bool,
    pub park_assist_zone: ParkZone,

    // Traffic Sign Recognition
    pub tsr_active: bool,
    pub tsr_speed_limit: Option<u32>,

    // Driver Attention Monitor (simulated)
    pub dam_drowsiness: f64, // 0-1
    pub dam_alert: bool,

    // Autonomous level: 0=none, 1=partial, 2=combined
    pub autonomy_level: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParkZone {
    Clear,
    Caution,
    Warning,
    Critical,
}

impl std::fmt::Display for ParkZone {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParkZone::Clear => write!(f, "CLEAR"),
            ParkZone::Caution => write!(f, "CAUTION"),
            ParkZone::Warning => write!(f, "WARNING"),
            ParkZone::Critical => write!(f, "CRITICAL"),
        }
    }
}

impl AdasModule {
    pub fn new() -> Self {
        Self {
            acc_enabled: false,
            acc_set_speed: 80.0,
            acc_headway_time: 2.0,
            acc_throttle_out: 0.0,
            acc_brake_out: 0.0,
            acc_speed_error_integral: 0.0,

            aeb_enabled: true,
            aeb_pre_charge: false,
            aeb_active: false,
            aeb_ttc: f64::INFINITY,

            lka_enabled: false,
            lka_active: false,
            lka_steering_correction: 0.0,
            ldw_warning: false,

            bsm_enabled: true,
            bsm_left_warning: false,
            bsm_right_warning: false,

            park_assist_enabled: false,
            park_assist_zone: ParkZone::Clear,

            tsr_active: true,
            tsr_speed_limit: Some(80),

            dam_drowsiness: 0.0,
            dam_alert: false,

            autonomy_level: 2,
        }
    }

    /// Returns (throttle_override, brake_override, steering_override)
    /// None means no override for that channel.
    pub fn update(
        &mut self,
        sensors: &SensorSuite,
        vehicle_speed: f64,
        _user_throttle: f64,
        user_brake: f64,
        dt: f64,
    ) -> (Option<f64>, Option<f64>, Option<f64>) {
        let mut throttle_out: Option<f64> = None;
        let mut brake_out: Option<f64> = None;
        let mut steer_out: Option<f64> = None;

        // ------------------------------------------------------------------
        // AEB — highest priority
        // ------------------------------------------------------------------
        self.aeb_ttc = f64::INFINITY;
        self.aeb_pre_charge = false;
        self.aeb_active = false;

        if self.aeb_enabled {
            // Find closest in-path forward target
            if let Some(target) = sensors
                .forward_targets
                .iter()
                .filter(|t| t.is_in_path && t.relative_speed > 0.0)
                .min_by(|a, b| a.distance.total_cmp(&b.distance))
            {
                let ttc = target.distance / (target.relative_speed / 3.6).max(0.01);
                self.aeb_ttc = ttc;

                if ttc < 2.5 {
                    self.aeb_pre_charge = true;
                }
                if ttc < 1.8 {
                    // Partial braking
                    let partial = ((2.5 - ttc) / 1.5).clamp(0.0, 0.6);
                    brake_out = Some(partial.max(user_brake));
                }
                if ttc < 0.9 {
                    // Full AEB
                    self.aeb_active = true;
                    brake_out = Some(1.0);
                    throttle_out = Some(0.0);
                }
            }
        }

        // ------------------------------------------------------------------
        // ACC — overrides throttle/brake when enabled
        // ------------------------------------------------------------------
        if self.acc_enabled && !self.aeb_active {
            let lead = sensors
                .forward_targets
                .iter()
                .filter(|t| t.is_in_path)
                .min_by(|a, b| a.distance.total_cmp(&b.distance));

            let (_target_speed, speed_error) = if let Some(t) = lead {
                let safe_dist = vehicle_speed / 3.6 * self.acc_headway_time;
                let dist_error = t.distance - safe_dist;
                // Blend: follow gap OR speed, whichever is more restrictive
                let gap_speed = vehicle_speed + dist_error * 0.4;
                let tgt = gap_speed.min(self.acc_set_speed).max(0.0);
                (tgt, tgt - vehicle_speed)
            } else {
                (self.acc_set_speed, self.acc_set_speed - vehicle_speed)
            };

            // PI speed controller
            self.acc_speed_error_integral += speed_error * dt;
            self.acc_speed_error_integral = self.acc_speed_error_integral.clamp(-30.0, 30.0);
            let control = speed_error * 0.05 + self.acc_speed_error_integral * 0.002;

            if control > 0.0 {
                self.acc_throttle_out = control.clamp(0.0, 1.0);
                self.acc_brake_out = 0.0;
                throttle_out = Some(self.acc_throttle_out);
            } else {
                self.acc_brake_out = (-control).clamp(0.0, 0.8);
                self.acc_throttle_out = 0.0;
                throttle_out = Some(0.0);
                brake_out = Some(self.acc_brake_out.max(brake_out.unwrap_or(0.0)));
            }
        }

        // ------------------------------------------------------------------
        // LKA — steering correction
        // ------------------------------------------------------------------
        self.lka_active = false;
        self.lka_steering_correction = 0.0;
        if self.lka_enabled && vehicle_speed > 30.0 {
            let correction = -sensors.lane_offset * 4.0; // proportional
            if correction.abs() > 0.3 {
                self.lka_active = true;
                self.lka_steering_correction = correction.clamp(-5.0, 5.0);
                steer_out = Some(self.lka_steering_correction);
            }
        }
        self.ldw_warning = sensors.lane_departure_left || sensors.lane_departure_right;

        // ------------------------------------------------------------------
        // BSM
        // ------------------------------------------------------------------
        if self.bsm_enabled {
            self.bsm_left_warning = sensors.bsm_left;
            self.bsm_right_warning = sensors.bsm_right;
        }

        // ------------------------------------------------------------------
        // Park Assist
        // ------------------------------------------------------------------
        let front_min = sensors.ultrasonic[..3]
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        self.park_assist_zone = if front_min > 2.0 {
            ParkZone::Clear
        } else if front_min > 1.0 {
            ParkZone::Caution
        } else if front_min > 0.4 {
            ParkZone::Warning
        } else {
            ParkZone::Critical
        };

        // ------------------------------------------------------------------
        // TSR
        // ------------------------------------------------------------------
        self.tsr_speed_limit = sensors.speed_limit_kmh;

        (throttle_out, brake_out, steer_out)
    }

    pub fn toggle_acc(&mut self, current_speed: f64) {
        self.acc_enabled = !self.acc_enabled;
        if self.acc_enabled {
            self.acc_set_speed = current_speed.max(30.0);
            self.acc_speed_error_integral = 0.0;
        }
    }

    pub fn acc_speed_up(&mut self) {
        self.acc_set_speed = (self.acc_set_speed + 5.0).min(200.0);
    }
    pub fn acc_speed_down(&mut self) {
        self.acc_set_speed = (self.acc_set_speed - 5.0).max(0.0);
    }
}
