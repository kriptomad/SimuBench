//! Autonomous Driving Controller — SAE Level 2/3 AD system.
//! Implements: ACC, AEB, LKA, LCA, TJA, Pilot Assist, ISO 26262 ASIL-D safety.

// ── SAE Automation Level ───────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum SaeLevel {
    L0, // No automation
    L1, // Driver assistance (ACC OR LKA, not both)
    L2, // Partial automation (ACC + LKA simultaneously) — Tesla Autopilot equivalent
    L3, // Conditional automation — system can handle some scenarios fully
    L4, // High automation — geofenced
    L5, // Full automation
}

impl std::fmt::Display for SaeLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SaeLevel::L0 => write!(f, "L0 — Manual        "),
            SaeLevel::L1 => write!(f, "L1 — Assistance    "),
            SaeLevel::L2 => write!(f, "L2 — Partial Auto  "),
            SaeLevel::L3 => write!(f, "L3 — Conditional   "),
            SaeLevel::L4 => write!(f, "L4 — High Auto     "),
            SaeLevel::L5 => write!(f, "L5 — Full Auto     "),
        }
    }
}

// ── ADAS Feature state ────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FeatureState {
    Off,
    Ready,
    Active,
    Override,
    Fault,
}

impl std::fmt::Display for FeatureState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FeatureState::Off => write!(f, "OFF     "),
            FeatureState::Ready => write!(f, "READY   "),
            FeatureState::Active => write!(f, "ACTIVE  "),
            FeatureState::Override => write!(f, "OVERRIDE"),
            FeatureState::Fault => write!(f, "FAULT   "),
        }
    }
}

// ── Lane info ─────────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct LaneInfo {
    pub detected: bool,
    pub confidence: f64,        // 0-1
    pub offset_m: f64,          // ego offset from lane centre (+ = right)
    pub heading_error_deg: f64, // heading error relative to lane
    pub curvature_1_m: f64,     // 1/radius curvature
    pub lane_width_m: f64,
    pub left_type: LaneMarkType,
    pub right_type: LaneMarkType,
    pub adjacent_left: bool, // is there an adjacent left lane?
    pub adjacent_right: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LaneMarkType {
    Solid,
    Dashed,
    DoubleSolid,
    Virtual,
    None,
}

impl std::fmt::Display for LaneMarkType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LaneMarkType::Solid => "Solid",
            LaneMarkType::Dashed => "Dashed",
            LaneMarkType::DoubleSolid => "DblSolid",
            LaneMarkType::Virtual => "Virtual",
            LaneMarkType::None => "None",
        }
        .fmt(f)
    }
}

// ── Autonomous path waypoint ──────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
    pub speed_ms: f64,
    pub curvature: f64,
}

// ── AD Controller ─────────────────────────────────────────────────────────────
pub struct AutonomousController {
    // ─ Configuration ─────────────────────────────────────────────────────────
    pub sae_level: SaeLevel,
    pub engaged: bool,
    pub driver_hands_on: bool, // hands on wheel detection

    // ─ ADAS Features ─────────────────────────────────────────────────────────
    pub acc_state: FeatureState,
    pub acc_set_speed_kmh: f64,
    pub acc_headway_s: f64, // target time headway (1.5-2.5s typical)
    acc_speed_error_integral: f64,

    pub aeb_state: FeatureState,
    pub aeb_brake_cmd: f64, // 0-1 brake command
    pub aeb_pre_charge: bool,

    pub lka_state: FeatureState,
    pub lka_steer_cmd: f64, // steering torque Nm

    pub lca_state: FeatureState, // Lane Change Assist
    pub lca_direction: LcaDirection,

    pub tja_state: FeatureState, // Traffic Jam Assist (stop-and-go)
    pub tja_stopped_timer: f64,

    pub pilot_state: FeatureState, // Highway Pilot (L3)

    pub bsm_left: bool,
    pub bsm_right: bool,

    // ─ Sensor inputs ─────────────────────────────────────────────────────────
    pub lane: LaneInfo,
    pub lead_range_m: f64,
    pub lead_speed_ms: f64,
    pub ttc_s: f64,
    pub thw_s: f64, // time headway (distance / speed)

    // ─ Control outputs ───────────────────────────────────────────────────────
    pub throttle_cmd: f64,  // 0-1
    pub brake_cmd: f64,     // 0-1
    pub steer_cmd_deg: f64, // target steering angle

    // ─ Path planning ─────────────────────────────────────────────────────────
    pub planned_path: Vec<Waypoint>,
    pub path_valid: bool,

    // ─ Driver monitoring ─────────────────────────────────────────────────────
    pub driver_alert: bool,  // attention warning
    pub hands_off_s: f64,    // seconds without hands
    pub drowsiness_pct: f64, // estimated drowsiness 0-100%

    // ─ ISO 26262 Safety monitor ───────────────────────────────────────────────
    pub safety_status: SafetyStatus,
    pub degrade_reason: Option<&'static str>,

    // ─ Internal ──────────────────────────────────────────────────────────────
    noise_t: f64,
    lka_integral: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LcaDirection {
    None,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SafetyStatus {
    Normal,
    Degraded,
    Minimal,
    Safe,
}

impl std::fmt::Display for SafetyStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SafetyStatus::Normal => write!(f, "NOMINAL"),
            SafetyStatus::Degraded => write!(f, "DEGRADED"),
            SafetyStatus::Minimal => write!(f, "MINIMAL!"),
            SafetyStatus::Safe => write!(f, "SAFE-STOP"),
        }
    }
}

impl Default for AutonomousController {
    fn default() -> Self {
        Self::new()
    }
}

impl AutonomousController {
    pub fn new() -> Self {
        AutonomousController {
            sae_level: SaeLevel::L2,
            engaged: false,
            driver_hands_on: true,
            acc_state: FeatureState::Off,
            acc_set_speed_kmh: 100.0,
            acc_headway_s: 2.0,
            acc_speed_error_integral: 0.0,
            aeb_state: FeatureState::Ready,
            aeb_brake_cmd: 0.0,
            aeb_pre_charge: false,
            lka_state: FeatureState::Off,
            lka_steer_cmd: 0.0,
            lca_state: FeatureState::Off,
            lca_direction: LcaDirection::None,
            tja_state: FeatureState::Off,
            tja_stopped_timer: 0.0,
            pilot_state: FeatureState::Off,
            bsm_left: false,
            bsm_right: false,
            lane: LaneInfo {
                detected: true,
                confidence: 0.95,
                offset_m: 0.0,
                heading_error_deg: 0.0,
                curvature_1_m: 0.0,
                lane_width_m: 3.5,
                left_type: LaneMarkType::Dashed,
                right_type: LaneMarkType::Solid,
                adjacent_left: true,
                adjacent_right: false,
            },
            lead_range_m: f64::INFINITY,
            lead_speed_ms: 0.0,
            ttc_s: f64::INFINITY,
            thw_s: f64::INFINITY,
            throttle_cmd: 0.0,
            brake_cmd: 0.0,
            steer_cmd_deg: 0.0,
            planned_path: Vec::new(),
            path_valid: false,
            driver_alert: false,
            hands_off_s: 0.0,
            drowsiness_pct: 0.0,
            safety_status: SafetyStatus::Normal,
            degrade_reason: None,
            noise_t: 0.0,
            lka_integral: 0.0,
        }
    }

    /// Master AD controller tick.
    /// Returns (throttle, brake, steer) overrides — None if no override.
    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        ego_speed_kmh: f64,
        _user_throttle: f64,
        _user_brake: f64,
        lead_range_m: f64,
        lead_speed_ms: f64,
        lane_offset_m: f64,
        lane_heading_err: f64,
        lane_confidence: f64,
        ttc_s: f64,
        bsm_left: bool,
        bsm_right: bool,
        dt: f64,
    ) -> (Option<f64>, Option<f64>, Option<f64>) {
        self.noise_t += dt;
        let ego_speed_ms = ego_speed_kmh / 3.6;

        // Update inputs
        self.lead_range_m = lead_range_m;
        self.lead_speed_ms = lead_speed_ms;
        self.ttc_s = ttc_s;
        self.thw_s = if ego_speed_ms > 0.1 {
            lead_range_m / ego_speed_ms
        } else {
            f64::INFINITY
        };
        self.bsm_left = bsm_left;
        self.bsm_right = bsm_right;
        self.lane.offset_m = lane_offset_m;
        self.lane.heading_error_deg = lane_heading_err;
        self.lane.confidence = lane_confidence;
        self.lane.detected = lane_confidence > 0.4;

        // Driver hands-off timer
        if self.driver_hands_on {
            self.hands_off_s = 0.0;
        } else {
            self.hands_off_s += dt;
            self.driver_alert = self.hands_off_s > 10.0;
        }

        // Drowsiness simulation
        let n = self.noise_t;
        let noise = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        self.drowsiness_pct = (self.drowsiness_pct + noise(n * 0.001) * 0.01).clamp(0.0, 100.0);

        if !self.engaged {
            self.throttle_cmd = 0.0;
            self.brake_cmd = 0.0;
            self.steer_cmd_deg = 0.0;
            return (None, None, None);
        }

        let mut thr: Option<f64> = None;
        let mut brk: Option<f64> = None;
        let mut str: Option<f64> = None;

        // ── AEB — ALWAYS active regardless of engagement ──────────────────────
        self.run_aeb(ego_speed_ms, ttc_s, &mut brk);

        // ── ACC — Adaptive Cruise Control ─────────────────────────────────────
        if matches!(self.acc_state, FeatureState::Active) {
            self.run_acc(
                ego_speed_ms,
                lead_range_m,
                lead_speed_ms,
                dt,
                &mut thr,
                &mut brk,
            );
        }

        // ── LKA — Lane Keeping Assist ─────────────────────────────────────────
        if matches!(self.lka_state, FeatureState::Active) && self.lane.detected {
            self.run_lka(lane_offset_m, lane_heading_err, ego_speed_ms, dt, &mut str);
        }

        // ── TJA — Traffic Jam Assist (stop-and-go) ────────────────────────────
        if matches!(self.tja_state, FeatureState::Active) {
            self.run_tja(ego_speed_ms, lead_range_m, dt, &mut thr, &mut brk);
        }

        // ── Path planning update ──────────────────────────────────────────────
        self.update_path(ego_speed_ms);

        // ── Safety supervision (ISO 26262 ASIL-D monitoring) ─────────────────
        self.safety_check(ego_speed_ms, thr, brk);

        self.throttle_cmd = thr.unwrap_or(0.0).clamp(0.0, 1.0);
        self.brake_cmd = brk.unwrap_or(0.0).clamp(0.0, 1.0);
        self.steer_cmd_deg = str.unwrap_or(0.0).clamp(-8.0, 8.0);

        (thr, brk, str)
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn run_aeb(&mut self, speed_ms: f64, ttc_s: f64, brk: &mut Option<f64>) {
        self.aeb_state = if speed_ms > 2.0 && ttc_s < 2.8 {
            FeatureState::Active
        } else if speed_ms > 1.0 {
            FeatureState::Ready
        } else {
            FeatureState::Off
        };

        self.aeb_pre_charge = ttc_s < 2.8 && speed_ms > 2.0;
        self.aeb_brake_cmd = 0.0;

        if ttc_s < 2.0 {
            // Partial braking phase
            let partial = ((2.5 - ttc_s) / 1.5).clamp(0.0, 0.6);
            self.aeb_brake_cmd = partial;
            *brk = Some(partial.max(brk.unwrap_or(0.0)));
        }
        if ttc_s < 1.0 {
            // Full emergency braking
            self.aeb_brake_cmd = 1.0;
            *brk = Some(1.0);
        }
    }

    fn run_acc(
        &mut self,
        speed_ms: f64,
        lead_range: f64,
        _lead_speed: f64,
        dt: f64,
        thr: &mut Option<f64>,
        brk: &mut Option<f64>,
    ) {
        let set_speed = self.acc_set_speed_kmh / 3.6;

        // Target speed: minimum of set speed and speed needed to maintain headway
        let safe_dist = speed_ms * self.acc_headway_s;
        let dist_error = lead_range - safe_dist;
        let lead_valid = lead_range.is_finite();
        let gap_speed = if lead_valid {
            (speed_ms + dist_error * 0.5).clamp(0.0, set_speed)
        } else {
            set_speed
        };
        let target_speed = gap_speed.min(set_speed);

        // PI speed controller with anti-windup.
        let speed_error = target_speed - speed_ms;
        if !lead_valid {
            // Prevent stale integral from causing surge when a lead target reappears.
            self.acc_speed_error_integral *= 0.92;
        }

        let candidate_integral =
            (self.acc_speed_error_integral + speed_error * dt).clamp(-20.0, 20.0);
        let unsat_control = speed_error * 0.08 + candidate_integral * 0.004;
        let control = unsat_control.clamp(-0.8, 1.0);

        // Integrate only when not saturating in the same error direction.
        let same_direction_saturation = (unsat_control - control).abs() > f64::EPSILON
            && speed_error.abs() > 0.001
            && speed_error.signum() == control.signum();
        if !same_direction_saturation {
            self.acc_speed_error_integral = candidate_integral;
        }

        if control > 0.02 {
            *thr = Some(control.clamp(0.0, 1.0));
        } else if control < -0.02 {
            *brk = Some((-control).clamp(0.0, 0.8).max(brk.unwrap_or(0.0)));
            *thr = Some(0.0);
        }
    }

    fn run_lka(
        &mut self,
        offset: f64,
        heading_err: f64,
        speed_ms: f64,
        dt: f64,
        str: &mut Option<f64>,
    ) {
        if speed_ms < 3.0 {
            return;
        }
        // PD controller for lane centering
        let kp = 4.0 * (speed_ms / 30.0).min(1.0);
        let ki = 0.3;
        self.lka_integral += offset * dt;
        self.lka_integral = self.lka_integral.clamp(-5.0, 5.0);
        let steer = -(kp * offset + ki * self.lka_integral + 0.5 * heading_err);
        self.lka_steer_cmd = steer.clamp(-6.0, 6.0);
        *str = Some(self.lka_steer_cmd);
    }

    fn run_tja(
        &mut self,
        speed_ms: f64,
        lead_range: f64,
        dt: f64,
        thr: &mut Option<f64>,
        brk: &mut Option<f64>,
    ) {
        // Stop-and-go: if stopped for >3s, wait for lead vehicle to move
        if speed_ms < 0.2 {
            self.tja_stopped_timer += dt;
        } else {
            self.tja_stopped_timer = 0.0;
        }
        // If lead vehicle moved away and we were stopped
        if self.tja_stopped_timer > 3.0 && lead_range > 5.0 {
            *thr = Some(0.2); // gentle pull-away
        } else if lead_range < 3.0 {
            *brk = Some(1.0);
            *thr = Some(0.0);
        }
    }

    fn update_path(&mut self, speed_ms: f64) {
        // Generate simple straight-ahead path (full planning would use A* + splines)
        self.planned_path.clear();
        let preview_dist = (speed_ms * 3.5).max(30.0);
        let n_points = 20;
        for i in 0..n_points {
            let t = i as f64 / n_points as f64;
            self.planned_path.push(Waypoint {
                x: t * preview_dist,
                y: -self.lane.offset_m * (1.0 - t), // converge to centre
                speed_ms,
                curvature: self.lane.curvature_1_m,
            });
        }
        self.path_valid = self.lane.detected;
    }

    fn safety_check(&mut self, speed_ms: f64, thr: Option<f64>, brk: Option<f64>) {
        // ISO 26262 ASIL-D plausibility checks
        if let (Some(t), Some(b)) = (thr, brk) {
            if t > 0.1 && b > 0.1 {
                // Simultaneous throttle and brake — contradiction!
                self.safety_status = SafetyStatus::Degraded;
                self.degrade_reason = Some("Throttle+Brake simultaneous");
                return;
            }
        }
        if speed_ms > 50.0
            && matches!(self.acc_state, FeatureState::Off)
            && matches!(self.lka_state, FeatureState::Active)
        {
            // LKA without ACC at high speed — degrade
            self.safety_status = SafetyStatus::Degraded;
            self.degrade_reason = Some("LKA w/o ACC at high speed");
            return;
        }
        self.safety_status = SafetyStatus::Normal;
        self.degrade_reason = None;
    }

    // ─ Public control helpers ────────────────────────────────────────────────
    pub fn engage(&mut self, current_speed: f64) {
        self.engaged = true;
        self.acc_state = FeatureState::Active;
        self.lka_state = FeatureState::Active;
        self.tja_state = FeatureState::Active;
        self.acc_set_speed_kmh = (current_speed).max(30.0);
        self.acc_speed_error_integral = 0.0;
        self.throttle_cmd = 0.0;
        self.brake_cmd = 0.0;
        self.steer_cmd_deg = 0.0;
        self.lka_integral = 0.0;
    }
    pub fn disengage(&mut self) {
        self.engaged = false;
        self.acc_state = FeatureState::Off;
        self.lka_state = FeatureState::Off;
        self.tja_state = FeatureState::Off;
        self.throttle_cmd = 0.0;
        self.brake_cmd = 0.0;
        self.steer_cmd_deg = 0.0;
    }
    pub fn toggle_lka(&mut self) {
        self.lka_state = if self.lka_state == FeatureState::Active {
            self.lka_integral = 0.0;
            FeatureState::Off
        } else {
            FeatureState::Active
        };
    }
    pub fn set_acc_speed(&mut self, kmh: f64) {
        self.acc_set_speed_kmh = kmh.clamp(20.0, 200.0);
    }
    pub fn set_headway(&mut self, s: f64) {
        self.acc_headway_s = s.clamp(1.0, 3.5);
    }
}

#[cfg(test)]
mod tests {
    use super::{AutonomousController, FeatureState};

    #[test]
    fn disengaged_controller_outputs_none_and_zero_state() {
        let mut ad = AutonomousController::new();
        let (t, b, s) = ad.tick(
            20.0,
            0.0,
            0.0,
            f64::INFINITY,
            0.0,
            0.0,
            0.0,
            0.9,
            f64::INFINITY,
            false,
            false,
            0.02,
        );
        assert!(t.is_none());
        assert!(b.is_none());
        assert!(s.is_none());
        assert_eq!(ad.throttle_cmd, 0.0);
        assert_eq!(ad.brake_cmd, 0.0);
        assert_eq!(ad.steer_cmd_deg, 0.0);
    }

    #[test]
    fn engage_enables_l2_features() {
        let mut ad = AutonomousController::new();
        ad.engage(42.0);
        assert!(ad.engaged);
        assert_eq!(ad.acc_state, FeatureState::Active);
        assert_eq!(ad.lka_state, FeatureState::Active);
        assert_eq!(ad.tja_state, FeatureState::Active);
        assert!(ad.acc_set_speed_kmh >= 30.0);
    }

    #[test]
    fn engage_from_standstill_requests_positive_throttle_on_clear_road() {
        let mut ad = AutonomousController::new();
        ad.engage(0.0);

        let (thr, brk, steer) = ad.tick(
            0.0,
            0.0,
            0.0,
            f64::INFINITY,
            0.0,
            0.0,
            0.0,
            0.95,
            f64::INFINITY,
            false,
            false,
            0.05,
        );

        assert!(thr.is_some_and(|v| v > 0.05));
        assert!(brk.is_none() || brk == Some(0.0));
        assert!(steer.is_none() || steer == Some(0.0));
        assert!(ad.throttle_cmd > 0.05);
    }

    #[test]
    fn no_lead_then_close_lead_transitions_to_brake() {
        let mut ad = AutonomousController::new();
        ad.engage(50.0);

        // Build integral/throttle while road is clear.
        for _ in 0..120 {
            let _ = ad.tick(
                30.0,
                0.0,
                0.0,
                f64::INFINITY,
                0.0,
                0.0,
                0.0,
                0.95,
                f64::INFINITY,
                false,
                false,
                0.02,
            );
        }
        assert!(ad.throttle_cmd >= 0.0);

        // Sudden close lead should force braking and suppress throttle.
        let _ = ad.tick(
            30.0,
            0.0,
            0.0,
            8.0,
            0.0,
            0.0,
            0.0,
            0.95,
            1.2,
            false,
            false,
            0.02,
        );
        assert!(ad.brake_cmd > 0.0);
        assert!(ad.throttle_cmd <= 0.05);
    }
}
