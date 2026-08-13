//! ABS / ESP / TCS — Electronic Stability Control ECU (SA 0x0B)
//!
//! Models a real stability control system:
//!   ABS  — Anti-lock Braking System (ISO 11992, J1939 EBC1)
//!   TCS  — Traction Control System (throttle + brake intervention)
//!   ESP  — Electronic Stability Program (yaw-rate based torque vectoring)
//!   Hill Hold — Gradient hold via brake pressure
//!   EBS  — Electronic Brake System interface
//!
//! Each of the 4 wheels has:
//!   • Speed sensor (Hall-effect, active)
//!   • Individual brake modulator (solenoid + pressure sensor)
//!   • Slip calculation relative to reference speed
//!
//! J1939: Transmits EBC1 (20 ms), EBC2 (100 ms), DM1 (1 Hz)

use crate::j1939::{self, addr, J1939Frame};

pub const ABS_SA: u8 = addr::BRAKES; // 0x0B

// ── Wheel index ───────────────────────────────────────────────────────────────
pub const FL: usize = 0;
pub const FR: usize = 1;
pub const RL: usize = 2;
pub const RR: usize = 3;

// ── Wheel Sensor State ────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorState {
    Ok,
    Erratic,
    Open,
    Shorted,
}

// ── ABS Valve State (per-wheel hydraulic modulator) ──────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AbsValveState {
    /// Normal: brake pressure follows pedal
    Normal,
    /// Hold: isolate wheel cylinder — pressure not building further
    Hold,
    /// Dump: open dump solenoid — reduce pressure rapidly
    Dump,
    /// Apply: allow pressure to build at controlled rate
    Apply,
}

impl std::fmt::Display for AbsValveState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AbsValveState::Normal => write!(f, "NORMAL"),
            AbsValveState::Hold => write!(f, "HOLD  "),
            AbsValveState::Dump => write!(f, "DUMP  "),
            AbsValveState::Apply => write!(f, "APPLY "),
        }
    }
}

// ── Per-Wheel State ───────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct Wheel {
    /// Measured wheel speed (km/h)
    pub speed: f64,
    /// Longitudinal slip ratio: (v_ref - v_wheel) / v_ref  [−1..1]
    pub slip_ratio: f64,
    /// Brake pressure at this wheel (bar)
    pub brake_pres_bar: f64,
    /// ABS modulator state
    pub valve_state: AbsValveState,
    /// Sensor health
    pub sensor_state: SensorState,
    /// True if ABS is currently cycling this wheel
    pub abs_active: bool,
    /// True if TCS is cutting traction on this wheel
    pub tcs_active: bool,
    /// Accumulated heat in brake disc (°C above ambient)
    pub brake_temp_c: f64,
    /// ABS cycle count for this wheel
    pub abs_cycles: u32,
    /// Internal ABS phase (0-1 sawtooth for pressure modulation)
    abs_phase: f64,
}

impl Default for Wheel {
    fn default() -> Self {
        Self::new()
    }
}

impl Wheel {
    pub fn new() -> Self {
        Wheel {
            speed: 0.0,
            slip_ratio: 0.0,
            brake_pres_bar: 0.0,
            valve_state: AbsValveState::Normal,
            sensor_state: SensorState::Ok,
            abs_active: false,
            tcs_active: false,
            brake_temp_c: 25.0,
            abs_cycles: 0,
            abs_phase: 0.0,
        }
    }

    pub fn is_locked(&self) -> bool {
        self.slip_ratio > 0.25 && self.speed < 1.0
    }
}

// ── ESP Condition ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EspCondition {
    Neutral,
    /// Understeer: front slides, yaw_actual < yaw_desired
    Understeer,
    /// Oversteer: rear slides, yaw_actual > yaw_desired
    Oversteer,
    /// Rollover threshold exceeded (lateral acceleration too high)
    RolloverRisk,
}

impl std::fmt::Display for EspCondition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EspCondition::Neutral => write!(f, "STABLE      "),
            EspCondition::Understeer => write!(f, "UNDERSTEER  "),
            EspCondition::Oversteer => write!(f, "OVERSTEER!  "),
            EspCondition::RolloverRisk => write!(f, "ROLLOVER!   "),
        }
    }
}

// ── ABS/ESP ECU ───────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuAbs {
    pub sa: u8,

    // ─ System states ─────────────────────────────────────────────────────────
    pub abs_system_active: bool, // any wheel is in ABS
    pub tcs_system_active: bool, // traction control intervening
    pub esp_system_active: bool, // ESP intervening
    pub abs_enabled: bool,       // user/TCM can disable (off-road mode)
    pub tcs_enabled: bool,
    pub esp_enabled: bool,

    // ─ Wheels ────────────────────────────────────────────────────────────────
    pub wheels: [Wheel; 4],

    // ─ Vehicle reference speed (vRef) — estimated from non-braking wheels ────
    pub v_ref_kmh: f64,

    // ─ Brake system ──────────────────────────────────────────────────────────
    pub master_cylinder_bar: f64, // driver pedal pressure
    pub brake_pedal_pct: f64,     // 0-100%
    pub brake_light_active: bool,
    pub park_brake_applied: bool,

    // ─ ESP sensors ───────────────────────────────────────────────────────────
    pub steering_angle_deg: f64,
    pub yaw_rate_deg_s: f64, // measured by gyro
    pub lateral_g: f64,      // measured by accelerometer
    pub longitudinal_g: f64,
    pub esp_condition: EspCondition,
    pub esp_yaw_error: f64, // actual − desired yaw rate

    // ─ TCS throttle cut output ───────────────────────────────────────────────
    /// Throttle cut requested by TCS (0=none, 1=full cut)
    pub tcs_throttle_cut: f64,
    /// Spark retard requested by TCS (unused for diesel — for reference)
    pub tcs_torque_request_nm: f64,

    // ─ Hill Hold ─────────────────────────────────────────────────────────────
    pub hill_hold_active: bool,
    pub grade_pct: f64, // estimated road grade

    // ─ Diagnostics ───────────────────────────────────────────────────────────
    pub abs_fault: bool, // system fault — ABS disabled
    pub esp_fault: bool,
    pub total_abs_events: u32,
    pub total_tcs_events: u32,

    // ─ J1939 TX timers ───────────────────────────────────────────────────────
    t_ebc1: f64,
    t_ebc2: f64,
    t_dm1: f64,
}

impl Default for EcuAbs {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuAbs {
    pub fn new() -> Self {
        EcuAbs {
            sa: ABS_SA,
            abs_system_active: false,
            tcs_system_active: false,
            esp_system_active: false,
            abs_enabled: true,
            tcs_enabled: true,
            esp_enabled: true,
            wheels: [Wheel::new(), Wheel::new(), Wheel::new(), Wheel::new()],
            v_ref_kmh: 0.0,
            master_cylinder_bar: 0.0,
            brake_pedal_pct: 0.0,
            brake_light_active: false,
            park_brake_applied: false,
            steering_angle_deg: 0.0,
            yaw_rate_deg_s: 0.0,
            lateral_g: 0.0,
            longitudinal_g: 0.0,
            esp_condition: EspCondition::Neutral,
            esp_yaw_error: 0.0,
            tcs_throttle_cut: 0.0,
            tcs_torque_request_nm: 0.0,
            hill_hold_active: false,
            grade_pct: 0.0,
            abs_fault: false,
            esp_fault: false,
            total_abs_events: 0,
            total_tcs_events: 0,
            t_ebc1: 0.0,
            t_ebc2: 0.0,
            t_dm1: 0.0,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Main tick. Call with current vehicle state.
    /// Returns throttle cut fraction (0-1) and J1939 frames.
    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        vehicle_speed_kmh: f64,
        brake_pct: f64,
        throttle_pct: f64,
        steering_deg: f64,
        yaw_rate_deg_s: f64,
        lateral_g: f64,
        dt: f64,
    ) -> (f64, Vec<J1939Frame>) {
        self.brake_pedal_pct = brake_pct.clamp(0.0, 100.0);
        self.steering_angle_deg = steering_deg;
        self.yaw_rate_deg_s = yaw_rate_deg_s;
        self.lateral_g = lateral_g;
        self.brake_light_active = brake_pct > 2.0;

        // Heavy machine hydraulic brakes: up to 220 bar peak line pressure
        self.master_cylinder_bar = (brake_pct / 100.0 * 220.0).min(220.0);

        // Estimate vehicle reference speed (best two non-locked wheels)
        self.update_v_ref(vehicle_speed_kmh);

        // Simulate wheel speeds given vehicle speed and braking
        self.update_wheel_speeds(vehicle_speed_kmh, dt);

        // ABS logic — per-wheel
        self.run_abs(dt);

        // TCS — detect wheel spin, cut throttle
        let tcs_cut = self.run_tcs(throttle_pct, vehicle_speed_kmh, dt);

        // ESP — detect over/understeer, apply differential braking
        self.run_esp(
            vehicle_speed_kmh,
            steering_deg,
            yaw_rate_deg_s,
            lateral_g,
            dt,
        );

        // Hill hold
        self.run_hill_hold(vehicle_speed_kmh, dt);

        // Brake disc thermal model
        for w in &mut self.wheels {
            let braking_heat = w.brake_pres_bar * (vehicle_speed_kmh / 3.6) * 0.005;
            let cooling = (w.brake_temp_c - 25.0) * 0.02;
            w.brake_temp_c += (braking_heat - cooling) * dt;
            w.brake_temp_c = w.brake_temp_c.clamp(25.0, 900.0);
        }

        // System-level flags
        let was_abs_active = self.abs_system_active;
        self.abs_system_active = self.wheels.iter().any(|w| w.abs_active);
        self.tcs_system_active = self.wheels.iter().any(|w| w.tcs_active);
        // Count ABS events only on rising edge (not every tick)
        if !was_abs_active && self.abs_system_active {
            self.total_abs_events += 1;
        }

        // Periodic J1939 TX
        self.t_ebc1 += dt;
        self.t_ebc2 += dt;
        self.t_dm1 += dt;
        let ts = vehicle_speed_kmh;
        let mut frames: Vec<J1939Frame> = Vec::new();

        if self.t_ebc1 >= 0.020 {
            self.t_ebc1 = 0.0;
            frames.push(self.build_ebc1(ts));
        }
        if self.t_ebc2 >= 0.100 {
            self.t_ebc2 = 0.0;
            frames.push(self.build_ebc2(ts));
        }
        if self.t_dm1 >= 1.000 {
            self.t_dm1 = 0.0;
            if self.abs_fault || self.esp_fault {
                frames.push(self.build_dm1_fault(ts));
            }
        }

        (tcs_cut, frames)
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn update_v_ref(&mut self, vehicle_speed: f64) {
        // Derive v_ref from fastest two non-braking wheels (robust to individual wheel lockout)
        let mut speeds: Vec<f64> = self.wheels.iter()
            .filter(|w| !w.abs_active && w.sensor_state == SensorState::Ok)
            .map(|w| w.speed)
            .collect();
        speeds.sort_by(|a, b| b.total_cmp(a));
        let sensor_ref = if speeds.len() >= 2 {
            (speeds[0] + speeds[1]) / 2.0
        } else if speeds.len() == 1 {
            speeds[0]
        } else {
            vehicle_speed // fallback to TCM speed if all wheels locked
        };
        // Lag to represent sensor filtering and CAN message delay
        self.v_ref_kmh += (sensor_ref - self.v_ref_kmh) * 0.6;
    }

    fn update_wheel_speeds(&mut self, vehicle_speed: f64, dt: f64) {
        // Wheel rotational inertia (per wheel, heavy machine axle ~6 kg·m² effective)
        const J_WHEEL: f64 = 6.0;
        const TIRE_R: f64 = 0.655;
        const MU_ROAD: f64 = 0.8; // dry tarmac peak slip coefficient
        const F_NORMAL_N: f64 = 50_000.0; // ~20 t / 4 wheels, simplified

        for w in self.wheels.iter_mut() {
            // Convert wheel speed km/h → angular velocity rad/s
            let omega = w.speed / 3.6 / TIRE_R;
            // Brake torque = pressure × caliper area × radius (simplified: 8 Nm/bar)
            let brake_torque = w.brake_pres_bar * 8.0;
            // Traction torque (capped by friction limit)
            let friction_torque = MU_ROAD * F_NORMAL_N * TIRE_R;
            let net_torque = -brake_torque.min(friction_torque);
            let omega_dot = net_torque / J_WHEEL;
            let new_omega = (omega + omega_dot * dt).max(0.0);
            w.speed = new_omega * TIRE_R * 3.6;

            if w.abs_active {
                // ABS dump phase — wheel recovers toward reference
                let recover_rate = 50.0 * dt;
                w.speed = (w.speed + recover_rate).min(self.v_ref_kmh);
            }

            w.slip_ratio = if vehicle_speed > 1.0 {
                (vehicle_speed - w.speed) / vehicle_speed
            } else {
                0.0
            };
        }
    }

    fn run_abs(&mut self, dt: f64) {
        if !self.abs_enabled || self.abs_fault {
            return;
        }
        let v_ref = self.v_ref_kmh;

        for w in &mut self.wheels {
            // Threshold: slip > 20% → wheel is locking up
            let slip_exceeded = w.slip_ratio > 0.20 && v_ref > 5.0;
            let pres = self.master_cylinder_bar;

            if !w.abs_active && slip_exceeded {
                w.abs_active = true;
                w.abs_cycles += 1;
                w.abs_phase = 0.0;
            }

            if w.abs_active {
                // 10 Hz ABS cycle: dump → hold → apply
                w.abs_phase += dt * 10.0;
                if w.abs_phase > 1.0 {
                    w.abs_phase -= 1.0;
                }

                w.valve_state = if w.abs_phase < 0.30 {
                    w.brake_pres_bar = (w.brake_pres_bar - 40.0 * dt).max(0.0);
                    AbsValveState::Dump
                } else if w.abs_phase < 0.55 {
                    AbsValveState::Hold
                } else {
                    w.brake_pres_bar = (w.brake_pres_bar + 60.0 * dt).min(pres);
                    AbsValveState::Apply
                };

                // Exit ABS: ramp pressure back (apply phase), do not snap to full pressure
                if w.slip_ratio < 0.05 && w.speed > v_ref * 0.95 {
                    w.abs_active = false;
                    w.valve_state = AbsValveState::Apply; // gradual re-application
                }
            } else {
                w.valve_state = AbsValveState::Normal;
                w.brake_pres_bar = pres;
            }
        }
    }

    fn run_tcs(&mut self, throttle_pct: f64, vehicle_speed: f64, _dt: f64) -> f64 {
        if !self.tcs_enabled {
            return 0.0;
        }
        self.tcs_throttle_cut = 0.0;

        // TCS active up to 70 km/h (heavy machines need traction on slopes and soft ground)
        if vehicle_speed > 70.0 {
            return 0.0;
        }

        let driven_avg = (self.wheels[RL].speed + self.wheels[RR].speed) / 2.0;
        let spin_excess = driven_avg - vehicle_speed;

        if spin_excess > 3.0 && throttle_pct > 20.0 {
            let cut = ((spin_excess - 3.0) / 10.0).clamp(0.0, 0.8);
            self.tcs_throttle_cut = cut;
            self.wheels[RL].tcs_active = true;
            self.wheels[RR].tcs_active = true;
            self.tcs_torque_request_nm = cut * 200.0; // request ECM to reduce torque
            if self.tcs_system_active {
                self.total_tcs_events += 1;
            }
        } else {
            self.wheels[RL].tcs_active = false;
            self.wheels[RR].tcs_active = false;
        }
        self.tcs_throttle_cut
    }

    fn run_esp(&mut self, speed: f64, steer_deg: f64, yaw_actual: f64, lat_g: f64, _dt: f64) {
        if !self.esp_enabled || speed < 15.0 {
            self.esp_condition = EspCondition::Neutral;
            return;
        }

        // Desired yaw rate from bicycle model: v * steer / (L * (1 + Kus * v²))
        // L=2.7m, Kus=0.005 (understeer gradient)
        let l = 2.7;
        let kus = 0.005;
        let v_ms = speed / 3.6;
        let steer_rad = steer_deg.to_radians();
        let yaw_desired = v_ms * steer_rad / (l * (1.0 + kus * v_ms * v_ms));
        let yaw_desired_degs = yaw_desired.to_degrees();

        self.esp_yaw_error = yaw_actual - yaw_desired_degs;
        let err = self.esp_yaw_error.abs();

        // Rollover detection: 0.4g threshold for high-CG heavy machinery (would roll at 0.35-0.45g)
        self.esp_condition = if lat_g.abs() > 0.4 {
            self.esp_system_active = true;
            EspCondition::RolloverRisk
        } else if err > 8.0 && yaw_actual > yaw_desired_degs {
            self.esp_system_active = true;
            EspCondition::Oversteer
        } else if err > 8.0 && yaw_actual < yaw_desired_degs {
            self.esp_system_active = true;
            EspCondition::Understeer
        } else {
            self.esp_system_active = false;
            EspCondition::Neutral
        };
    }

    fn run_hill_hold(&mut self, speed: f64, _dt: f64) {
        // Activate hill hold if vehicle speed drops to 0 while brake applied
        if speed < 0.5 && self.master_cylinder_bar > 10.0 {
            self.hill_hold_active = true;
        } else if speed > 2.0 {
            self.hill_hold_active = false;
        }
    }

    // ─ J1939 frame builders ──────────────────────────────────────────────────
    fn build_ebc1(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        // SPN 561: ABS Control Active (bits 0-1)
        data[0] = if self.abs_system_active { 0x01 } else { 0x00 };
        // SPN 562: ABS off-road switch
        data[0] |= if !self.abs_enabled { 0x04 } else { 0x00 };
        // SPN 563: ASR (TCS) brake control
        data[0] |= if self.tcs_system_active { 0x10 } else { 0x00 };
        // SPN 1121: ESP active
        data[1] = if self.esp_system_active { 0x01 } else { 0x00 };
        // SPN 521: Front axle brake demand (% of master)
        data[2] = (self.master_cylinder_bar / 150.0 * 250.0) as u8;
        // SPN 522: Rear axle brake demand
        data[3] = data[2];
        // Brake lamps
        data[4] = if self.brake_light_active { 0x03 } else { 0x00 };
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(2, j1939::pgn::EBC1, self.sa, 0xFF),
            &data,
        )
    }

    fn build_ebc2(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        // SPN 904: Front left wheel speed (0.125 km/h/bit)
        let fl = (self.wheels[FL].speed / 0.125) as u16;
        let fr = (self.wheels[FR].speed / 0.125) as u16;
        let rl = (self.wheels[RL].speed / 0.125) as u16;
        let rr = (self.wheels[RR].speed / 0.125) as u16;
        data[0] = (fl & 0xFF) as u8;
        data[1] = (fl >> 8) as u8;
        data[2] = (fr & 0xFF) as u8;
        data[3] = (fr >> 8) as u8;
        data[4] = (rl & 0xFF) as u8;
        data[5] = (rl >> 8) as u8;
        data[6] = (rr & 0xFF) as u8;
        data[7] = (rr >> 8) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, 65215, self.sa, 0xFF), &data)
    }

    fn build_dm1_fault(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        data[0] = 0x04; // Amber warning
        data[1] = 0xFF;
        // SPN 9002 = ABS wheel speed sensor fault, FMI 9 = abnormal update rate
        data[2] = 0x2A;
        data[3] = 0x23;
        data[4] = 0x48;
        data[5] = 0x01;
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(6, j1939::pgn::DM1, self.sa, 0xFF),
            &data,
        )
    }

    // ─ Convenience ───────────────────────────────────────────────────────────
    pub fn toggle_abs(&mut self) {
        self.abs_enabled = !self.abs_enabled;
    }
    pub fn toggle_tcs(&mut self) {
        self.tcs_enabled = !self.tcs_enabled;
    }
    pub fn toggle_esp(&mut self) {
        self.esp_enabled = !self.esp_enabled;
    }

    pub fn any_wheel_abs_active(&self) -> bool {
        self.wheels.iter().any(|w| w.abs_active)
    }
}
