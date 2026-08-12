//! Transmission Control Module (TCM) — Powershift 16F/16R + optional IVT/CVT.
//!
//! Models a real agricultural powershift transmission:
//!   • 4 ranges (A=crawl, B=field-low, C=field-high, D=road)
//!   • 4 powershift speeds per range = 16 forward / 16 reverse
//!   • Wet multi-disc powershift clutches (P1–P4) with fill, modulate, lock phases
//!   • Electronic shuttle (direction change under load)
//!   • Auto-shift based on engine load and RPM
//!   • Creeper gear for slow tillage work
//!   • Ground Speed Management (GSM): maintains target speed regardless of load
//!
//! J1939 SA: 0x03, Transmits ETC1 (20ms), ETC2 (20ms).

use crate::j1939::{self, addr, J1939Frame};

pub const TCM_SA: u8 = addr::TRANSMISSION; // 0x03

// ── Transmission Type ────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransmissionType {
    /// Traditional 16F/16R powershift
    Powershift,
    /// Continuously/Infinitely Variable (e.g. John Deere IVT, Case CVX)
    /// Hydrostatic + mechanical power-split — zero to max speed, no ratio steps
    IVT,
}

// ── Gear Range ───────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GearRange {
    /// Crawl: 0.3–4 km/h — deep tillage, PTO work
    A,
    /// Field Low: 4–8 km/h — normal tillage
    B,
    /// Field High: 8–18 km/h — light field work, transport
    C,
    /// Road: 18–50 km/h — road transport
    D,
}

impl std::fmt::Display for GearRange {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GearRange::A => "A",
            GearRange::B => "B",
            GearRange::C => "C",
            GearRange::D => "D",
        }
        .fmt(f)
    }
}

// ── Shuttle/Direction ────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Forward,
    Reverse,
    Neutral,
    Park,
}

impl std::fmt::Display for Direction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Direction::Forward => write!(f, "F"),
            Direction::Reverse => write!(f, "R"),
            Direction::Neutral => write!(f, "N"),
            Direction::Park => write!(f, "P"),
        }
    }
}

// ── Clutch State ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClutchState {
    /// Clutch piston bore filling with oil (~80–200 ms)
    Filling,
    /// Clutch plates touching — modulation phase for smooth engagement
    Modulating,
    /// Fully locked — zero slip
    Locked,
    /// Disengaged — no torque transfer
    Open,
}

impl std::fmt::Display for ClutchState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClutchState::Filling => write!(f, "FILLING "),
            ClutchState::Modulating => write!(f, "MODULATG"),
            ClutchState::Locked => write!(f, "LOCKED  "),
            ClutchState::Open => write!(f, "OPEN    "),
        }
    }
}

// ── Shift Quality ────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub enum ShiftQuality {
    Smooth,
    Normal,
    Harsh,
}

impl std::fmt::Display for ShiftQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ShiftQuality::Smooth => "SMOOTH",
            ShiftQuality::Normal => "NORMAL",
            ShiftQuality::Harsh => "HARSH!",
        }
        .fmt(f)
    }
}

// ── Auto-Shift Mode ──────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AutoShiftMode {
    /// TCM picks gear automatically
    Auto,
    /// Operator sets gear; TCM prevents engine stall only
    Manual,
    /// Hold current gear — useful at headlands
    Hold,
}

// ── Gear ratios: [range][speed 1-4] = overall ratio (incl. final drive ×3.82) ─
const GEAR_RATIOS: [[f64; 4]; 4] = [
    // A: 0.3–4 km/h
    [120.0, 85.0, 60.0, 42.0],
    // B: 4–8 km/h
    [32.0, 22.0, 16.0, 11.0],
    // C: 8–18 km/h
    [9.0, 6.5, 4.8, 3.5],
    // D: 18–50 km/h
    [2.8, 2.1, 1.6, 1.2],
];

const TIRE_RADIUS_M: f64 = 0.80; // rear tyre radius (metre)
const MAX_SPEED_KMH: f64 = 50.0;

// ── TCM ──────────────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuTcm {
    pub sa: u8,
    pub trans_type: TransmissionType,

    // ─ Gear position ─────────────────────────────────────────────────────────
    pub range: GearRange,
    pub speed_step: u8, // 1-4 within the range
    pub direction: Direction,
    pub gear_label: String, // "D3", "B2", "R-C1", etc.

    // ─ Output shaft ──────────────────────────────────────────────────────────
    pub output_shaft_rpm: f64,
    pub output_torque_nm: f64,
    pub ground_speed_kmh: f64,
    pub gear_ratio: f64, // current overall ratio

    // ─ Clutch pack ──────────────────────────────────────────────────────────
    pub clutch_state: ClutchState,
    pub clutch_slip_pct: f64,
    pub clutch_temp_c: f64,
    pub clutch_fill_pres: f64, // fill pressure (bar)
    clutch_phase_timer: f64,

    // ─ Shifting ──────────────────────────────────────────────────────────────
    pub is_shifting: bool,
    pub last_shift_qual: ShiftQuality,
    pub total_shifts: u32,
    pub pending_range: Option<GearRange>,
    pub pending_speed: Option<u8>,
    pub pending_dir: Option<Direction>,
    shift_timer: f64,
    /// Duration of current shift phase (varies with temperature & quality)
    shift_duration: f64,

    // ─ Auto-shift ────────────────────────────────────────────────────────────
    pub auto_mode: AutoShiftMode,
    /// Target ground speed for GSM (km/h)
    pub gsm_target_kmh: f64,
    pub gsm_enabled: bool,

    // ─ IVT / CVT (used when trans_type = IVT) ───────────────────────────────
    /// IVT ratio: 0 = full reverse, 0.5 = zero mechanical out, 1 = max fwd
    pub ivt_ratio: f64,

    // ─ Creeper ───────────────────────────────────────────────────────────────
    pub creeper_engaged: bool, // additional ÷4 reduction
    pub creeper_ratio: f64,

    // ─ Thermal ───────────────────────────────────────────────────────────────
    pub oil_temp_c: f64,
    pub sump_temp_c: f64,

    // ─ Safety interlocks ────────────────────────────────────────────────────
    pub is_neutral: bool,          // read by boot sequence for crank inhibit
    pub handbrake_interlock: bool, // prevent direction change above speed threshold

    // ─ J1939 TX timers ───────────────────────────────────────────────────────
    t_etc1: f64,
    t_etc2: f64,

    // ─ Stats ─────────────────────────────────────────────────────────────────
    pub total_distance_km: f64,
}

impl EcuTcm {
    pub fn new() -> Self {
        EcuTcm {
            sa: TCM_SA,
            trans_type: TransmissionType::Powershift,
            range: GearRange::B,
            speed_step: 1,
            direction: Direction::Neutral,
            gear_label: "N".into(),
            output_shaft_rpm: 0.0,
            output_torque_nm: 0.0,
            ground_speed_kmh: 0.0,
            gear_ratio: GEAR_RATIOS[1][0],
            clutch_state: ClutchState::Open,
            clutch_slip_pct: 0.0,
            clutch_temp_c: 30.0,
            clutch_fill_pres: 0.0,
            clutch_phase_timer: 0.0,
            is_shifting: false,
            last_shift_qual: ShiftQuality::Normal,
            total_shifts: 0,
            pending_range: None,
            pending_speed: None,
            pending_dir: None,
            shift_timer: 0.0,
            shift_duration: 0.30,
            auto_mode: AutoShiftMode::Auto,
            gsm_target_kmh: 8.0,
            gsm_enabled: false,
            ivt_ratio: 0.5,
            creeper_engaged: false,
            creeper_ratio: 4.0,
            oil_temp_c: 30.0,
            sump_temp_c: 30.0,
            is_neutral: true,
            handbrake_interlock: false,
            t_etc1: 0.0,
            t_etc2: 0.0,
            total_distance_km: 0.0,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Main TCM tick. `engine_rpm` and `engine_torque_nm` from ECM.
    /// Returns J1939 frames to transmit.
    pub fn tick(
        &mut self,
        engine_rpm: f64,
        engine_torque_nm: f64,
        throttle_pct: f64,
        brake_pct: f64,
        dt: f64,
    ) -> Vec<J1939Frame> {
        // ─ Resolve current gear ratio ────────────────────────────────────────
        let ri = self.range as usize;
        let si = (self.speed_step as usize).saturating_sub(1).min(3);
        self.gear_ratio = GEAR_RATIOS[ri][si]
            * if self.creeper_engaged {
                self.creeper_ratio
            } else {
                1.0
            };

        // ─ IVT: continuously vary ratio toward target speed ──────────────────
        if self.trans_type == TransmissionType::IVT {
            self.update_ivt(engine_rpm, throttle_pct, dt);
        }

        // ─ Output shaft speed and ground speed ───────────────────────────────
        self.output_shaft_rpm =
            if self.direction == Direction::Neutral || self.direction == Direction::Park {
                0.0
            } else {
                engine_rpm / self.gear_ratio
            };

        // v = ω_out × r_tyre × 3.6 (convert m/s → km/h)
        self.ground_speed_kmh =
            (self.output_shaft_rpm / 60.0 * 2.0 * std::f64::consts::PI * TIRE_RADIUS_M * 3.6)
                .clamp(0.0, MAX_SPEED_KMH);

        // ─ Output torque amplified by gear ratio (efficiency ~0.96) ─────────
        self.output_torque_nm = engine_torque_nm * self.gear_ratio * 0.96;

        // ─ Clutch dynamics ────────────────────────────────────────────────────
        self.update_clutch(engine_rpm, engine_torque_nm, dt);

        // ─ Auto-shift logic ──────────────────────────────────────────────────
        if !self.is_shifting
            && self.auto_mode == AutoShiftMode::Auto
            && self.direction == Direction::Forward
        {
            self.auto_shift_logic(engine_rpm, throttle_pct, dt);
        }

        // ─ Shift completion ──────────────────────────────────────────────────
        self.update_shift(dt);

        // ─ Thermal model ─────────────────────────────────────────────────────
        let heat = self.clutch_slip_pct * engine_torque_nm * 0.001;
        let tgt_oil = 70.0 + heat * 0.5 + throttle_pct * 0.2;
        self.oil_temp_c += (tgt_oil - self.oil_temp_c) * dt * 0.005;
        self.sump_temp_c = self.oil_temp_c - 5.0;

        // ─ Distance accumulation ─────────────────────────────────────────────
        self.total_distance_km += self.ground_speed_kmh * dt / 3600.0;

        // ─ Safety flags ──────────────────────────────────────────────────────
        self.is_neutral = self.direction == Direction::Neutral || self.direction == Direction::Park;
        self.handbrake_interlock = self.ground_speed_kmh > 3.0
            && (self.direction == Direction::Forward
                && self.pending_dir == Some(Direction::Reverse)
                || self.direction == Direction::Reverse
                    && self.pending_dir == Some(Direction::Forward));

        // ─ Gear label ────────────────────────────────────────────────────────
        self.gear_label = self.compute_gear_label();

        // ─ Brake: if significant braking, disengage clutch ──────────────────
        if brake_pct > 0.3 && self.clutch_state == ClutchState::Locked {
            self.clutch_state = ClutchState::Open;
        }

        // ─ J1939 periodic TX ─────────────────────────────────────────────────
        self.t_etc1 += dt;
        self.t_etc2 += dt;
        let ts = self.total_distance_km; // monotonic timestamp approximation
        let mut frames: Vec<J1939Frame> = Vec::new();

        if self.t_etc1 >= 0.020 {
            self.t_etc1 = 0.0;
            frames.push(self.build_etc1(ts));
        }
        if self.t_etc2 >= 0.020 {
            self.t_etc2 = 0.0;
            frames.push(self.build_etc2(ts));
        }
        frames
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn update_clutch(&mut self, _engine_rpm: f64, torque_nm: f64, dt: f64) {
        match self.clutch_state {
            ClutchState::Open => {
                self.clutch_slip_pct = 100.0;
                self.clutch_fill_pres = 0.0;
            }
            ClutchState::Filling => {
                self.clutch_phase_timer += dt;
                // Fill ramp 0 → 10 bar in 80-150 ms (faster when warm)
                let fill_time = (0.15 - self.oil_temp_c.min(80.0) / 80.0 * 0.07).max(0.05);
                self.clutch_fill_pres = (self.clutch_phase_timer / fill_time * 10.0).min(10.0);
                if self.clutch_phase_timer >= fill_time {
                    self.clutch_state = ClutchState::Modulating;
                    self.clutch_phase_timer = 0.0;
                }
                self.clutch_slip_pct = 80.0;
            }
            ClutchState::Modulating => {
                self.clutch_phase_timer += dt;
                // Modulate pressure 10 → 25 bar over ~250 ms, reducing slip
                let mod_time = 0.25;
                let pres_ramp = self.clutch_phase_timer / mod_time;
                self.clutch_fill_pres = 10.0 + pres_ramp * 15.0;
                self.clutch_slip_pct = (80.0 * (1.0 - pres_ramp)).max(0.0);
                if self.clutch_phase_timer >= mod_time {
                    self.clutch_state = ClutchState::Locked;
                    self.clutch_slip_pct = 0.0;
                    self.clutch_fill_pres = 25.0;
                }
                // Clutch temp from slip energy
                self.clutch_temp_c += self.clutch_slip_pct * torque_nm * 0.00005;
            }
            ClutchState::Locked => {
                self.clutch_fill_pres = 25.0;
                self.clutch_slip_pct = 0.0;
                self.clutch_temp_c = (self.clutch_temp_c - 5.0 * dt).max(self.oil_temp_c);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn auto_shift_logic(&mut self, engine_rpm: f64, throttle_pct: f64, _dt: f64) {
        // Upshift: engine RPM above rated AND throttle not floored
        let upshift_rpm = match self.range {
            GearRange::A | GearRange::B => 2100.0,
            GearRange::C | GearRange::D => 2000.0,
        };
        let downshift_rpm = 1000.0;

        if engine_rpm > upshift_rpm && throttle_pct < 95.0 {
            self.request_upshift();
        } else if engine_rpm < downshift_rpm {
            self.request_downshift();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn request_upshift(&mut self) {
        if self.speed_step < 4 {
            self.begin_shift(self.range, self.speed_step + 1, self.direction);
        } else {
            // Cross-range upshift
            let next_range = match self.range {
                GearRange::A => Some(GearRange::B),
                GearRange::B => Some(GearRange::C),
                GearRange::C => Some(GearRange::D),
                GearRange::D => None,
            };
            if let Some(r) = next_range {
                self.begin_shift(r, 1, self.direction);
            }
        }
    }

    fn request_downshift(&mut self) {
        if self.speed_step > 1 {
            self.begin_shift(self.range, self.speed_step - 1, self.direction);
        } else {
            let prev_range = match self.range {
                GearRange::B => Some(GearRange::A),
                GearRange::C => Some(GearRange::B),
                GearRange::D => Some(GearRange::C),
                GearRange::A => None,
            };
            if let Some(r) = prev_range {
                self.begin_shift(r, 4, self.direction);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn begin_shift(&mut self, new_range: GearRange, new_speed: u8, new_dir: Direction) {
        if self.is_shifting {
            return;
        }
        if self.handbrake_interlock {
            return;
        }
        self.pending_range = Some(new_range);
        self.pending_speed = Some(new_speed);
        self.pending_dir = Some(new_dir);
        self.is_shifting = true;
        self.shift_timer = 0.0;
        // Shift duration: longer for range change, shorter for speed-step
        let range_change = new_range != self.range || new_dir != self.direction;
        self.shift_duration = if range_change { 0.45 } else { 0.25 };
        // Disengage clutch to start shift
        self.clutch_state = ClutchState::Open;
        self.clutch_phase_timer = 0.0;
        self.total_shifts += 1;
    }

    fn update_shift(&mut self, dt: f64) {
        if !self.is_shifting {
            return;
        }
        self.shift_timer += dt;
        // Mid-shift: engage new gear at ~50% of shift time
        if self.shift_timer >= self.shift_duration * 0.5 {
            if let (Some(r), Some(s), Some(d)) =
                (self.pending_range, self.pending_speed, self.pending_dir)
            {
                self.range = r;
                self.speed_step = s.clamp(1, 4);
                self.direction = d;
                self.pending_range = None;
                self.pending_speed = None;
                self.pending_dir = None;
            }
            // Start filling new clutch
            if self.clutch_state == ClutchState::Open {
                self.clutch_state = ClutchState::Filling;
                self.clutch_phase_timer = 0.0;
            }
        }
        if self.shift_timer >= self.shift_duration {
            self.is_shifting = false;
            // Assess shift quality by clutch temperature rise
            self.last_shift_qual = if self.clutch_temp_c > 150.0 {
                ShiftQuality::Harsh
            } else if self.clutch_temp_c > 100.0 {
                ShiftQuality::Normal
            } else {
                ShiftQuality::Smooth
            };
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn update_ivt(&mut self, _engine_rpm: f64, throttle_pct: f64, dt: f64) {
        // IVT: target ratio → target_speed = throttle_pct / 100 * MAX_SPEED
        let target_spd = throttle_pct / 100.0 * MAX_SPEED_KMH;
        let current_spd = self.ground_speed_kmh;
        let spd_err = target_spd - current_spd;
        // Vary IVT ratio to close speed error
        let ratio_change = (spd_err * 0.01).clamp(-0.05, 0.05);
        self.ivt_ratio = (self.ivt_ratio + ratio_change * dt).clamp(0.0, 1.0);
        // Effective gear ratio: 0 ratio → infinite, 1 ratio → direct
        if self.ivt_ratio > 0.01 {
            self.gear_ratio = 50.0 / (self.ivt_ratio * 50.0);
        } else {
            self.gear_ratio = 999.0;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Manual gear controls (called from main.rs keyboard input)
    pub fn manual_upshift(&mut self) {
        if self.auto_mode != AutoShiftMode::Auto {
            self.request_upshift();
        }
    }
    pub fn manual_downshift(&mut self) {
        if self.auto_mode != AutoShiftMode::Auto {
            self.request_downshift();
        }
    }
    pub fn set_direction(&mut self, dir: Direction) {
        self.begin_shift(self.range, self.speed_step, dir);
    }
    pub fn set_neutral(&mut self) {
        self.direction = Direction::Neutral;
        self.is_neutral = true;
    }
    pub fn toggle_creeper(&mut self) {
        self.creeper_engaged = !self.creeper_engaged;
    }
    pub fn toggle_auto(&mut self) {
        self.auto_mode = if self.auto_mode == AutoShiftMode::Auto {
            AutoShiftMode::Manual
        } else {
            AutoShiftMode::Auto
        };
    }

    fn compute_gear_label(&self) -> String {
        match self.direction {
            Direction::Neutral => "N".into(),
            Direction::Park => "P".into(),
            Direction::Forward => format!("{}{}", self.range, self.speed_step),
            Direction::Reverse => format!("R-{}{}", self.range, self.speed_step),
        }
    }

    // ─ J1939 frame builders ──────────────────────────────────────────────────
    fn build_etc1(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        // SPN 560: Transmission driveline engaged (bits 0-1)
        data[0] = if self.direction != Direction::Neutral {
            0x01
        } else {
            0x00
        };
        // SPN 573: Transmission torque converter lockup engaged
        data[0] |= if self.clutch_state == ClutchState::Locked {
            0x04
        } else {
            0x00
        };
        // SPN 191: Transmission Output Shaft Speed (0.125 rpm/bit)
        let oss = (self.output_shaft_rpm / 0.125) as u16;
        data[3] = (oss & 0xFF) as u8;
        data[4] = (oss >> 8) as u8;
        // SPN 161: Input shaft speed
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(3, j1939::pgn::ETC1, self.sa, 0xFF),
            &data,
        )
    }

    fn build_etc2(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        // SPN 524: Transmission Selected Gear
        let gear_byte = match self.direction {
            Direction::Neutral => 125u8,
            Direction::Park => 126u8,
            Direction::Forward => (self.range as u8 * 4 + self.speed_step) + 125,
            Direction::Reverse => (125u8).saturating_sub(self.range as u8 * 4 + self.speed_step),
        };
        data[3] = gear_byte;
        // SPN 523: Transmission Actual Gear
        data[4] = gear_byte;
        // SPN 525: Transmission Current Range
        data[7] = self.range as u8;
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(3, j1939::pgn::ETC2, self.sa, 0xFF),
            &data,
        )
    }
}
