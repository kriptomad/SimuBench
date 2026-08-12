#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriveMode {
    Park,
    Reverse,
    Neutral,
    Drive,
    Sport,
    Eco,
}

impl std::fmt::Display for DriveMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            DriveMode::Park => "P",
            DriveMode::Reverse => "R",
            DriveMode::Neutral => "N",
            DriveMode::Drive => "D",
            DriveMode::Sport => "S",
            DriveMode::Eco => "E",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShiftEvent {
    None,
    Up,
    Down,
}

#[derive(Debug, Clone)]
pub struct TransmissionEcu {
    pub gear: u8, // 1-8
    pub mode: DriveMode,
    pub is_shifting: bool,
    pub shift_timer: f64,
    pub last_shift: ShiftEvent,
    pub tcu_temp: f64,
    pub torque_converter_slip: f64, // 0-1
    pub oil_temp: f64,
}

// [gear index 0..7] → upshift threshold RPM by mode
const UP_DRIVE: [f64; 8] = [0.0, 3100.0, 3200.0, 3200.0, 3100.0, 3000.0, 3000.0, 3000.0];
const UP_SPORT: [f64; 8] = [0.0, 5800.0, 5800.0, 5800.0, 5800.0, 5800.0, 5800.0, 5800.0];
const UP_ECO: [f64; 8] = [0.0, 2100.0, 2200.0, 2200.0, 2000.0, 1900.0, 1900.0, 1900.0];
const DOWN_THR: f64 = 1300.0; // downshift below this RPM

impl TransmissionEcu {
    pub fn new() -> Self {
        Self {
            gear: 1,
            mode: DriveMode::Drive,
            is_shifting: false,
            shift_timer: 0.0,
            last_shift: ShiftEvent::None,
            tcu_temp: 25.0,
            torque_converter_slip: 0.0,
            oil_temp: 25.0,
        }
    }

    pub fn set_mode(&mut self, mode: DriveMode) {
        self.mode = mode;
        match mode {
            DriveMode::Drive | DriveMode::Eco | DriveMode::Sport => {
                if self.gear == 0 {
                    self.gear = 1;
                }
            }
            DriveMode::Park | DriveMode::Neutral => {}
            DriveMode::Reverse => self.gear = 0,
        }
    }

    pub fn update(&mut self, rpm: f64, throttle: f64, speed_kmh: f64, dt: f64) {
        // Shift cooldown
        if self.is_shifting {
            self.shift_timer -= dt;
            if self.shift_timer <= 0.0 {
                self.is_shifting = false;
                self.last_shift = ShiftEvent::None;
            }
            return;
        }

        if !matches!(
            self.mode,
            DriveMode::Drive | DriveMode::Sport | DriveMode::Eco
        ) {
            return;
        }
        if speed_kmh < 2.0 {
            return;
        }

        let gi = (self.gear as usize).saturating_sub(1).min(7);
        let upshift_rpm = match self.mode {
            DriveMode::Sport => UP_SPORT[gi],
            DriveMode::Eco => UP_ECO[gi],
            _ => UP_DRIVE[gi],
        };

        if rpm > upshift_rpm && self.gear < 8 {
            self.gear += 1;
            self.begin_shift(ShiftEvent::Up);
        } else if rpm < DOWN_THR && throttle > 0.1 && self.gear > 1 {
            self.gear -= 1;
            self.begin_shift(ShiftEvent::Down);
        }

        // TC slip is highest at low speeds / high throttle
        self.torque_converter_slip =
            ((1.0 - (speed_kmh / 40.0).min(1.0)) * throttle * 0.35).max(0.0);

        let tcu_target = 75.0 + throttle * 20.0;
        self.tcu_temp += (tcu_target - self.tcu_temp) * dt * 0.008;
        self.oil_temp = self.tcu_temp - 2.0;
    }

    fn begin_shift(&mut self, event: ShiftEvent) {
        self.is_shifting = true;
        self.last_shift = event;
        // Sport shifts faster (~150ms), Drive ~220ms, Eco ~280ms
        self.shift_timer = match self.mode {
            DriveMode::Sport => 0.15,
            DriveMode::Eco => 0.28,
            _ => 0.22,
        };
    }

    pub fn gear_label(&self) -> String {
        match self.mode {
            DriveMode::Park => "P".into(),
            DriveMode::Reverse => "R".into(),
            DriveMode::Neutral => "N".into(),
            _ => format!("{}{}", self.mode, self.gear),
        }
    }
}
