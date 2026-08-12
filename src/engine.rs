// ZF 8HP gear ratios + final drive
pub const GEAR_RATIOS: [f64; 9] = [0.0, 4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667];
pub const FINAL_DRIVE: f64 = 3.06;
pub const WHEEL_RADIUS_M: f64 = 0.32;
pub const FUEL_TANK_L: f64 = 55.0;

#[derive(Debug, Clone)]
pub struct EngineEcu {
    pub rpm: f64,
    pub throttle_position: f64,    // 0-1
    pub engine_temp: f64,          // °C
    pub oil_temp: f64,             // °C
    pub oil_pressure: f64,         // bar
    pub coolant_temp: f64,         // °C
    pub fuel_level: f64,           // 0-1
    pub fuel_consumption_lph: f64, // L/hour instantaneous
    pub boost_pressure: f64,       // bar (turbo)
    pub intake_air_temp: f64,      // °C
    pub output_torque: f64,        // Nm
    pub output_power_kw: f64,      // kW
    pub check_engine: bool,
    pub engine_on: bool,
    pub rev_limiter_active: bool,
    pub idle_rpm: f64,
    pub redline_rpm: f64,
    pub max_rpm: f64,
}

impl EngineEcu {
    pub fn new() -> Self {
        Self {
            rpm: 820.0,
            throttle_position: 0.0,
            engine_temp: 22.0,
            oil_temp: 22.0,
            oil_pressure: 4.5,
            coolant_temp: 22.0,
            fuel_level: 0.85,
            fuel_consumption_lph: 0.6,
            boost_pressure: 0.0,
            intake_air_temp: 25.0,
            output_torque: 0.0,
            output_power_kw: 0.0,
            check_engine: false,
            engine_on: true,
            rev_limiter_active: false,
            idle_rpm: 820.0,
            redline_rpm: 6800.0,
            max_rpm: 7200.0,
        }
    }

    /// Torque curve: 2.0L turbo, ~250 Nm peak at 2000-4500 RPM
    fn torque_at_rpm(&self, rpm: f64, throttle: f64) -> f64 {
        if rpm < 500.0 {
            return 0.0;
        }
        // Flat torque plateau between 2000-4500 RPM (turbocharged)
        let peak = 250.0;
        let t = if rpm < 1500.0 {
            rpm / 1500.0 * 0.7
        } else if rpm < 2000.0 {
            0.7 + (rpm - 1500.0) / 500.0 * 0.3
        } else if rpm < 4500.0 {
            1.0
        } else if rpm < 6800.0 {
            1.0 - (rpm - 4500.0) / 2300.0 * 0.6
        } else {
            0.0
        };
        peak * t * throttle
    }

    pub fn update(&mut self, throttle: f64, gear: u8, vehicle_speed_kmh: f64, dt: f64) {
        self.throttle_position = throttle.clamp(0.0, 1.0);
        self.rev_limiter_active = self.rpm >= self.redline_rpm;

        let effective_throttle = if self.rev_limiter_active {
            0.0
        } else {
            throttle
        };

        // Compute RPM from wheel speed × gear ratio
        let target_rpm = if gear > 0 && gear <= 8 {
            let wheel_rps = (vehicle_speed_kmh / 3.6) / WHEEL_RADIUS_M;
            (wheel_rps * 60.0 * GEAR_RATIOS[gear as usize] * FINAL_DRIVE).max(self.idle_rpm)
        } else {
            self.idle_rpm + effective_throttle * 1500.0
        };

        self.rpm += (target_rpm - self.rpm) * (dt / 0.08).min(1.0);
        self.rpm = self.rpm.clamp(self.idle_rpm * 0.9, self.max_rpm);

        self.output_torque = self.torque_at_rpm(self.rpm, effective_throttle);
        self.output_power_kw = self.output_torque * self.rpm * std::f64::consts::PI / 30.0 / 1000.0;

        // Turbo boost builds with RPM + throttle
        let boost_target =
            (effective_throttle * 1.6 * ((self.rpm - 1500.0) / 1500.0).clamp(0.0, 1.0)).max(0.0);
        self.boost_pressure += (boost_target - self.boost_pressure) * dt * 3.0;

        // Thermal model
        let temp_target = 92.0 + effective_throttle * 10.0;
        self.engine_temp += (temp_target - self.engine_temp) * dt * 0.015;
        self.coolant_temp = self.engine_temp;
        self.oil_temp = self.engine_temp + 8.0;

        // Oil pressure drops slightly at idle, rises with RPM
        self.oil_pressure = 1.2 + (self.rpm / self.max_rpm) * 4.0;

        // Fuel consumption: idle ~0.6 L/h, WOT at high RPM ~18 L/h
        self.fuel_consumption_lph = 0.6 + effective_throttle * 17.0 * (self.rpm / 4000.0).min(1.0);
        let consumed = self.fuel_consumption_lph * dt / 3600.0;
        self.fuel_level = (self.fuel_level - consumed / FUEL_TANK_L).max(0.0);
    }

    /// Torque available at wheel (through drivetrain, at current RPM)
    pub fn wheel_drive_force_n(&self, gear: u8) -> f64 {
        if gear == 0 || gear > 8 {
            return 0.0;
        }
        let wheel_torque = self.output_torque * GEAR_RATIOS[gear as usize] * FINAL_DRIVE;
        wheel_torque / WHEEL_RADIUS_M
    }
}
