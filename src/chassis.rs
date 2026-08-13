const WHEELBASE: f64 = 2.7; // m

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WheelState {
    Rolling,
    Braking,   // normal brake pressure applied, no lockup
    Skidding,
    AbsActive,
    TcsLimited,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EpsMode {
    Normal,
    Sport,
    Comfort,
}

/// Chassis domain controller: ABS + ESP + TCS + EPS
#[derive(Debug, Clone)]
pub struct ChassisControl {
    // ABS
    pub abs_active: bool,
    pub abs_cycles: u32,
    abs_cycle_phases: [f64; 4],

    // ESP — Electronic Stability Program
    pub esp_active: bool,
    pub oversteer: bool,
    pub understeer: bool,
    pub esp_brake_correction: [f64; 4], // individual wheel brake delta

    // TCS — Traction Control
    pub tcs_active: bool,
    pub tcs_throttle_cut: f64, // 0-1 fraction cut

    // EPS — Electronic Power Steering
    pub eps_assist_torque: f64, // Nm
    pub eps_mode: EpsMode,

    // Per-wheel state
    pub wheel_states: [WheelState; 4],
    pub brake_pressures: [f64; 4],
    pub wheel_velocities: [f64; 4],
}

impl ChassisControl {
    pub fn new() -> Self {
        Self {
            abs_active: false,
            abs_cycles: 0,
            abs_cycle_phases: [0.0; 4],
            esp_active: false,
            oversteer: false,
            understeer: false,
            esp_brake_correction: [0.0; 4],
            tcs_active: false,
            tcs_throttle_cut: 0.0,
            eps_assist_torque: 0.0,
            eps_mode: EpsMode::Normal,
            wheel_states: [WheelState::Rolling; 4],
            brake_pressures: [0.0; 4],
            wheel_velocities: [0.0; 4],
        }
    }

    /// Returns effective throttle after TCS cut
    pub fn update(
        &mut self,
        vehicle_speed: f64,
        steering_angle: f64,
        yaw_rate: f64,
        user_brake: f64,
        user_throttle: f64,
        dt: f64,
    ) -> (f64, f64) {
        // --- ABS ----------------------------------------------------------
        let mut effective_brake = user_brake;
        self.abs_active = false;

        for i in 0..4 {
            // Wheel dynamics: brake slows wheel faster than vehicle
            let decel = self.brake_pressures[i] * 12.0;
            self.wheel_velocities[i] = (self.wheel_velocities[i] - decel * dt).max(0.0);
            // Follow vehicle speed when not braking
            self.wheel_velocities[i] +=
                ((vehicle_speed - self.wheel_velocities[i]) * 0.3 * dt).max(0.0);

            let slip = vehicle_speed - self.wheel_velocities[i];
            if slip > 5.0 && user_brake > 0.3 && vehicle_speed > 5.0 {
                self.wheel_states[i] = WheelState::Skidding;
                self.abs_active = true;
                self.abs_cycles += 1;
            } else if user_brake > 0.05 {
                self.wheel_states[i] = WheelState::Braking; // normal braking, not ABS
            } else {
                self.wheel_states[i] = WheelState::Rolling;
                self.wheel_velocities[i] = vehicle_speed; // sync
            }
        }

        // ABS modulates brake pressure at 8 Hz
        if self.abs_active {
            for i in 0..4 {
                self.abs_cycle_phases[i] += dt * 8.0;
                if self.abs_cycle_phases[i] > 1.0 {
                    self.abs_cycle_phases[i] -= 1.0;
                }
                let modulated = if self.abs_cycle_phases[i] < 0.45 {
                    user_brake * 0.28
                } else {
                    user_brake * 0.92
                };
                self.brake_pressures[i] = modulated;
            }
            effective_brake = self.brake_pressures.iter().sum::<f64>() / 4.0;
        } else {
            for p in &mut self.brake_pressures {
                *p = user_brake;
            }
        }

        // --- TCS ----------------------------------------------------------
        let mut effective_throttle = user_throttle;
        self.tcs_active = false;
        self.tcs_throttle_cut = 0.0;

        if user_throttle > 0.2 && vehicle_speed < 60.0 {
            // Detect wheel spin (driven wheels RL/RR faster than vehicle)
            let driven_avg = (self.wheel_velocities[2] + self.wheel_velocities[3]) / 2.0;
            if driven_avg > vehicle_speed + 6.0 {
                self.tcs_active = true;
                self.tcs_throttle_cut = ((driven_avg - vehicle_speed - 6.0) / 10.0).min(0.8);
                effective_throttle *= 1.0 - self.tcs_throttle_cut;
                // Mark driven wheels
                self.wheel_states[2] = WheelState::TcsLimited;
                self.wheel_states[3] = WheelState::TcsLimited;
            }
        }

        // --- ESP ----------------------------------------------------------
        self.esp_active = false;
        self.esp_brake_correction = [0.0; 4];

        if vehicle_speed > 20.0 {
            // Theoretical yaw rate for neutral steer
            let yaw_neutral = (vehicle_speed / 3.6) * (steering_angle.to_radians()) / WHEELBASE;
            let yaw_error = yaw_rate.to_radians() - yaw_neutral;

            if yaw_error.abs() > 0.08 {
                self.esp_active = true;
                if yaw_error > 0.0 {
                    // Oversteer: apply outer front wheel brake
                    self.oversteer = true;
                    self.understeer = false;
                    self.esp_brake_correction[1] = (yaw_error * 3.0).min(0.6); // FR
                } else {
                    // Understeer: apply inner rear wheel brake
                    self.understeer = true;
                    self.oversteer = false;
                    self.esp_brake_correction[2] = ((-yaw_error) * 2.0).min(0.5);
                    // RL
                }
            } else {
                self.oversteer = false;
                self.understeer = false;
            }
        }

        // --- EPS ----------------------------------------------------------
        // Assist drops at high speed (less assist needed)
        let speed_factor = (1.0 - vehicle_speed / 200.0).clamp(0.2, 1.0);
        self.eps_assist_torque = match self.eps_mode {
            EpsMode::Comfort => steering_angle.abs() * 0.12 * speed_factor,
            EpsMode::Normal => steering_angle.abs() * 0.08 * speed_factor,
            EpsMode::Sport => steering_angle.abs() * 0.04 * speed_factor,
        };

        (effective_throttle, effective_brake)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
