//! Body Control Module (BCM) — manages all electrical body functions.
//! SA: 0x27 (Cab Controller).
//! Models: lighting, horn, wipers, battery/charging, fuses, switches.
//! J1939: Transmits heartbeat via Prop-B, monitors battery (PGN 65272).

use crate::j1939::{self, addr, J1939Frame};

pub const BCM_SA: u8 = addr::CAB; // 0x27

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WiperSpeed {
    Off,
    Low,
    High,
    Intermittent,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChargingState {
    Off,
    Charging,
    Fault,
}

#[derive(Debug, Clone)]
pub struct Fuse {
    pub id: &'static str,
    pub rating_a: f64,
    pub blown: bool,
    pub load_a: f64,
}

impl Fuse {
    pub fn new(id: &'static str, rating: f64) -> Self {
        Fuse {
            id,
            rating_a: rating,
            blown: false,
            load_a: 0.0,
        }
    }
    pub fn check(&mut self) {
        if self.load_a > self.rating_a * 1.15 {
            self.blown = true;
        }
    }
}

// ── BCM ──────────────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuBcm {
    pub sa: u8,

    // ─ Lighting ─────────────────────────────────────────────────────────────
    pub road_lights: bool,       // low-beam road driving lights
    pub work_lights_front: bool, // high-intensity work lights (front boom)
    pub work_lights_rear: bool,
    pub beacon_light: bool, // rotating amber beacon (road transport)
    pub hazard_lights: bool,
    pub cab_interior_light: bool,

    // ─ Horn ──────────────────────────────────────────────────────────────────
    pub horn_active: bool,
    horn_timer: f64,

    // ─ Wipers ────────────────────────────────────────────────────────────────
    pub wiper_speed: WiperSpeed,
    pub wiper_position: f64, // 0=parked, 1=fully swept
    wiper_sweep_dir: f64,    // +1 or -1

    // ─ Battery / Charging ────────────────────────────────────────────────────
    pub battery_voltage: f64,
    pub battery_current_a: f64, // + = charging, - = discharging
    pub battery_soc_pct: f64,
    pub charging_state: ChargingState,
    pub alternator_amps: f64,

    // ─ Total electrical load ─────────────────────────────────────────────────
    pub total_load_amps: f64,

    // ─ Fuses ─────────────────────────────────────────────────────────────────
    pub fuses: Vec<Fuse>,

    // ─ Cab climate (simplified) ──────────────────────────────────────────────
    pub hvac_on: bool,
    pub hvac_temp_set_c: f64,
    pub cab_temp_c: f64,

    // ─ J1939 TX timer ─────────────────────────────────────────────────────────
    t_heartbeat: f64,
}

impl Default for EcuBcm {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuBcm {
    pub fn new() -> Self {
        let fuses = vec![
            Fuse::new("WORK_LT_F", 25.0),
            Fuse::new("WORK_LT_R", 25.0),
            Fuse::new("ROAD_LT", 15.0),
            Fuse::new("WIPER", 10.0),
            Fuse::new("HVAC", 20.0),
            Fuse::new("HORN", 5.0),
            Fuse::new("CAB_INT", 10.0),
            Fuse::new("BEACON", 5.0),
        ];
        EcuBcm {
            sa: BCM_SA,
            road_lights: false,
            work_lights_front: false,
            work_lights_rear: false,
            beacon_light: false,
            hazard_lights: false,
            cab_interior_light: true,
            horn_active: false,
            horn_timer: 0.0,
            wiper_speed: WiperSpeed::Off,
            wiper_position: 0.0,
            wiper_sweep_dir: 1.0,
            battery_voltage: 12.8,
            battery_current_a: -2.0,
            battery_soc_pct: 80.0,
            charging_state: ChargingState::Off,
            alternator_amps: 0.0,
            total_load_amps: 0.0,
            fuses,
            hvac_on: false,
            hvac_temp_set_c: 22.0,
            cab_temp_c: 25.0,
            t_heartbeat: 0.0,
        }
    }

    /// Main BCM tick — returns J1939 frames.
    pub fn tick(&mut self, alternator_v: f64, dt: f64) -> Vec<J1939Frame> {
        // ─ Charging model ────────────────────────────────────────────────────
        self.alternator_amps = if alternator_v > 13.5 {
            self.charging_state = ChargingState::Charging;
            80.0 // 80A alternator
        } else {
            self.charging_state = ChargingState::Off;
            0.0
        };

        // ─ Calculate total electrical load ───────────────────────────────────
        self.total_load_amps = 0.0;
        if self.work_lights_front {
            self.total_load_amps += 18.0;
        }
        if self.work_lights_rear {
            self.total_load_amps += 14.0;
        }
        if self.road_lights {
            self.total_load_amps += 8.0;
        }
        if self.beacon_light {
            self.total_load_amps += 4.0;
        }
        if self.hazard_lights {
            self.total_load_amps += 6.0;
        }
        if self.cab_interior_light {
            self.total_load_amps += 2.0;
        }
        if self.hvac_on {
            self.total_load_amps += 15.0;
        }
        if self.horn_active {
            self.total_load_amps += 8.0;
        }
        // Base ECU load
        self.total_load_amps += 5.0;

        // ─ Battery current and voltage ────────────────────────────────────────
        self.battery_current_a = self.alternator_amps - self.total_load_amps;
        // Simple SoC model
        self.battery_soc_pct = (self.battery_soc_pct
            + self.battery_current_a * dt / 3600.0 * 100.0 / 88.0)
            .clamp(0.0, 100.0); // 88 Ah battery
        self.battery_voltage =
            11.5 + self.battery_soc_pct / 100.0 * 1.8 + if alternator_v > 13.0 { 0.5 } else { 0.0 };

        // ─ Fuse monitoring ────────────────────────────────────────────────────
        for fuse in &mut self.fuses {
            fuse.load_a = match fuse.id {
                "WORK_LT_F" => {
                    if self.work_lights_front {
                        18.0
                    } else {
                        0.0
                    }
                }
                "WORK_LT_R" => {
                    if self.work_lights_rear {
                        14.0
                    } else {
                        0.0
                    }
                }
                "ROAD_LT" => {
                    if self.road_lights {
                        8.0
                    } else {
                        0.0
                    }
                }
                "WIPER" => {
                    if self.wiper_speed != WiperSpeed::Off {
                        4.0
                    } else {
                        0.0
                    }
                }
                "HVAC" => {
                    if self.hvac_on {
                        15.0
                    } else {
                        0.0
                    }
                }
                "HORN"
                    if self.horn_active => {
                        8.0
                    }
                _ => 0.0,
            };
            fuse.check();
        }

        // ─ Wiper simulation ──────────────────────────────────────────────────
        let wiper_ok = !self.fuses.iter().any(|f| f.id == "WIPER" && f.blown);
        if wiper_ok {
            let sweep_speed = match self.wiper_speed {
                WiperSpeed::Off => 0.0,
                WiperSpeed::Low => 0.6,
                WiperSpeed::High => 1.4,
                WiperSpeed::Intermittent => {
                    if (self.t_heartbeat % 3.0) < 0.5 {
                        1.0
                    } else {
                        0.0
                    }
                }
            };
            self.wiper_position += self.wiper_sweep_dir * sweep_speed * dt;
            if self.wiper_position >= 1.0 {
                self.wiper_sweep_dir = -1.0;
            }
            if self.wiper_position <= 0.0 {
                self.wiper_sweep_dir = 1.0;
            }
            self.wiper_position = self.wiper_position.clamp(0.0, 1.0);
        }

        // ─ Horn (auto-off after 2 s) ─────────────────────────────────────────
        if self.horn_active {
            self.horn_timer += dt;
            if self.horn_timer > 2.0 {
                self.horn_active = false;
                self.horn_timer = 0.0;
            }
        }

        // ─ HVAC simple model ─────────────────────────────────────────────────
        if self.hvac_on {
            let cool = if self.cab_temp_c > self.hvac_temp_set_c {
                -8.0
            } else {
                3.0
            };
            self.cab_temp_c += cool * dt * 0.05;
        }

        // ─ J1939 heartbeat (100 ms) ──────────────────────────────────────────
        self.t_heartbeat += dt;
        let mut frames: Vec<J1939Frame> = Vec::new();
        if self.t_heartbeat >= 0.100 {
            self.t_heartbeat = 0.0;
            // Proprietary B: BCM status (PGN 65280)
            let mut data = [0u8; 8];
            data[0] = (if self.road_lights { 0x01 } else { 0 })
                | (if self.work_lights_front { 0x02 } else { 0 })
                | (if self.work_lights_rear { 0x04 } else { 0 })
                | (if self.beacon_light { 0x08 } else { 0 })
                | (if self.hazard_lights { 0x10 } else { 0 });
            data[1] = (self.battery_voltage * 20.0) as u8; // 0.05V/bit
            data[2] = self.battery_soc_pct as u8;
            data[3] = if self.charging_state == ChargingState::Charging {
                0x01
            } else {
                0x00
            };
            let raw_id = J1939Frame::build_id(7, j1939::pgn::PROP_A, self.sa, 0xFF);
            frames.push(J1939Frame::from_raw(self.t_heartbeat, raw_id, &data));
        }
        frames
    }

    // ─ Convenience toggles ───────────────────────────────────────────────────
    pub fn toggle_work_lights(&mut self) {
        self.work_lights_front = !self.work_lights_front;
        self.work_lights_rear = !self.work_lights_rear;
    }
    pub fn toggle_road_lights(&mut self) {
        self.road_lights = !self.road_lights;
    }
    pub fn toggle_beacon(&mut self) {
        self.beacon_light = !self.beacon_light;
    }
    pub fn honk(&mut self) {
        self.horn_active = true;
        self.horn_timer = 0.0;
    }
    pub fn cycle_wiper(&mut self) {
        self.wiper_speed = match self.wiper_speed {
            WiperSpeed::Off => WiperSpeed::Intermittent,
            WiperSpeed::Intermittent => WiperSpeed::Low,
            WiperSpeed::Low => WiperSpeed::High,
            WiperSpeed::High => WiperSpeed::Off,
        };
    }
    pub fn toggle_hvac(&mut self) {
        self.hvac_on = !self.hvac_on;
    }

    pub fn reset_all_fuses(&mut self) {
        for fuse in &mut self.fuses {
            fuse.blown = false;
        }
    }
}
