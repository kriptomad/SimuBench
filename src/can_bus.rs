use std::collections::VecDeque;

const BUS_LOAD_ALPHA: f64 = 0.05;

#[derive(Debug, Clone)]
pub struct CanFrame {
    pub id: u32,
    pub name: &'static str,
    pub value: String,
    pub timestamp: f64,
}

/// Simulated 500 kbps CAN bus (HS-CAN)
pub struct CanBus {
    pub frames: VecDeque<CanFrame>,
    pub total_frames: u64,
    pub bus_load_pct: f64,
    pub frames_per_second: u32,
    frame_counter: u32,
    frame_timer: f64,
}

impl CanBus {
    pub fn new() -> Self {
        Self {
            frames: VecDeque::new(),
            total_frames: 0,
            bus_load_pct: 0.0,
            frames_per_second: 0,
            frame_counter: 0,
            frame_timer: 0.0,
        }
    }

    pub fn publish(&mut self, id: u32, name: &'static str, value: String, ts: f64) {
        self.frames.push_front(CanFrame {
            id,
            name,
            value,
            timestamp: ts,
        });
        if self.frames.len() > 12 {
            self.frames.pop_back();
        }
        self.total_frames += 1;
        self.frame_counter += 1;
    }

    pub fn tick(&mut self, dt: f64) {
        self.frame_timer += dt;
        if self.frame_timer >= 1.0 {
            self.frames_per_second = self.frame_counter;
            // 500kbps, each frame ~128 bits → ~3900 frames/s max
            let load = self.frame_counter as f64 / 3900.0 * 100.0;
            self.bus_load_pct += (load.min(100.0) - self.bus_load_pct) * BUS_LOAD_ALPHA;
            self.frame_counter = 0;
            self.frame_timer = 0.0;
        }
    }

    /// Publish all periodic ECU messages
    #[allow(clippy::too_many_arguments)]
    pub fn broadcast_all(
        &mut self,
        speed: f64,
        rpm: f64,
        gear: &str,
        engine_temp: f64,
        throttle: f64,
        brake: f64,
        abs_active: bool,
        esp_active: bool,
        tcs_active: bool,
        acc_enabled: bool,
        acc_speed: f64,
        aeb_ttc: f64,
        lka_active: bool,
        fuel_level: f64,
        boost: f64,
        torque: f64,
        oil_pressure: f64,
        elapsed: f64,
    ) {
        self.publish(0x100, "VEH_SPEED", format!("{:06.2} km/h", speed), elapsed);
        self.publish(0x101, "ENGINE_RPM", format!("{:5.0} rpm", rpm), elapsed);
        self.publish(0x102, "GEAR_POS", format!("{:<5}", gear), elapsed);
        self.publish(
            0x110,
            "ENG_TEMP",
            format!("{:5.1} °C", engine_temp),
            elapsed,
        );
        self.publish(
            0x111,
            "THROTTLE",
            format!("{:5.1} %", throttle * 100.0),
            elapsed,
        );
        self.publish(
            0x112,
            "BRAKE_REQ",
            format!("{:5.1} %", brake * 100.0),
            elapsed,
        );
        self.publish(0x113, "TORQUE", format!("{:6.1} Nm", torque), elapsed);
        self.publish(0x114, "BOOST", format!("{:4.2} bar", boost), elapsed);
        self.publish(
            0x115,
            "OIL_PRESS",
            format!("{:4.2} bar", oil_pressure),
            elapsed,
        );
        self.publish(
            0x120,
            "FUEL_LEVEL",
            format!("{:5.1} %", fuel_level * 100.0),
            elapsed,
        );
        self.publish(
            0x200,
            "ABS_STATUS",
            if abs_active {
                "ACTIVE".into()
            } else {
                "IDLE".into()
            },
            elapsed,
        );
        self.publish(
            0x201,
            "ESP_STATUS",
            if esp_active {
                "ACTIVE".into()
            } else {
                "IDLE".into()
            },
            elapsed,
        );
        self.publish(
            0x202,
            "TCS_STATUS",
            if tcs_active {
                "ACTIVE".into()
            } else {
                "IDLE".into()
            },
            elapsed,
        );
        self.publish(
            0x300,
            "ACC_STATUS",
            if acc_enabled {
                format!("{:3.0}km/h", acc_speed)
            } else {
                "OFF  ".into()
            },
            elapsed,
        );
        self.publish(
            0x301,
            "AEB_TTC",
            if aeb_ttc.is_infinite() {
                " ∞ s".into()
            } else {
                format!("{:4.1} s", aeb_ttc)
            },
            elapsed,
        );
        self.publish(
            0x302,
            "LKA_STATUS",
            if lka_active {
                "ACTIVE".into()
            } else {
                "IDLE".into()
            },
            elapsed,
        );
    }
}
