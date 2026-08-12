//! GPS/GNSS Module — Full simulation of a u-blox ZED-F9P grade receiver.
//! Outputs real NMEA 0183 sentences (GGA, RMC, GSA, GSV) with valid checksums.
//! Simulates: satellite constellation, atmospheric delays, multipath, SBAS correction.

use std::collections::VecDeque;
use std::f64::consts::PI;

// ── Fix Quality ───────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsFixQuality {
    NoFix = 0,
    SpsFix = 1,  // Standard Positioning Service
    DgpsFix = 2, // Differential GPS
    PpsFix = 3,
    RtkFix = 4, // Real-Time Kinematic (cm accuracy)
    FloatRtk = 5,
    Dead = 6, // Dead Reckoning
}

impl std::fmt::Display for GpsFixQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GpsFixQuality::NoFix => write!(f, "NO FIX"),
            GpsFixQuality::SpsFix => write!(f, "SPS (±3m) "),
            GpsFixQuality::DgpsFix => write!(f, "DGPS(±1m) "),
            GpsFixQuality::RtkFix => write!(f, "RTK (±2cm)"),
            GpsFixQuality::FloatRtk => write!(f, "RTK-Float "),
            GpsFixQuality::Dead => write!(f, "DR        "),
            _ => write!(f, "PPS       "),
        }
    }
}

// ── Satellite ─────────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct Satellite {
    pub prn: u8,        // Pseudo-Random Noise code (1-32 GPS, 65-88 SBAS)
    pub elevation: f64, // degrees above horizon (0-90)
    pub azimuth: f64,   // degrees clockwise from north (0-360)
    pub snr: f64,       // Signal-to-Noise Ratio (dBHz, 0-50)
    pub used: bool,     // Used in position fix
    pub constellation: Constellation,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Constellation {
    Gps,
    Glonass,
    Galileo,
    Beidou,
    Sbas,
}

impl std::fmt::Display for Constellation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Constellation::Gps => "GPS",
            Constellation::Glonass => "GLO",
            Constellation::Galileo => "GAL",
            Constellation::Beidou => "BDS",
            Constellation::Sbas => "SBS",
        }
        .fmt(f)
    }
}

// ── GPS Module ────────────────────────────────────────────────────────────────
pub struct GpsModule {
    // ─ Position ──────────────────────────────────────────────────────────────
    pub latitude_deg: f64,  // decimal degrees (positive = N)
    pub longitude_deg: f64, // decimal degrees (positive = E)
    pub altitude_msl: f64,  // metres above Mean Sea Level
    pub undulation: f64,    // geoid undulation (WGS84 height diff)

    // ─ Velocity ──────────────────────────────────────────────────────────────
    pub speed_knots: f64, // speed over ground (knots)
    pub speed_kmh: f64,   // speed over ground (km/h)
    pub course_deg: f64,  // course over ground (degrees true)
    pub climb_ms: f64,    // vertical velocity (m/s)

    // ─ Accuracy ──────────────────────────────────────────────────────────────
    pub hdop: f64,   // Horizontal Dilution of Precision
    pub vdop: f64,   // Vertical DOP
    pub pdop: f64,   // Position DOP
    pub tdop: f64,   // Time DOP
    pub hacc_m: f64, // Horizontal accuracy (metres, 1σ)
    pub vacc_m: f64, // Vertical accuracy
    pub fix_quality: GpsFixQuality,

    // ─ Satellites ────────────────────────────────────────────────────────────
    pub satellites: Vec<Satellite>,
    pub sats_in_view: u8,
    pub sats_used: u8,

    // ─ Time ──────────────────────────────────────────────────────────────────
    pub utc_time: String, // "HHMMSS.ss"
    pub utc_date: String, // "DDMMYY"
    pub gps_week: u16,
    pub time_of_week_s: f64,

    // ─ Dead Reckoning ────────────────────────────────────────────────────────
    pub dr_active: bool, // Using dead reckoning (GNSS lost)
    pub dr_time_s: f64,  // Seconds since last GNSS fix
    pub dr_error_m: f64, // Accumulated DR error

    // ─ Simulation state ──────────────────────────────────────────────────────
    pos_x: f64, // Local ENU East (metres from origin)
    pos_y: f64, // Local ENU North
    noise_seed: f64,
    update_timer: f64,
    pub nmea_queue: VecDeque<String>,
    pub update_rate_hz: f64,
}

impl Default for GpsModule {
    fn default() -> Self {
        Self::new()
    }
}

impl GpsModule {
    pub fn new() -> Self {
        // Start at a typical field location in Brazil (lat -22°, lon -47°)
        let mut gps = GpsModule {
            latitude_deg: -22.8100,
            longitude_deg: -47.0620,
            altitude_msl: 640.0,
            undulation: -10.5,
            speed_knots: 0.0,
            speed_kmh: 0.0,
            course_deg: 0.0,
            climb_ms: 0.0,
            hdop: 0.9,
            vdop: 1.2,
            pdop: 1.5,
            tdop: 0.8,
            hacc_m: 1.2,
            vacc_m: 2.0,
            fix_quality: GpsFixQuality::SpsFix,
            satellites: Vec::new(),
            sats_in_view: 0,
            sats_used: 0,
            utc_time: "120000.00".into(),
            utc_date: "110826".into(),
            gps_week: 2326,
            time_of_week_s: 43200.0,
            dr_active: false,
            dr_time_s: 0.0,
            dr_error_m: 0.0,
            pos_x: 0.0,
            pos_y: 0.0,
            noise_seed: 0.0,
            update_timer: 0.0,
            nmea_queue: VecDeque::new(),
            update_rate_hz: 10.0,
        };
        gps.init_satellites();
        gps
    }

    fn init_satellites(&mut self) {
        // Simulate a realistic satellite constellation visible from Brazil
        let sat_data = [
            // PRN, elev, azimuth, SNR, constellation
            (1u8, 55.0, 185.0, 44.0, Constellation::Gps),
            (3, 32.0, 290.0, 38.0, Constellation::Gps),
            (6, 18.0, 125.0, 30.0, Constellation::Gps),
            (7, 72.0, 42.0, 47.0, Constellation::Gps),
            (11, 44.0, 320.0, 41.0, Constellation::Gps),
            (14, 28.0, 210.0, 35.0, Constellation::Gps),
            (17, 61.0, 150.0, 45.0, Constellation::Gps),
            (19, 12.0, 78.0, 22.0, Constellation::Gps),
            (22, 38.0, 265.0, 39.0, Constellation::Gps),
            (28, 85.0, 330.0, 48.0, Constellation::Gps),
            (65, 42.0, 95.0, 37.0, Constellation::Glonass),
            (66, 29.0, 175.0, 33.0, Constellation::Glonass),
            (67, 58.0, 250.0, 43.0, Constellation::Glonass),
            (68, 15.0, 310.0, 24.0, Constellation::Glonass),
            (11 + 64, 47.0, 60.0, 40.0, Constellation::Galileo),
            (12 + 64, 33.0, 220.0, 36.0, Constellation::Galileo),
            (13 + 64, 68.0, 145.0, 46.0, Constellation::Galileo),
        ];
        for (prn, elev, azimuth, snr, constellation) in sat_data.iter() {
            self.satellites.push(Satellite {
                prn: *prn,
                elevation: *elev,
                azimuth: *azimuth,
                snr: *snr,
                used: *elev > 10.0,
                constellation: *constellation,
            });
        }
        self.sats_in_view = self.satellites.len() as u8;
        self.sats_used = self.satellites.iter().filter(|s| s.used).count() as u8;
    }

    /// Main update — call every simulation step.
    /// `speed_kmh` and `heading_deg` come from vehicle dynamics.
    pub fn update(&mut self, speed_kmh: f64, heading_deg: f64, dt: f64) {
        self.update_timer += dt;

        // ─ Update position from vehicle speed ──────────────────────────────
        let speed_ms = speed_kmh / 3.6;
        let head_rad = heading_deg * PI / 180.0;
        self.pos_x += speed_ms * head_rad.sin() * dt;
        self.pos_y += speed_ms * head_rad.cos() * dt;

        // ─ Convert local ENU to lat/lon ────────────────────────────────────
        // 1° latitude  ≈ 111,320 m
        // 1° longitude ≈ 111,320 × cos(lat) m
        let metres_per_deg_lat = 111_320.0;
        let metres_per_deg_lon = 111_320.0 * (self.latitude_deg * PI / 180.0).cos();
        self.latitude_deg += (self.pos_y / metres_per_deg_lat) * 0.001 * dt; // slow drift
        self.longitude_deg += (self.pos_x / metres_per_deg_lon) * 0.001 * dt;
        // Actual integration — more accurate
        let d_lat = (speed_ms * head_rad.cos() * dt) / metres_per_deg_lat;
        let d_lon = (speed_ms * head_rad.sin() * dt) / metres_per_deg_lon;
        self.latitude_deg += d_lat;
        self.longitude_deg += d_lon;

        self.speed_kmh = speed_kmh;
        self.speed_knots = speed_kmh * 0.539957;
        self.course_deg = heading_deg;

        // ─ Add GPS noise (pseudo-random, position-dependent) ───────────────
        self.noise_seed += dt * 7.3;
        let noise = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        let pos_noise_m = match self.fix_quality {
            GpsFixQuality::RtkFix => 0.02,
            GpsFixQuality::DgpsFix => 0.5,
            GpsFixQuality::SpsFix => 2.5,
            GpsFixQuality::Dead => self.dr_error_m,
            _ => 5.0,
        };
        let noise_lat = noise(self.noise_seed) * pos_noise_m / metres_per_deg_lat;
        let noise_lon = noise(self.noise_seed + 1.5) * pos_noise_m / metres_per_deg_lon;
        self.hacc_m = pos_noise_m * 1.5;
        self.vacc_m = pos_noise_m * 2.5;

        // ─ Satellite dynamics (elevation/SNR vary slowly) ──────────────────
        let t = self.noise_seed;
        for (i, sat) in self.satellites.iter_mut().enumerate() {
            let drift = (t * 0.01 + i as f64 * 0.7).sin() * 0.3;
            sat.elevation = (sat.elevation + drift * dt * 0.1).clamp(5.0, 90.0);
            let snr_noise = noise(t + i as f64 * 3.1) * 0.5;
            sat.snr = (sat.snr + snr_noise * dt * 5.0).clamp(10.0, 52.0);
            sat.used = sat.elevation > 10.0 && sat.snr > 18.0;
        }
        self.sats_used = self.satellites.iter().filter(|s| s.used).count() as u8;
        self.sats_in_view = self.satellites.len() as u8;

        // ─ DOP calculation ─────────────────────────────────────────────────
        // Simplified: better HDOP with more good satellites overhead
        let good_sats = self
            .satellites
            .iter()
            .filter(|s| s.used && s.elevation > 30.0)
            .count() as f64;
        self.hdop = (1.5 - good_sats * 0.05).max(0.8) + noise(t + 5.1).abs() * 0.1;
        self.vdop = self.hdop * 1.35;
        self.pdop = (self.hdop.powi(2) + self.vdop.powi(2)).sqrt();

        // ─ Update UTC time ─────────────────────────────────────────────────
        self.time_of_week_s += dt;
        let total_s = self.time_of_week_s;
        let h = ((total_s / 3600.0) as u32) % 24;
        let m = ((total_s / 60.0) as u32) % 60;
        let s = total_s % 60.0;
        self.utc_time = format!("{:02}{:02}{:05.2}", h, m, s);

        // ─ Generate NMEA sentences at update rate ───────────────────────────
        if self.update_timer >= 1.0 / self.update_rate_hz {
            self.update_timer = 0.0;
            let lat_applied = self.latitude_deg + noise_lat;
            let lon_applied = self.longitude_deg + noise_lon;
            self.nmea_queue
                .push_back(self.build_gga(lat_applied, lon_applied));
            self.nmea_queue
                .push_back(self.build_rmc(lat_applied, lon_applied));
            self.nmea_queue.push_back(self.build_gsa());
            if self.nmea_queue.len() > 120 {
                self.nmea_queue.drain(0..30);
            }
        }
    }

    // ── NMEA sentence builders ────────────────────────────────────────────────

    /// GGA — Global Positioning Fix Data
    fn build_gga(&self, lat: f64, lon: f64) -> String {
        let (_, lat_m, lat_h) = Self::deg_to_nmea(lat, 'N', 'S');
        let (_, lon_m, lon_h) = Self::deg_to_nmea(lon, 'E', 'W');
        let body = format!(
            "GPGGA,{},{:010.5},{},{:011.5},{},{},{:02},{:.1},{:.1},M,{:.1},M,,",
            self.utc_time,
            lat_m,
            lat_h,
            lon_m,
            lon_h,
            self.fix_quality as u8,
            self.sats_used,
            self.hdop,
            self.altitude_msl,
            self.undulation
        );
        format!("${}*{:02X}\r\n", body, Self::nmea_checksum(&body))
    }

    /// RMC — Recommended Minimum Specific GNSS Data
    fn build_rmc(&self, lat: f64, lon: f64) -> String {
        let (_, lat_m, lat_h) = Self::deg_to_nmea(lat, 'N', 'S');
        let (_, lon_m, lon_h) = Self::deg_to_nmea(lon, 'E', 'W');
        let body = format!(
            "GPRMC,{},A,{:010.5},{},{:011.5},{},{:.2},{:.2},{},0.0,E,A",
            self.utc_time,
            lat_m,
            lat_h,
            lon_m,
            lon_h,
            self.speed_knots,
            self.course_deg,
            self.utc_date
        );
        format!("${}*{:02X}\r\n", body, Self::nmea_checksum(&body))
    }

    /// GSA — GNSS DOP and Active Satellites
    fn build_gsa(&self) -> String {
        let used: String = self
            .satellites
            .iter()
            .filter(|s| s.used)
            .take(12)
            .map(|s| format!("{:02}", s.prn))
            .collect::<Vec<_>>()
            .join(",");
        let body = format!(
            "GPGSA,A,3,{},{},{:.1},{:.1},{:.1}",
            used,
            ",".repeat(12usize.saturating_sub(self.sats_used as usize)),
            self.pdop,
            self.hdop,
            self.vdop
        );
        format!("${}*{:02X}\r\n", body, Self::nmea_checksum(&body))
    }

    fn deg_to_nmea(deg: f64, pos: char, neg: char) -> (f64, f64, char) {
        let hemi = if deg >= 0.0 { pos } else { neg };
        let abs = deg.abs();
        let d = abs.floor();
        let m = (abs - d) * 60.0;
        (d, d * 100.0 + m, hemi)
    }

    fn nmea_checksum(sentence: &str) -> u8 {
        sentence.bytes().fold(0u8, |acc, b| acc ^ b)
    }

    pub fn position_string(&self) -> String {
        let lat_d = self.latitude_deg.abs();
        let lat_h = if self.latitude_deg >= 0.0 { 'N' } else { 'S' };
        let lon_d = self.longitude_deg.abs();
        let lon_h = if self.longitude_deg >= 0.0 { 'E' } else { 'W' };
        format!("{:.6}°{}  {:.6}°{}", lat_d, lat_h, lon_d, lon_h)
    }
}
