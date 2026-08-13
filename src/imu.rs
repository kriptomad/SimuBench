//! IMU — 9-DOF Inertial Measurement Unit (Accelerometer + Gyroscope + Magnetometer)
//! Implements the Madgwick AHRS filter for attitude estimation.
//! Models: noise, bias drift, temperature coefficient, vibration.

use std::f64::consts::PI;

// ── IMU configuration ─────────────────────────────────────────────────────────
const ACCEL_NOISE: f64 = 0.003; // m/s² (rms noise density × √BW)
const GYRO_NOISE: f64 = 0.0001; // rad/s
const MAG_NOISE: f64 = 0.5; // µT
#[allow(dead_code)]
const ACCEL_BIAS_STD: f64 = 0.01;
#[allow(dead_code)]
const GYRO_BIAS_STD: f64 = 0.001;
const GYRO_DRIFT: f64 = 5e-7; // rad/s per second (in-run bias stability)
const BETA: f64 = 0.1; // Madgwick filter gain (0.01=slow, 0.5=fast)

// ── Quaternion ────────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn identity() -> Self {
        Quaternion {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn normalise(&mut self) {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n > 1e-10 {
            self.w /= n;
            self.x /= n;
            self.y /= n;
            self.z /= n;
        }
    }

    /// Convert to Euler angles (roll, pitch, yaw) in degrees
    pub fn to_euler_deg(&self) -> (f64, f64, f64) {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let sin_p = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
        let pitch = sin_p.asin();
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        (roll * 180.0 / PI, pitch * 180.0 / PI, yaw * 180.0 / PI)
    }
}

// ── IMU ──────────────────────────────────────────────────────────────────────
pub struct Imu {
    // ─ Raw sensor outputs (body frame) ─────────────────────────────────────
    pub accel_x: f64, // m/s² (+x = forward)
    pub accel_y: f64, // m/s² (+y = left)
    pub accel_z: f64, // m/s² (+z = up)

    pub gyro_x: f64, // rad/s roll rate
    pub gyro_y: f64, // rad/s pitch rate
    pub gyro_z: f64, // rad/s yaw rate

    pub mag_x: f64, // µT (earth field + bias)
    pub mag_y: f64,
    pub mag_z: f64,

    // ─ Calibrated values (bias removed) ────────────────────────────────────
    pub accel_cal: [f64; 3],
    pub gyro_cal: [f64; 3],

    // ─ Attitude (Madgwick AHRS) ─────────────────────────────────────────────
    pub q: Quaternion,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64, // heading (0-360, 0=North)

    // ─ Linear acceleration (gravity removed) ────────────────────────────────
    pub lin_accel_x: f64, // m/s² in world frame
    pub lin_accel_y: f64,
    pub lin_accel_z: f64,

    // ─ Temperature ──────────────────────────────────────────────────────────
    pub temperature_c: f64,

    // ─ Derived ─────────────────────────────────────────────────────────────
    pub lateral_g: f64,      // g-force lateral (positive = right)
    pub longitudinal_g: f64, // g-force longitudinal (positive = forward)
    pub vertical_g: f64,     // g-force vertical

    // ─ Fault detection ───────────────────────────────────────────────────────
    pub accel_fault: bool,
    pub gyro_fault: bool,
    pub mag_fault: bool,

    // ─ Internal simulation state ─────────────────────────────────────────────
    accel_bias: [f64; 3],
    gyro_bias: [f64; 3],
    noise_t: f64,
    vibe_amp: f64, // vibration amplitude from engine
}

impl Default for Imu {
    fn default() -> Self {
        Self::new()
    }
}

impl Imu {
    pub fn new() -> Self {
        let mut imu = Imu {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 9.81,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            mag_x: 20.0,
            mag_y: -5.0,
            mag_z: -40.0, // Brazil magnetic field
            accel_cal: [0.0; 3],
            gyro_cal: [0.0; 3],
            q: Quaternion::identity(),
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
            lin_accel_x: 0.0,
            lin_accel_y: 0.0,
            lin_accel_z: 0.0,
            temperature_c: 25.0,
            lateral_g: 0.0,
            longitudinal_g: 0.0,
            vertical_g: 1.0,
            accel_fault: false,
            gyro_fault: false,
            mag_fault: false,
            accel_bias: [0.002, -0.001, 0.015],
            gyro_bias: [0.0003, -0.0002, 0.0001],
            noise_t: 0.0,
            vibe_amp: 0.0,
        };
        imu.q.normalise();
        imu
    }

    /// Update IMU with vehicle dynamics.
    /// `ax_ms2` = longitudinal acceleration, `yaw_rate` = deg/s, `rpm` = engine RPM
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        ax_ms2: f64,
        lat_acc_ms2: f64,
        yaw_rate_degs: f64,
        heading_deg: f64,
        grade_pct: f64,
        rpm: f64,
        dt: f64,
    ) {
        self.noise_t += dt;
        let n = self.noise_t;

        // ─ Engine vibration model (fundamental + harmonics) ─────────────────
        let vibe_freq = rpm / 60.0 * (Imu::ncyl() as f64 / 2.0); // firing frequency Hz
        self.vibe_amp = (rpm / 2200.0 * 0.15).min(0.3);
        let vibe = self.vibe_amp
            * ((2.0 * PI * vibe_freq * n).sin() + 0.3 * (4.0 * PI * vibe_freq * n).sin());

        // ─ True accelerations in body frame ──────────────────────────────────
        // Gravity projects onto body Z (cosine of pitch) and X (sine of pitch)
        let pitch_rad = (grade_pct / 100.0).atan();
        let g_component = 9.81 * pitch_rad.sin();
        let true_ax = ax_ms2 - g_component;
        let true_ay = lat_acc_ms2;
        let true_az = -9.81 * pitch_rad.cos(); // reduced Z-component on grade

        // ─ Add noise, bias and vibration ─────────────────────────────────────
        let an = |seed: f64, amp: f64| {
            (((seed * 127.1 + 311.7).sin() * 43758.5).fract() - 0.5) * 2.0 * amp
        };
        self.accel_x = true_ax + self.accel_bias[0] + an(n * 1.1, ACCEL_NOISE) + vibe;
        self.accel_y = true_ay + self.accel_bias[1] + an(n * 2.3, ACCEL_NOISE) + vibe * 0.6;
        self.accel_z = true_az + self.accel_bias[2] + an(n * 3.7, ACCEL_NOISE) + vibe * 0.8;

        let yaw_rate_rad = yaw_rate_degs * PI / 180.0;
        // Roll rate: derived from lateral acceleration and speed (bank angle dynamics)
        let roll_rate_rad = lat_acc_ms2 / (ax_ms2.hypot(lat_acc_ms2) + 9.81).max(1.0) * 0.5;
        // Pitch rate: derived from longitudinal acceleration on grade changes
        let pitch_rate_rad = -ax_ms2 / 9.81 * 0.3;
        // Gyro bias drifts slowly
        self.gyro_bias[2] += an(n * 0.5, GYRO_DRIFT) * dt;
        self.gyro_x = roll_rate_rad  + self.gyro_bias[0] + an(n * 4.1, GYRO_NOISE);
        self.gyro_y = pitch_rate_rad + self.gyro_bias[1] + an(n * 5.3, GYRO_NOISE);
        self.gyro_z = yaw_rate_rad   + self.gyro_bias[2] + an(n * 6.7, GYRO_NOISE);

        // Calibrated values
        self.accel_cal = [
            self.accel_x - self.accel_bias[0],
            self.accel_y - self.accel_bias[1],
            self.accel_z - self.accel_bias[2],
        ];
        self.gyro_cal = [
            self.gyro_x - self.gyro_bias[0],
            self.gyro_y - self.gyro_bias[1],
            self.gyro_z - self.gyro_bias[2],
        ];

        // ─ Magnetometer (earth field + vehicle field distortion) ─────────────
        let yaw_rad = heading_deg * PI / 180.0;
        self.mag_x = 20.0 * yaw_rad.cos() + an(n * 7.1, MAG_NOISE);
        self.mag_y = 20.0 * (-yaw_rad).sin() + an(n * 8.3, MAG_NOISE);
        self.mag_z = -40.0 + an(n * 9.5, MAG_NOISE);

        // ─ Madgwick AHRS filter ───────────────────────────────────────────────
        self.madgwick_update(dt);

        // ─ Extract Euler angles ───────────────────────────────────────────────
        let (r, p, y) = self.q.to_euler_deg();
        self.roll_deg = r;
        self.pitch_deg = p;
        self.yaw_deg = (y + 360.0) % 360.0;

        // ─ Remove gravity to get linear acceleration ─────────────────────────
        let (qw, qx, qy, qz) = (self.q.w, self.q.x, self.q.y, self.q.z);
        let gx = 2.0 * (qx * qz - qw * qy);
        let gy = 2.0 * (qw * qx + qy * qz);
        let gz = qw * qw - qx * qx - qy * qy + qz * qz;
        self.lin_accel_x = self.accel_cal[0] - gx * 9.81;
        self.lin_accel_y = self.accel_cal[1] - gy * 9.81;
        self.lin_accel_z = self.accel_cal[2] - gz * 9.81;

        // ─ G-forces ───────────────────────────────────────────────────────────
        self.longitudinal_g = self.lin_accel_x / 9.81;
        self.lateral_g = self.lin_accel_y / 9.81;
        self.vertical_g = self.accel_cal[2].abs() / 9.81;

        // IMU temperature: ambient + electronics self-heat (~7°C); no coupling to engine RPM
        let _ = rpm; // rpm no longer used for temperature (only for vibration model above)
        let imu_self_heat = 7.0;
        let ambient_approx = 25.0; // ideally from BCM ambient sensor
        self.temperature_c += (ambient_approx + imu_self_heat - self.temperature_c) * dt * 0.005;

        // ─ Fault detection ───────────────────────────────────────────────────
        let accel_mag = (self.accel_x.powi(2) + self.accel_y.powi(2) + self.accel_z.powi(2)).sqrt();
        self.accel_fault = !(4.0..=25.0).contains(&accel_mag);
        self.gyro_fault = self.gyro_z.abs() > 10.0; // >10 rad/s = sensor fault
    }

    /// Madgwick AHRS algorithm — fuses accel + gyro + mag into quaternion attitude.
    fn madgwick_update(&mut self, dt: f64) {
        let [ax, ay, az] = self.accel_cal;
        let [gx, gy, gz] = self.gyro_cal;
        let (mx, my, mz) = (self.mag_x, self.mag_y, self.mag_z);

        let q0 = self.q.w;
        let q1 = self.q.x;
        let q2 = self.q.y;
        let q3 = self.q.z;

        // Normalise accelerometer
        let an = (ax * ax + ay * ay + az * az).sqrt();
        if an < 1e-10 {
            return;
        }
        let (ax, ay, az) = (ax / an, ay / an, az / an);

        // Normalise magnetometer
        let mn = (mx * mx + my * my + mz * mz).sqrt();
        if mn < 1e-10 {
            return;
        }
        let (mx, my, mz) = (mx / mn, my / mn, mz / mn);

        // Reference direction of earth's magnetic field
        let hx = 2.0 * mx * (0.5 - q2 * q2 - q3 * q3)
            + 2.0 * my * (q1 * q2 - q0 * q3)
            + 2.0 * mz * (q1 * q3 + q0 * q2);
        let hy = 2.0 * mx * (q1 * q2 + q0 * q3)
            + 2.0 * my * (0.5 - q1 * q1 - q3 * q3)
            + 2.0 * mz * (q2 * q3 - q0 * q1);
        let hz = 2.0 * mx * (q1 * q3 - q0 * q2)
            + 2.0 * my * (q2 * q3 + q0 * q1)
            + 2.0 * mz * (0.5 - q1 * q1 - q2 * q2);
        let bx = (hx * hx + hy * hy).sqrt();
        let bz = hz;

        // Gradient descent objective function
        let f1 = 2.0 * (q1 * q3 - q0 * q2) - ax;
        let f2 = 2.0 * (q0 * q1 + q2 * q3) - ay;
        let f3 = 1.0 - 2.0 * (q1 * q1 + q2 * q2) - az;
        let f4 = 2.0 * bx * (0.5 - q2 * q2 - q3 * q3) + 2.0 * bz * (q1 * q3 - q0 * q2) - mx;
        let f5 = 2.0 * bx * (q1 * q2 - q0 * q3) + 2.0 * bz * (q0 * q1 + q2 * q3) - my;
        let f6 = 2.0 * bx * (q0 * q2 + q1 * q3) + 2.0 * bz * (0.5 - q1 * q1 - q2 * q2) - mz;

        // Jacobian and gradient
        let j11 = -2.0 * q2;
        let j12 = 2.0 * q3;
        let j13 = -2.0 * q0;
        let j14 = 2.0 * q1;
        let j21 = 2.0 * q0;
        let j22 = 2.0 * q1;
        let j23 = 2.0 * q2;
        let j24 = 2.0 * q3;
        let j31 = 0.0;
        let j32 = -4.0 * q1;
        let j33 = -4.0 * q2;
        let j34 = 0.0;

        let mut s0 = j11 * f1 + j21 * f2 + j31 * f3 - 2.0 * bz * q2 * f4
            + (-2.0 * bx * q3 + 2.0 * bz * q1) * f5
            + 2.0 * bx * q2 * f6;
        let mut s1 = j12 * f1
            + j22 * f2
            + j32 * f3
            + 2.0 * bz * q3 * f4
            + (2.0 * bx * q2 + 2.0 * bz * q0) * f5
            + (2.0 * bx * q3 - 4.0 * bz * q1) * f6;
        let mut s2 = j13 * f1
            + j23 * f2
            + j33 * f3
            + (-4.0 * bx * q2 - 2.0 * bz * q0) * f4
            + (2.0 * bx * q1 + 2.0 * bz * q3) * f5
            + (2.0 * bx * q0 - 4.0 * bz * q2) * f6;
        let mut s3 = j14 * f1
            + j24 * f2
            + j34 * f3
            + (-4.0 * bx * q3 + 2.0 * bz * q1) * f4
            + (-2.0 * bx * q0 + 2.0 * bz * q2) * f5
            + 2.0 * bx * q1 * f6;
        let sn = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3).sqrt().max(1e-10);
        s0 /= sn;
        s1 /= sn;
        s2 /= sn;
        s3 /= sn;

        // Rate of change of quaternion from gyro
        let qdot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - BETA * s0;
        let qdot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - BETA * s1;
        let qdot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - BETA * s2;
        let qdot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - BETA * s3;

        self.q.w += qdot0 * dt;
        self.q.x += qdot1 * dt;
        self.q.y += qdot2 * dt;
        self.q.z += qdot3 * dt;
        self.q.normalise();
    }

    fn ncyl() -> u8 {
        6
    } // 6-cylinder
}
