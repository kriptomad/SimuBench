//! LIDAR — 3D rotating LIDAR simulation (Velodyne VLP-16 / Ouster OS1-64 style).
//! Generates real point clouds by raycasting against simulated environment obstacles.
//! 16 vertical channels, 360° horizontal, 10 Hz rotation rate.

use std::f64::consts::PI;

// ── Point ─────────────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub struct LidarPoint {
    pub x: f32, // metres (forward = +x, left = +y, up = +z)
    pub y: f32,
    pub z: f32,
    pub intensity: u8, // 0-255 return intensity
    pub ring: u8,      // beam index (0=bottom, 15=top for VLP-16)
    pub azimuth: f32,  // horizontal angle degrees 0-360
    pub distance: f32, // slant range metres
}

// ── Detected obstacle cluster ─────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct LidarCluster {
    pub id: u32,
    pub centroid_x: f64, // relative to sensor
    pub centroid_y: f64,
    pub centroid_z: f64,
    pub length_m: f64,
    pub width_m: f64,
    pub height_m: f64,
    pub distance_m: f64, // front-to-front distance
    pub object_type: LidarObjectType,
    pub point_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LidarObjectType {
    Vehicle,
    Pedestrian,
    Cyclist,
    Barrier,
    GroundPoint,
    Unknown,
}

impl std::fmt::Display for LidarObjectType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LidarObjectType::Vehicle => write!(f, "Vehicle   "),
            LidarObjectType::Pedestrian => write!(f, "Pedestrian"),
            LidarObjectType::Cyclist => write!(f, "Cyclist   "),
            LidarObjectType::Barrier => write!(f, "Barrier   "),
            LidarObjectType::GroundPoint => write!(f, "Ground    "),
            LidarObjectType::Unknown => write!(f, "Unknown   "),
        }
    }
}

// ── World obstacle (for ray casting) ─────────────────────────────────────────
pub struct WorldObstacle {
    pub x: f64, // centre relative to ego vehicle
    pub y: f64,
    pub z_min: f64,
    pub z_max: f64,
    pub half_w: f64,       // half-width in y
    pub half_l: f64,       // half-length in x
    pub reflectivity: f64, // 0-1 (affects intensity)
}

impl WorldObstacle {
    pub fn new_vehicle(x: f64, y: f64, _speed_ms: f64) -> Self {
        WorldObstacle {
            x,
            y,
            z_min: 0.0,
            z_max: 1.5,
            half_w: 0.9,
            half_l: 2.5,
            reflectivity: 0.5,
        }
    }
    pub fn new_pedestrian(x: f64, y: f64) -> Self {
        WorldObstacle {
            x,
            y,
            z_min: 0.0,
            z_max: 1.8,
            half_w: 0.25,
            half_l: 0.25,
            reflectivity: 0.3,
        }
    }
    pub fn new_barrier(x: f64, y: f64, length: f64) -> Self {
        WorldObstacle {
            x,
            y,
            z_min: 0.0,
            z_max: 0.8,
            half_w: 0.15,
            half_l: length / 2.0,
            reflectivity: 0.8,
        }
    }

    /// Returns distance to ray intersection, or None if no hit
    pub fn ray_intersect(
        &self,
        origin_x: f64,
        origin_y: f64,
        dir_x: f64,
        dir_y: f64,
        z: f64,
    ) -> Option<f64> {
        if z < self.z_min - 0.1 || z > self.z_max + 0.1 {
            return None;
        }
        // AABB intersection
        let inv_dx = if dir_x.abs() > 1e-10 {
            1.0 / dir_x
        } else {
            1e10
        };
        let inv_dy = if dir_y.abs() > 1e-10 {
            1.0 / dir_y
        } else {
            1e10
        };
        let t1x = ((self.x - self.half_l) - origin_x) * inv_dx;
        let t2x = ((self.x + self.half_l) - origin_x) * inv_dx;
        let t1y = ((self.y - self.half_w) - origin_y) * inv_dy;
        let t2y = ((self.y + self.half_w) - origin_y) * inv_dy;
        let tmin = t1x.min(t2x).max(t1y.min(t2y));
        let tmax = t1x.max(t2x).min(t1y.max(t2y));
        if tmax >= tmin && tmin > 0.0 {
            Some(tmin)
        } else {
            None
        }
    }
}

// ── LIDAR sensor ──────────────────────────────────────────────────────────────
pub struct LidarSensor {
    // ─ Configuration ─────────────────────────────────────────────────────────
    pub channels: u8,      // 16, 32, or 64
    pub rotation_hz: f64,  // 10 or 20 Hz
    pub h_resolution: f64, // horizontal resolution degrees (0.2° for VLP-16)
    pub max_range_m: f64,  // maximum range (100m VLP-16, 200m Ouster)
    pub min_range_m: f64,
    pub active: bool,

    // ─ Output ────────────────────────────────────────────────────────────────
    pub points: Vec<LidarPoint>,
    pub clusters: Vec<LidarCluster>,
    pub points_per_scan: u32,
    pub scan_time_ms: f64,

    // ─ Occupancy grid (simplified) ───────────────────────────────────────────
    pub occupancy_grid: Vec<Vec<u8>>, // 100×100 grid, 1m cells, centre=ego
    pub grid_size: usize,

    // ─ Stats ─────────────────────────────────────────────────────────────────
    pub total_scans: u64,
    pub obstacles_seen: usize,

    // ─ Ground model ──────────────────────────────────────────────────────────
    pub ground_level: f64, // metres (accounts for hills)

    // ─ Internal ──────────────────────────────────────────────────────────────
    #[allow(dead_code)]
    rotation_angle: f64,
    scan_timer: f64,
    cluster_id_counter: u32,
    noise_t: f64,
}

impl LidarSensor {
    pub fn new_vlp16() -> Self {
        LidarSensor {
            channels: 16,
            rotation_hz: 10.0,
            h_resolution: 0.2,
            max_range_m: 100.0,
            min_range_m: 0.3,
            active: true,
            points: Vec::new(),
            clusters: Vec::new(),
            points_per_scan: 0,
            scan_time_ms: 0.0,
            occupancy_grid: vec![vec![0u8; 100]; 100],
            grid_size: 100,
            total_scans: 0,
            obstacles_seen: 0,
            ground_level: 0.0,
            rotation_angle: 0.0,
            scan_timer: 0.0,
            cluster_id_counter: 0,
            noise_t: 0.0,
        }
    }

    /// Update LIDAR — generates point cloud from world obstacles.
    pub fn update(&mut self, world: &[WorldObstacle], vehicle_pitch: f64, dt: f64) {
        self.noise_t += dt;
        self.scan_timer += dt;
        if self.scan_timer < 1.0 / self.rotation_hz {
            return;
        }
        self.scan_timer = 0.0;

        let t0 = std::time::Instant::now();
        self.points.clear();
        self.clear_grid();

        // VLP-16 vertical angles: -15° to +15° in 2° steps
        let vert_angles: Vec<f64> = (0..self.channels as i32)
            .map(|i| -15.0 + i as f64 * 2.0) // degrees
            .collect();

        let n = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        let nt = self.noise_t;

        // Full 360° scan
        let mut az = 0.0_f64;
        while az < 360.0 {
            let az_rad = az * PI / 180.0;
            let dir_x = az_rad.cos();
            let dir_y = az_rad.sin();

            for (ring, vert_deg) in vert_angles.iter().enumerate() {
                let vert_rad = (vert_deg + vehicle_pitch) * PI / 180.0;
                let h_factor = vert_rad.cos();
                let z_per_m = vert_rad.sin();

                // Find closest intersection
                let mut min_dist = self.max_range_m;
                let mut hit_intensity = 0u8;

                // Ground return
                if *vert_deg < 0.0 {
                    let ground_dist = (self.ground_level - 1.8) / (-vert_rad.sin()).max(1e-6);
                    if ground_dist < min_dist && ground_dist > self.min_range_m {
                        min_dist = ground_dist;
                        hit_intensity = 30;
                    }
                }

                // Obstacle returns
                for obs in world {
                    let z_at_range = |r: f64| self.ground_level + z_per_m * r + 1.8; // sensor height 1.8m
                                                                                     // Check at expected obstacle position
                    if let Some(dist) = obs.ray_intersect(
                        0.0,
                        0.0,
                        dir_x * h_factor,
                        dir_y * h_factor,
                        z_at_range(obs.x.hypot(obs.y)),
                    ) {
                        if dist < min_dist {
                            min_dist = dist;
                            let snr = obs.reflectivity * (1.0 - dist / self.max_range_m);
                            hit_intensity = (snr * 200.0 + 20.0).clamp(0.0, 255.0) as u8;
                        }
                    }
                }

                if min_dist < self.max_range_m && min_dist > self.min_range_m {
                    let noise_m = n(nt + az * 0.01 + ring as f64 * 0.1) * 0.02; // ±2cm noise
                    let d = (min_dist + noise_m) as f32;
                    let px = (d * dir_x as f32 * vert_rad.cos() as f32) as f32;
                    let py = (d * dir_y as f32 * vert_rad.cos() as f32) as f32;
                    let pz = (self.ground_level + z_per_m * min_dist) as f32;

                    self.points.push(LidarPoint {
                        x: px,
                        y: py,
                        z: pz,
                        intensity: hit_intensity,
                        ring: ring as u8,
                        azimuth: az as f32,
                        distance: d,
                    });

                    // Update occupancy grid
                    let gx = (50.0 + px as f64) as usize;
                    let gy = (50.0 + py as f64) as usize;
                    if gx < self.grid_size && gy < self.grid_size && pz > 0.2 {
                        self.occupancy_grid[gy][gx] =
                            self.occupancy_grid[gy][gx].saturating_add(10);
                    }
                }
            }
            az += self.h_resolution;
        }

        self.points_per_scan = self.points.len() as u32;
        self.scan_time_ms = t0.elapsed().as_secs_f64() * 1000.0;
        self.total_scans += 1;

        // Simple clustering: group nearby occupied grid cells
        self.cluster_obstacles(world);
    }

    fn cluster_obstacles(&mut self, world: &[WorldObstacle]) {
        self.clusters.clear();
        for (_, obs) in world.iter().enumerate() {
            let dist = (obs.x.powi(2) + obs.y.powi(2)).sqrt();
            if dist > self.max_range_m {
                continue;
            }
            let obj_type = if obs.z_max > 1.2 && obs.half_w > 0.5 {
                LidarObjectType::Vehicle
            } else if obs.z_max > 1.5 {
                LidarObjectType::Pedestrian
            } else if obs.z_max < 0.9 {
                LidarObjectType::Barrier
            } else {
                LidarObjectType::Unknown
            };
            let pts = self
                .points
                .iter()
                .filter(|p| {
                    ((p.x as f64 - obs.x).powi(2) + (p.y as f64 - obs.y).powi(2)).sqrt() < 3.0
                })
                .count();
            if pts > 0 {
                self.clusters.push(LidarCluster {
                    id: self.cluster_id_counter,
                    centroid_x: obs.x,
                    centroid_y: obs.y,
                    centroid_z: obs.z_max / 2.0,
                    length_m: obs.half_l * 2.0,
                    width_m: obs.half_w * 2.0,
                    height_m: obs.z_max,
                    distance_m: dist,
                    object_type: obj_type,
                    point_count: pts,
                });
                self.cluster_id_counter = self.cluster_id_counter.wrapping_add(1);
            }
        }
        self.obstacles_seen = self.clusters.len();
    }

    fn clear_grid(&mut self) {
        for row in &mut self.occupancy_grid {
            for cell in row.iter_mut() {
                *cell = (*cell).saturating_sub(5);
            }
        }
    }

    /// Get occupancy as percentage for a given grid cell
    pub fn cell_occupied(&self, row: usize, col: usize) -> bool {
        row < self.grid_size && col < self.grid_size && self.occupancy_grid[row][col] > 50
    }
}
