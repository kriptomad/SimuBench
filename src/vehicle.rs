/// Physical state of the ego vehicle
#[derive(Debug, Clone)]
pub struct VehicleState {
    pub velocity: f64,          // km/h longitudinal
    pub acceleration: f64,      // m/s²
    pub steering_angle: f64,    // degrees (-45..45)
    pub heading: f64,           // degrees 0-360
    pub yaw_rate: f64,          // deg/s
    pub position_x: f64,        // meters
    pub position_y: f64,        // meters
    pub lateral_g: f64,         // g
    pub longitudinal_g: f64,    // g
    pub wheel_speeds: [f64; 4], // FL,FR,RL,RR  km/h
    pub wheel_slip: [f64; 4],   // -1..1
}

impl VehicleState {
    pub fn new() -> Self {
        Self {
            velocity: 0.0,
            acceleration: 0.0,
            steering_angle: 0.0,
            heading: 0.0,
            yaw_rate: 0.0,
            position_x: 0.0,
            position_y: 0.0,
            lateral_g: 0.0,
            longitudinal_g: 0.0,
            wheel_speeds: [0.0; 4],
            wheel_slip: [0.0; 4],
        }
    }

    /// Returns speed in m/s
    pub fn velocity_ms(&self) -> f64 {
        self.velocity / 3.6
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// A simulated traffic participant
#[derive(Debug, Clone)]
pub struct TrafficObject {
    pub id: u32,
    pub distance: f64,       // m ahead (positive) / behind (negative)
    pub lateral_offset: f64, // m from ego lane center
    pub speed: f64,          // km/h
    pub closing_speed: f64,  // km/h (positive = approaching ego)
    pub width: f64,          // m
    pub is_vehicle: bool,
}

impl TrafficObject {
    pub fn new(id: u32, distance: f64, lateral_offset: f64, speed: f64) -> Self {
        Self {
            id,
            distance,
            lateral_offset,
            speed,
            closing_speed: 0.0,
            width: 1.8,
            is_vehicle: true,
        }
    }

    /// Time to collision in seconds (0 = already colliding)
    pub fn ttc(&self) -> f64 {
        if self.closing_speed <= 0.0 || self.distance <= 0.0 {
            return f64::INFINITY;
        }
        self.distance / (self.closing_speed / 3.6)
    }

    /// Is this object in the ego vehicle path?
    pub fn in_path(&self) -> bool {
        self.lateral_offset.abs() < 1.5 && self.distance > 0.0
    }
}

/// Simple traffic simulation: one lead vehicle + side threats
pub struct TrafficSimulator {
    pub objects: Vec<TrafficObject>,
    lead_behavior_timer: f64,
    lead_target_speed: f64,
}

impl TrafficSimulator {
    pub fn new() -> Self {
        let mut sim = Self {
            objects: Vec::new(),
            lead_behavior_timer: 0.0,
            lead_target_speed: 80.0,
        };
        // Lead vehicle
        sim.objects.push(TrafficObject::new(1, 80.0, 0.0, 80.0));
        // Static obstacle on side
        sim.objects.push(TrafficObject::new(2, 30.0, 3.2, 0.0));
        sim
    }

    pub fn update(&mut self, ego_speed: f64, dt: f64) {
        self.lead_behavior_timer += dt;

        // Lead vehicle behavior changes every 8 seconds
        if self.lead_behavior_timer > 8.0 {
            self.lead_behavior_timer = 0.0;
            // Alternate: slow down / speed up
            self.lead_target_speed = if self.lead_target_speed > 60.0 {
                30.0
            } else {
                90.0
            };
        }

        for obj in &mut self.objects {
            if !obj.is_vehicle {
                continue;
            }
            // Lead vehicle (id=1) tracks target speed
            if obj.id == 1 {
                let speed_diff = self.lead_target_speed - obj.speed;
                obj.speed += speed_diff * dt * 0.5;
                obj.speed = obj.speed.max(0.0);
            }
            // Relative distance changes by closing speed
            let relative_speed_ms = (ego_speed - obj.speed) / 3.6;
            obj.distance -= relative_speed_ms * dt;
            obj.closing_speed = ego_speed - obj.speed;

            // Teleport lead vehicle if it goes too far
            if obj.id == 1 {
                if obj.distance < -20.0 {
                    obj.distance = 180.0;
                    obj.speed = 80.0;
                } else if obj.distance > 250.0 {
                    obj.distance = 250.0;
                }
            }
        }
    }

    pub fn lead_vehicle(&self) -> Option<&TrafficObject> {
        self.objects.iter().find(|o| o.id == 1 && o.in_path())
    }
}
