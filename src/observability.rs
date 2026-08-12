use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct StructuredEvent {
    pub timestamp: f64,
    pub level: &'static str,
    pub module: &'static str,
    pub correlation_id: u64,
    pub event: &'static str,
    pub details: String,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct SimMetrics {
    pub loop_duration_ms: f64,
    pub steps_completed: u64,
    pub error_count: u64,
    pub replay_failures: u64,
}

impl SimMetrics {
    pub fn on_step(&mut self, step_duration_ms: f64) {
        self.steps_completed = self.steps_completed.saturating_add(1);
        self.loop_duration_ms = if self.steps_completed == 1 {
            step_duration_ms
        } else {
            self.loop_duration_ms * 0.95 + step_duration_ms * 0.05
        };
    }

    pub fn on_error(&mut self) {
        self.error_count = self.error_count.saturating_add(1);
    }

    pub fn on_replay_failure(&mut self) {
        self.replay_failures = self.replay_failures.saturating_add(1);
    }
}

pub fn log_structured(ev: &StructuredEvent) {
    if let Ok(line) = serde_json::to_string(ev) {
        #[cfg(feature = "advanced_observability")]
        {
            tracing::info!(target: "auto_breaking", "{}", line);
        }
        #[cfg(not(feature = "advanced_observability"))]
        {
            println!("{}", line);
        }
    }
}
