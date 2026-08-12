#[derive(Debug, Default, Clone)]
pub struct HwMetrics {
    pub hw_rx_frames_total_can: u64,
    pub hw_rx_frames_total_serial: u64,
    pub hw_tx_frames_total_can_allowed: u64,
    pub hw_tx_frames_total_serial_allowed: u64,
    pub hw_tx_frames_total_blocked: u64,
    pub hw_rate_limited_total: u64,
    pub hw_read_errors_total: u64,
    pub hw_write_errors_total: u64,
    pub hw_last_rx_timestamp_ms: u64,
}

impl HwMetrics {
    pub fn on_rx_can(&mut self, ts_ms: u64) {
        self.hw_rx_frames_total_can += 1;
        self.hw_last_rx_timestamp_ms = ts_ms;
    }

    pub fn on_rx_serial(&mut self, ts_ms: u64) {
        self.hw_rx_frames_total_serial += 1;
        self.hw_last_rx_timestamp_ms = ts_ms;
    }

    pub fn on_tx_can_allowed(&mut self) {
        self.hw_tx_frames_total_can_allowed += 1;
    }

    pub fn on_tx_serial_allowed(&mut self) {
        self.hw_tx_frames_total_serial_allowed += 1;
    }

    pub fn on_tx_blocked(&mut self) {
        self.hw_tx_frames_total_blocked += 1;
    }

    pub fn on_rate_limited(&mut self) {
        self.hw_rate_limited_total += 1;
    }

    pub fn on_read_error(&mut self) {
        self.hw_read_errors_total += 1;
    }

    pub fn on_write_error(&mut self) {
        self.hw_write_errors_total += 1;
    }
}
