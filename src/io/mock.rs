use std::collections::VecDeque;

use super::hw::{Frame, HardwareInterface, HwConfig, HwError};

#[derive(Debug, Default)]
pub struct MockAdapter {
    config: Option<HwConfig>,
    closed: bool,
    disconnected: bool,
    pub rx_queue: VecDeque<Frame>,
    pub tx_queue: VecDeque<Frame>,
}

impl MockAdapter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn inject_rx(&mut self, frame: Frame) {
        self.rx_queue.push_back(frame);
    }

    pub fn inject_disconnect(&mut self) {
        self.disconnected = true;
    }

    pub fn clear_disconnect(&mut self) {
        self.disconnected = false;
    }
}

impl HardwareInterface for MockAdapter {
    fn init(&mut self, config: &HwConfig) -> Result<(), HwError> {
        self.config = Some(config.clone());
        self.closed = false;
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Frame, HwError> {
        if self.disconnected {
            return Err(HwError::Timeout);
        }
        self.rx_queue.pop_front().ok_or_else(|| HwError::Timeout)
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        if self.disconnected {
            return Err(HwError::Timeout);
        }
        Ok(self.rx_queue.pop_front())
    }

    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError> {
        if self.disconnected {
            return Err(HwError::TransceiverError);
        }
        let cfg = self
            .config
            .as_ref()
            .ok_or_else(|| HwError::Unknown("adapter not initialized".to_string()))?;
        if !cfg.write_effectively_enabled() {
            return Err(HwError::WriteBlockedAllowlist);
        }
        self.tx_queue.push_back(frame);
        Ok(())
    }

    fn close(&mut self) -> Result<(), HwError> {
        self.closed = true;
        Ok(())
    }
}
