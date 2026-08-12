use std::collections::VecDeque;
use std::path::PathBuf;

use super::allowlist::Allowlist;
use super::hw::{Frame, HardwareInterface, HwConfig, HwError};
use super::metrics::HwMetrics;
use super::rate_limiter::TwoLevelRateLimiter;
use super::replay::{append_record, frame_to_record};

#[derive(Debug, Default)]
pub struct MockAdapter {
    config: Option<HwConfig>,
    closed: bool,
    disconnected: bool,
    pub rx_queue: VecDeque<Frame>,
    pub tx_queue: VecDeque<Frame>,
    allowlist: Option<Allowlist>,
    limiter: Option<TwoLevelRateLimiter>,
    log_path: Option<PathBuf>,
    pub metrics: HwMetrics,
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
        self.allowlist = if let Some(path) = &config.allowlist_path {
            Some(
                Allowlist::from_path(path)
                    .map_err(|e| HwError::Unknown(format!("load allowlist failed: {e}")))?,
            )
        } else {
            None
        };
        self.limiter = Some(TwoLevelRateLimiter::new(
            config.rate_limit_global_per_sec,
            config.rate_limit_per_id_per_sec,
        ));
        self.log_path = Some(config.log_dir.join("mock-session.jsonl"));
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Frame, HwError> {
        if self.disconnected {
            self.metrics.on_read_error();
            return Err(HwError::Timeout);
        }

        let frame = self.rx_queue.pop_front().ok_or_else(|| {
            self.metrics.on_read_error();
            HwError::Timeout
        })?;

        match &frame {
            Frame::Can(cf) => self
                .metrics
                .on_rx_can(cf.timestamp_ms.unwrap_or_else(now_ms)),
            Frame::Serial(sf) => self
                .metrics
                .on_rx_serial(sf.timestamp_ms.unwrap_or_else(now_ms)),
        }

        if let Some(path) = &self.log_path {
            let rec = frame_to_record(&frame, "rx", None, None);
            let _ = append_record(path, &rec);
        }

        Ok(frame)
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        if self.disconnected {
            self.metrics.on_read_error();
            return Err(HwError::Timeout);
        }

        let out = self.rx_queue.pop_front();
        if let Some(frame) = &out {
            match frame {
                Frame::Can(cf) => self
                    .metrics
                    .on_rx_can(cf.timestamp_ms.unwrap_or_else(now_ms)),
                Frame::Serial(sf) => self
                    .metrics
                    .on_rx_serial(sf.timestamp_ms.unwrap_or_else(now_ms)),
            }
            if let Some(path) = &self.log_path {
                let rec = frame_to_record(frame, "rx", None, None);
                let _ = append_record(path, &rec);
            }
        }

        Ok(out)
    }

    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError> {
        if self.disconnected {
            self.metrics.on_write_error();
            return Err(HwError::TransceiverError);
        }

        let cfg = self
            .config
            .as_ref()
            .ok_or_else(|| HwError::Unknown("adapter not initialized".to_string()))?;

        if !cfg.write_intent_enabled() {
            self.metrics.on_tx_blocked();
            return Err(HwError::WriteBlockedAllowlist);
        }

        let is_allowed = self
            .allowlist
            .as_ref()
            .is_some_and(|al| al.is_allowed(&frame));
        if !is_allowed {
            self.metrics.on_tx_blocked();
            return Err(HwError::WriteBlockedAllowlist);
        }

        let is_rate_ok = if let Some(lim) = self.limiter.as_mut() {
            match &frame {
                Frame::Can(cf) => lim.check_can(cf.id),
                Frame::Serial(_) => lim.check_serial(),
            }
        } else {
            true
        };

        if !is_rate_ok {
            self.metrics.on_rate_limited();
            self.metrics.on_write_error();
            return Err(HwError::RateLimited);
        }

        if let Some(path) = &self.log_path {
            let rec = frame_to_record(&frame, "tx", Some(true), Some(cfg.dry_run));
            let _ = append_record(path, &rec);
        }

        if cfg.dry_run {
            match &frame {
                Frame::Can(_) => self.metrics.on_tx_can_allowed(),
                Frame::Serial(_) => self.metrics.on_tx_serial_allowed(),
            }
            return Ok(());
        }

        self.tx_queue.push_back(frame.clone());
        match frame {
            Frame::Can(_) => self.metrics.on_tx_can_allowed(),
            Frame::Serial(_) => self.metrics.on_tx_serial_allowed(),
        }

        Ok(())
    }

    fn close(&mut self) -> Result<(), HwError> {
        self.closed = true;
        Ok(())
    }
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
