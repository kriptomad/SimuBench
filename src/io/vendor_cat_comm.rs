use super::hw::{Frame, HardwareInterface, HwConfig, HwError};

/// Windows-first template for Caterpillar Cat Comm integrations.
///
/// This adapter intentionally ships as a safe placeholder:
/// - it documents exactly where vendor SDK or bridge integration should happen;
/// - it fails closed (no writes, no silent fallback);
/// - it keeps the runtime contract stable for later real SDK wiring.
#[derive(Debug, Default)]
pub struct CatCommAdapter {
    connected: bool,
}

impl CatCommAdapter {
    pub fn open(cfg: &HwConfig) -> Result<Self, HwError> {
        let _vendor = cfg
            .vendor_name
            .as_deref()
            .unwrap_or("cat_comm")
            .to_ascii_lowercase();

        Err(HwError::Unknown(
            "Cat Comm adapter template is enabled, but real vendor binding is not implemented yet. \
Provide Cat Comm SDK DLL/header details or a vendor bridge process, then wire open/read/send in src/io/vendor_cat_comm.rs"
                .to_string(),
        ))
    }

    #[allow(dead_code)]
    fn ensure_connected(&self) -> Result<(), HwError> {
        if self.connected {
            Ok(())
        } else {
            Err(HwError::Unknown(
                "Cat Comm bridge not connected. Complete vendor binding first.".to_string(),
            ))
        }
    }
}

impl HardwareInterface for CatCommAdapter {
    fn init(&mut self, _config: &HwConfig) -> Result<(), HwError> {
        self.connected = true;
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Frame, HwError> {
        Err(HwError::Unknown(
            "Cat Comm read path not implemented in template".to_string(),
        ))
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        Ok(None)
    }

    fn send_frame(&mut self, _frame: Frame) -> Result<(), HwError> {
        Err(HwError::WriteBlockedAllowlist)
    }

    fn close(&mut self) -> Result<(), HwError> {
        self.connected = false;
        Ok(())
    }
}
