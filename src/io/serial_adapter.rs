use std::io::{Read, Write};
use std::time::Duration;

use serialport::SerialPort;

use super::hw::{Frame, HardwareInterface, HwConfig, HwError, SerialFrame};

#[derive(Debug, Default)]
pub struct SerialAdapter {
    port: Option<Box<dyn SerialPort>>,
}

impl SerialAdapter {
    pub fn open(port_name: &str, baud: u32) -> Result<Self, HwError> {
        let port = serialport::new(port_name, baud)
            .timeout(Duration::from_millis(250))
            .open()
            .map_err(|e| {
                let msg = e.to_string();
                if msg.to_ascii_lowercase().contains("permission") {
                    HwError::PermissionDenied(msg)
                } else {
                    HwError::PortNotFound(msg)
                }
            })?;
        Ok(Self { port: Some(port) })
    }
}

impl HardwareInterface for SerialAdapter {
    fn init(&mut self, _config: &HwConfig) -> Result<(), HwError> {
        Ok(())
    }

    fn read_frame(&mut self) -> Result<Frame, HwError> {
        let Some(port) = self.port.as_mut() else {
            return Err(HwError::PortNotFound("serial port not initialized".into()));
        };

        let mut buf = vec![0u8; 1024];
        let n = port.read(&mut buf).map_err(|e| {
            if e.kind() == std::io::ErrorKind::TimedOut {
                HwError::Timeout
            } else {
                HwError::TransceiverError
            }
        })?;

        if n == 0 {
            return Err(HwError::Timeout);
        }

        buf.truncate(n);
        Ok(Frame::Serial(SerialFrame {
            bytes: buf,
            protocol_hint: Some("raw-serial".into()),
            timestamp_ms: Some(now_ms()),
        }))
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        match self.read_frame() {
            Ok(f) => Ok(Some(f)),
            Err(HwError::Timeout) => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError> {
        let Some(port) = self.port.as_mut() else {
            return Err(HwError::PortNotFound("serial port not initialized".into()));
        };

        let bytes: Vec<u8> = match frame {
            Frame::Serial(sf) => sf.bytes,
            Frame::Can(cf) => cf.data[..cf.len.min(8)].to_vec(),
        };

        port.write_all(&bytes)
            .map_err(|_e| HwError::TransceiverError)?;
        port.flush().map_err(|_e| HwError::TransceiverError)?;
        Ok(())
    }

    fn close(&mut self) -> Result<(), HwError> {
        self.port = None;
        Ok(())
    }
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
