use super::hw::{Frame, HardwareInterface, HwConfig, HwError};

#[cfg(target_os = "linux")]
use super::hw::CanFrame;

#[cfg(target_os = "linux")]
use socketcan::{CanFrame as LinuxCanFrame, CanSocket, EmbeddedFrame, ExtendedId, Socket};

#[derive(Debug, Default)]
pub struct SocketCanAdapter {
    #[cfg(target_os = "linux")]
    socket: Option<CanSocket>,
}

impl SocketCanAdapter {
    #[cfg(target_os = "linux")]
    pub fn open(iface: &str) -> Result<Self, HwError> {
        let socket = CanSocket::open(iface).map_err(|e| HwError::PortNotFound(e.to_string()))?;
        socket
            .set_read_timeout(std::time::Duration::from_millis(250))
            .map_err(|_e| HwError::TransceiverError)?;
        Ok(Self {
            socket: Some(socket),
        })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn open(_iface: &str) -> Result<Self, HwError> {
        Err(HwError::Unknown(
            "SocketCAN real adapter is supported on Linux only".to_string(),
        ))
    }
}

impl HardwareInterface for SocketCanAdapter {
    fn init(&mut self, _config: &HwConfig) -> Result<(), HwError> {
        Ok(())
    }

    #[cfg(target_os = "linux")]
    fn read_frame(&mut self) -> Result<Frame, HwError> {
        let Some(socket) = self.socket.as_ref() else {
            return Err(HwError::PortNotFound("CAN socket not initialized".into()));
        };

        let f = socket.read_frame().map_err(|e| {
            if e.kind() == std::io::ErrorKind::TimedOut {
                HwError::Timeout
            } else {
                HwError::TransceiverError
            }
        })?;

        let mut data = [0u8; 8];
        let payload = f.data();
        let len = payload.len().min(8);
        data[..len].copy_from_slice(&payload[..len]);

        let id = match f.id() {
            socketcan::Id::Standard(stdid) => u32::from(stdid.as_raw()),
            socketcan::Id::Extended(extid) => extid.as_raw(),
        };

        Ok(Frame::Can(CanFrame {
            id,
            dlc: len as u8,
            data,
            len,
            timestamp_ms: Some(now_ms()),
        }))
    }

    #[cfg(not(target_os = "linux"))]
    fn read_frame(&mut self) -> Result<Frame, HwError> {
        Err(HwError::Unknown(
            "SocketCAN real adapter is supported on Linux only".to_string(),
        ))
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        match self.read_frame() {
            Ok(f) => Ok(Some(f)),
            Err(HwError::Timeout) => Ok(None),
            Err(e) => Err(e),
        }
    }

    #[cfg(target_os = "linux")]
    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError> {
        let Some(socket) = self.socket.as_ref() else {
            return Err(HwError::PortNotFound("CAN socket not initialized".into()));
        };

        let cf = match frame {
            Frame::Can(c) => c,
            Frame::Serial(_) => {
                return Err(HwError::ParseError {
                    cause: "cannot send serial frame over SocketCAN".to_string(),
                    raw_data: "serial-frame".to_string(),
                })
            }
        };

        let Some(ext_id) = ExtendedId::new(cf.id) else {
            return Err(HwError::ParseError {
                cause: "invalid CAN extended id".to_string(),
                raw_data: format!("{}", cf.id),
            });
        };

        let Some(frame) = LinuxCanFrame::new(ext_id, &cf.data[..cf.len.min(8)]) else {
            return Err(HwError::ParseError {
                cause: "failed to build CAN frame".to_string(),
                raw_data: format!("id={}", cf.id),
            });
        };

        socket
            .write_frame(&frame)
            .map_err(|_e| HwError::TransceiverError)
    }

    #[cfg(not(target_os = "linux"))]
    fn send_frame(&mut self, _frame: Frame) -> Result<(), HwError> {
        Err(HwError::Unknown(
            "SocketCAN real adapter is supported on Linux only".to_string(),
        ))
    }

    fn close(&mut self) -> Result<(), HwError> {
        #[cfg(target_os = "linux")]
        {
            self.socket = None;
        }
        Ok(())
    }
}

#[cfg(target_os = "linux")]
fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
