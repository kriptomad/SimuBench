#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }

    let raw_id = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let payload = if data.len() > 4 { &data[4..] } else { &[] };

    let frame = auto_breaking::j1939::J1939Frame::from_raw(0.0, raw_id, payload);
    let _ = frame.pgn_name();
    let _ = frame.sa_name();
    let _ = frame.data_hex();
});
