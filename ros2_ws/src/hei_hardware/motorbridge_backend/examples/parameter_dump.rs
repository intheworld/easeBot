//! Offline maintenance reader. Sends ONLY register-read frames; never enables,
//! disables, moves, changes mode, zeroes, stores or writes motor parameters.
//! Stop the controller owning this serial port before running this example.
use motor_core::bus::{CanBus, CanFrame};
use motor_core::dm_serial::DmSerialBus;
use motor_vendor_damiao::protocol as dm;
use std::io::{self, Write};
use std::time::{Duration, Instant};

pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const REGISTERS: &[(u8, &str, bool)] = &[
    (4, "ACC", false),
    (5, "DEC", false),
    (6, "MAX_SPD", false),
    (7, "MST_ID", true),
    (8, "ESC_ID", true),
    (9, "TIMEOUT", true),
    (10, "CTRL_MODE", true),
    (11, "Damp", false),
    (12, "Inertia", false),
    (13, "hw_ver", true),
    (14, "sw_ver", true),
    (20, "Gr", false),
    (21, "PMAX", false),
    (22, "VMAX", false),
    (23, "TMAX", false),
    (24, "I_BW", false),
    (25, "KP_ASR", false),
    (26, "KI_ASR", false),
    (27, "KP_APR", false),
    (28, "KI_APR", false),
    (30, "GREF", false),
    (31, "Deta", false),
    (32, "V_BW", false),
    (36, "sub_ver", true),
];

fn request(id: u16, register: u8) -> CanFrame {
    CanFrame {
        arbitration_id: 0x7ff,
        data: dm::encode_register_read_cmd(id, register),
        dlc: 8,
        is_extended: false,
        is_rx: false,
    }
}
fn reply(frame: CanFrame, id: u16, register: u8) -> Option<[u8; 4]> {
    if !frame.is_rx
        || frame.is_extended
        || frame.dlc != 8
        || frame.arbitration_id != u32::from(0x10 + id)
        || u16::from_le_bytes([frame.data[0], frame.data[1]]) != id
    {
        return None;
    }
    match dm::decode_register_value(frame.data) {
        Ok((rid, bytes)) if rid == register => Some(bytes),
        _ => None,
    }
}
pub(crate) fn read(bus: &DmSerialBus, id: u16, register: u8) -> Result<Option<[u8; 4]>> {
    for _ in 0..2 {
        // A failed/partial send propagates immediately; never append to an
        // ambiguous byte stream and never send cleanup motor commands.
        bus.send(request(id, register))?;
        let deadline = Instant::now() + Duration::from_millis(200);
        while Instant::now() < deadline {
            if let Some(frame) = bus.recv(Duration::from_millis(5))? {
                if let Some(bytes) = reply(frame, id, register) {
                    return Ok(Some(bytes));
                }
            }
        }
    }
    Ok(None)
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    if !(2..=3).contains(&args.len()) {
        return Err("usage: parameter_dump PORT [BAUD=921600]; controller must be stopped".into());
    }
    let baud = args
        .get(2)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(921600);
    eprintln!(
        "Read-only register queries: port={}, baud={}, motors=1..7",
        args[1], baud
    );
    // motorbridge/serialport opens exclusively; busy ports are an error.
    let bus = DmSerialBus::open(&args[1], baud)?;
    let deadline = Instant::now() + Duration::from_millis(100);
    for _ in 0..512 {
        if Instant::now() >= deadline {
            break;
        }
        let _ = bus.recv(Duration::from_millis(1))?;
    }
    println!("motor_id,register,name,type,value,raw_le,status");
    let mut missing = 0;
    for id in 1..=7 {
        for &(register, name, integer) in REGISTERS {
            match read(&bus, id, register)? {
                Some(bytes) => {
                    let value = if integer {
                        u32::from_le_bytes(bytes).to_string()
                    } else {
                        f32::from_le_bytes(bytes).to_string()
                    };
                    println!(
                        "{id},{register},{name},{},{value},{:02x}{:02x}{:02x}{:02x},ok",
                        if integer { "u32" } else { "f32" },
                        bytes[0],
                        bytes[1],
                        bytes[2],
                        bytes[3]
                    );
                }
                None => {
                    missing += 1;
                    println!(
                        "{id},{register},{name},{},,,timeout",
                        if integer { "u32" } else { "f32" }
                    );
                }
            }
            io::stdout().flush()?;
        }
    }
    eprintln!("Finished read-only dump; missing registers={missing}; no motor state changes sent");
    if missing > 0 {
        std::process::exit(2);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn only_read_requests_and_exact_reply_matching() {
        for id in 1..=7 {
            for &(rid, _, _) in REGISTERS {
                let sent = request(id, rid);
                assert_eq!(sent.arbitration_id, 0x7ff);
                assert_eq!(sent.data, [id as u8, 0, 0x33, rid, 0, 0, 0, 0]);
                let frame = CanFrame {
                    arbitration_id: u32::from(0x10 + id),
                    is_rx: true,
                    data: [id as u8, 0, 0x33, rid, 1, 2, 3, 4],
                    ..sent
                };
                assert_eq!(reply(frame, id, rid), Some([1, 2, 3, 4]));
                assert!(reply(frame, id + 1, rid).is_none());
                assert!(reply(frame, id, rid + 1).is_none());
                assert!(reply(
                    CanFrame {
                        is_rx: false,
                        ..frame
                    },
                    id,
                    rid
                )
                .is_none());
                assert!(reply(
                    CanFrame {
                        data: [id as u8, 0, 0x55, rid, 1, 2, 3, 4],
                        ..frame
                    },
                    id,
                    rid
                )
                .is_none());
            }
        }
    }
}
