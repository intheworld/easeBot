//! Apply ONLY the four POS_VEL gains of the referenced Seeed SDK DM profile.
//! Source: vectorBH6/reBotArm_control_py@40ab6ce58fec3c58cb603efb3f30240d6f5849e4
//! Requires exclusively owned port and fresh DISABLED six-axis feedback.
//! RAM writes only. No motor enable/disable, movement, mode change or EEPROM save.
#[allow(dead_code)]
#[path = "parameter_dump.rs"]
mod parameter_dump;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::dm_serial::DmSerialBus;
use motor_vendor_damiao::protocol as dm;
use parameter_dump::Result;
use std::fs::OpenOptions;
use std::io::Write;
use std::time::{Duration, Instant};

const REGISTERS: [(u8, &str); 4] = [
    (25, "KP_ASR"),
    (26, "KI_ASR"),
    (27, "KP_APR"),
    (28, "KI_APR"),
];
fn target(id: u16, rid: u8) -> Result<f32> {
    if !(1..=6).contains(&id) {
        return Err("Only arm motors 1..6 may be changed".into());
    }
    match (id <= 3, rid) {
        (true, 25) => Ok(0.0125),
        (true, 26) => Ok(0.004),
        (true, 27) => Ok(150.),
        (true, 28) => Ok(0.5),
        (false, 25) => Ok(0.0008),
        (false, 26) => Ok(0.002),
        (false, 27) => Ok(50.),
        (false, 28) => Ok(1.),
        _ => Err("Only four allowlisted POS_VEL gain registers may be changed".into()),
    }
}
fn frame(id: u32, data: [u8; 8]) -> CanFrame {
    CanFrame {
        arbitration_id: id,
        data,
        dlc: 8,
        is_extended: false,
        is_rx: false,
    }
}
fn disabled(bus: &DmSerialBus, id: u16) -> Result<()> {
    bus.send(frame(0x7ff, dm::encode_feedback_request_cmd(id)))?;
    let deadline = Instant::now() + Duration::from_millis(250);
    while Instant::now() < deadline {
        if let Some(f) = bus.recv(Duration::from_millis(5))? {
            if f.is_rx
                && !f.is_extended
                && f.dlc == 8
                && f.arbitration_id == u32::from(0x10 + id)
                && !dm::is_register_reply(&f.data)
                && !dm::is_register_write_ack(&f.data)
                && f.data[0] & 15 == id as u8
            {
                if f.data[0] >> 4 != 0 {
                    return Err(
                        format!("Motor {id} is enabled or faulted; refusing gain writes").into(),
                    );
                }
                return Ok(());
            }
        }
    }
    Err(format!("Motor {id}: no disabled-state feedback; refusing gain writes").into())
}
fn read(bus: &DmSerialBus, id: u16, rid: u8) -> Result<[u8; 4]> {
    parameter_dump::read(bus, id, rid)?
        .ok_or_else(|| format!("Register read timed out: motor={id}, rid={rid}").into())
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 || args[3] != "--apply-seeedsdk-profile" {
        return Err("usage: align_arm_gains PORT BACKUP.csv --apply-seeedsdk-profile; stopped, supported arm required".into());
    }
    let mut backup = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])?;
    let bus = DmSerialBus::open(&args[1], 921600)?;
    // Drain a bounded amount of pre-existing input before requesting fresh state.
    let end = Instant::now() + Duration::from_millis(100);
    for _ in 0..512 {
        if Instant::now() >= end {
            break;
        }
        let _ = bus.recv(Duration::from_millis(1))?;
    }
    writeln!(
        backup,
        "motor_id,register,name,before,requested,before_raw_le"
    )?;
    for id in 1..=6 {
        disabled(&bus, id)?;
        if u32::from_le_bytes(read(&bus, id, 10)?) != 2 {
            return Err(format!("Motor {id} is not POS_VEL; refusing gain writes").into());
        }
        for (rid, name) in REGISTERS {
            let bytes = read(&bus, id, rid)?;
            let before = f32::from_le_bytes(bytes);
            if !before.is_finite() {
                return Err("Non-finite existing gain".into());
            }
            writeln!(
                backup,
                "{id},{rid},{name},{before},{},{:02x}{:02x}{:02x}{:02x}",
                target(id, rid)?,
                bytes[0],
                bytes[1],
                bytes[2],
                bytes[3]
            )?;
        }
    }
    backup.flush()?;
    backup.sync_all()?; // All six motors backed up durably BEFORE the first write.
    println!("motor_id,register,name,requested,actual,status");
    for id in 1..=6 {
        disabled(&bus, id)?;
        for (rid, name) in REGISTERS {
            let value = target(id, rid)?;
            // A failed send propagates immediately. No retry/cleanup writes on
            // an ambiguous partial serial frame. The backup remains available.
            bus.send(frame(
                0x7ff,
                dm::encode_register_write_cmd(id, rid, value.to_le_bytes()),
            ))?;
            std::thread::sleep(Duration::from_millis(20));
            let actual = f32::from_le_bytes(read(&bus, id, rid)?);
            let matches =
                actual.is_finite() && (actual - value).abs() <= 1e-6_f32.max(value.abs() * 1e-6);
            println!(
                "{id},{rid},{name},{value},{actual},{}",
                if matches { "verified" } else { "mismatch" }
            );
            std::io::stdout().flush()?;
            if !matches {
                return Err(format!(
                    "Motor {id}, register {rid}: write verification failed; stopping"
                )
                .into());
            }
        }
        disabled(&bus, id)?;
    }
    eprintln!("24 gain writes verified. No enable/motion/mode/store commands sent.");
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_reference_values_and_rejected_other_writes() {
        for id in 1..=6 {
            for rid in 0..=255 {
                assert_eq!(target(id, rid).is_ok(), (25..=28).contains(&rid));
            }
            let expected = if id <= 3 {
                [0.0125, 0.004, 150., 0.5]
            } else {
                [0.0008, 0.002, 50., 1.]
            };
            for (i, (rid, _)) in REGISTERS.iter().enumerate() {
                assert_eq!(target(id, *rid).unwrap(), expected[i]);
            }
        }
        assert!(target(0, 25).is_err());
        assert!(target(7, 25).is_err());
    }
}
