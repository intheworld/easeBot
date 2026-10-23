//! Bounded motor-level right-gripper probe, not a ROS action server.
use motor_core::bus::{CanBus, CanFrame};
use motor_core::dm_serial::DmSerialBus;
use motor_core::error::{MotorError, Result as MotorResult};
use motor_vendor_damiao::{protocol as dm, DamiaoController, DamiaoMotor, MotorFeedbackState};
use std::sync::{
    atomic::{AtomicBool, AtomicU8, Ordering},
    Arc,
};
use std::time::{Duration, Instant};

static CANCELLED: AtomicBool = AtomicBool::new(false);
extern "C" fn cancel(_: i32) {
    CANCELLED.store(true, Ordering::Relaxed);
}
extern "C" {
    fn signal(sig: i32, handler: extern "C" fn(i32)) -> usize;
}
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
struct GuardedBus {
    bus: DmSerialBus,
    failed: AtomicBool,
    fault: AtomicU8,
}
impl CanBus for GuardedBus {
    fn send(&self, frame: CanFrame) -> MotorResult<()> {
        if self.failed.load(Ordering::Acquire) {
            return Err(MotorError::Io(
                "transport failed; no further writes allowed".into(),
            ));
        }
        let d = frame.data;
        let id = u16::from_le_bytes([d[0], d[1]]);
        let query = frame.arbitration_id == 0x7ff
            && (1..=7).contains(&id)
            && (d == dm::encode_feedback_request_cmd(id)
                || d == dm::encode_register_read_cmd(id, 10));
        let mode = frame.arbitration_id == 0x7ff
            && id == 7
            && (1..=4)
                .any(|value: u32| d == dm::encode_register_write_cmd(7, 10, value.to_le_bytes()));
        let special = frame.arbitration_id == 7
            && (d == dm::encode_enable_cmd() || d == dm::encode_disable_cmd());
        let force = frame.arbitration_id == 0x307;
        if CANCELLED.load(Ordering::Relaxed)
            && (force || (frame.arbitration_id == 7 && d == dm::encode_enable_cmd()))
        {
            return Err(MotorError::Io("probe interrupted".into()));
        }
        if frame.dlc != 8 || frame.is_extended || !(query || mode || special || force) {
            return Err(MotorError::InvalidArgument(
                "probe TX whitelist rejected frame".into(),
            ));
        }
        let result = self.bus.send(frame);
        if result.is_err() {
            self.failed.store(true, Ordering::Release);
        }
        result
    }
    fn recv(&self, timeout: Duration) -> MotorResult<Option<CanFrame>> {
        let result = self.bus.recv(timeout);
        if let Ok(Some(f)) = &result {
            if f.dlc == 8
                && !f.is_extended
                && (0x11..=0x17).contains(&f.arbitration_id)
                && !dm::is_register_reply(&f.data)
                && !dm::is_register_write_ack(&f.data)
                && u32::from(f.data[0] & 15) == f.arbitration_id - 0x10
                && f.data[0] >> 4 > 1
            {
                self.fault.store(f.data[0] >> 4, Ordering::Release);
            }
        }
        result
    }
    fn shutdown(&self) -> MotorResult<()> {
        Ok(())
    } // Drop closes the port; no flush/disable_all.
}
fn fresh(motor: &DamiaoMotor, bus: &GuardedBus) -> Result<MotorFeedbackState> {
    if bus.fault.load(Ordering::Acquire) != 0 {
        return Err("latched motor fault".into());
    }
    let state = motor
        .request_fresh_state(Duration::from_millis(200))?
        .ok_or("feedback timeout")?;
    if bus.fault.load(Ordering::Acquire) != 0
        || state.status_code > 1
        || !state.pos.is_finite()
        || !state.vel.is_finite()
        || !state.torq.is_finite()
    {
        return Err("fault or invalid feedback".into());
    }
    Ok(state)
}
fn mode(motor: &DamiaoMotor) -> Result<u32> {
    Ok(motor.get_register_u32(10, Duration::from_millis(300))?)
}
fn wait_status(motor: &DamiaoMotor, bus: &GuardedBus, expected: u8) -> Result<()> {
    let deadline = Instant::now() + Duration::from_millis(300);
    while Instant::now() < deadline {
        if fresh(motor, bus)?.status_code == expected {
            return Ok(());
        }
    }
    Err("enable/disable not acknowledged".into())
}
struct Session {
    gripper: Arc<DamiaoMotor>,
    bus: Arc<GuardedBus>,
    original_mode: u32,
    switched: bool,
    owned: bool,
}
impl Session {
    fn finish(&mut self) -> Result<()> {
        if self.owned {
            self.gripper.disable()?;
            wait_status(&self.gripper, &self.bus, 0)?;
            self.owned = false;
        }
        if self.switched {
            if fresh(&self.gripper, &self.bus)?.status_code != 0 {
                return Err("refusing mode restore while enabled".into());
            }
            self.gripper.write_register_u32(10, self.original_mode)?;
            if mode(&self.gripper)? != self.original_mode
                || fresh(&self.gripper, &self.bus)?.status_code != 0
            {
                return Err("mode/disabled state restore not verified".into());
            }
            self.switched = false;
        }
        Ok(())
    }
}
impl Drop for Session {
    fn drop(&mut self) {
        if let Err(e) = self.finish() {
            eprintln!("CLEANUP FAILED: {e}");
        }
    }
}
fn run() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--help") {
        println!("gripper_probe [--port /dev/hei_right_arm] [--move]\nDefault: read-only. --move: motor7 only, delta=-0.02rad, 2s, speed<=0.05rad/s, current ratio=0.01; disable and restore mode afterwards.");
        return Ok(());
    }
    let mut port = "/dev/hei_right_arm".to_string();
    let mut movement = false;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--move" => movement = true,
            "--port" => {
                i += 1;
                port = args.get(i).ok_or("missing --port value")?.clone();
            }
            _ => return Err("unknown argument".into()),
        }
        i += 1;
    }
    if !port.starts_with('/') {
        return Err("port must be absolute".into());
    }
    let bus = Arc::new(GuardedBus {
        bus: DmSerialBus::open(&port, 921600)?,
        failed: AtomicBool::new(false),
        fault: AtomicU8::new(0),
    });
    let controller = DamiaoController::new(bus.clone());
    let mut motors = Vec::new();
    for id in 1..=7 {
        motors.push(controller.add_motor(id, 0x10 + id, if id <= 3 { "4340" } else { "4310" })?);
    }
    let before = motors
        .iter()
        .map(|m| fresh(m, &bus))
        .collect::<Result<Vec<_>>>()?;
    let original_mode = mode(&motors[6])?;
    println!(
        "BEFORE positions={:?} statuses={:?} gripper_mode={original_mode}",
        before.iter().map(|s| s.pos).collect::<Vec<_>>(),
        before.iter().map(|s| s.status_code).collect::<Vec<_>>()
    );
    if !movement {
        controller.close_bus()?;
        return Ok(());
    }
    if before.iter().any(|s| s.status_code != 0) {
        return Err("all seven motors must initially be disabled".into());
    }
    if !(1..=4).contains(&original_mode) {
        return Err("unexpected original gripper mode".into());
    }
    let start = before[6].pos;
    let target = start - 0.02;
    if !(-4.5..=0.0).contains(&start) || !(-4.5..=0.0).contains(&target) {
        return Err("gripper position/target outside reference motor range [-4.5, 0]".into());
    }
    let mut session = Session {
        gripper: motors[6].clone(),
        bus: bus.clone(),
        original_mode,
        switched: false,
        owned: false,
    };
    if original_mode != 4 {
        session.switched = true;
        session.gripper.write_register_u32(10, 4)?;
        std::thread::sleep(Duration::from_millis(20));
    }
    if mode(&session.gripper)? != 4 || fresh(&session.gripper, &bus)?.status_code != 0 {
        return Err("mode4/disabled verification failed".into());
    }
    session.gripper.send_cmd_force_pos(start, 0.05, 0.01)?;
    session.owned = true; // Attempt disable even if the enable write is ambiguous.
    session.gripper.enable()?;
    wait_status(&session.gripper, &bus, 1)?;
    let begin = Instant::now();
    let mut last = before[6];
    while begin.elapsed() < Duration::from_millis(2300) {
        let alpha = (begin.elapsed().as_secs_f32() / 2.0).min(1.0);
        session
            .gripper
            .send_cmd_force_pos(start + (target - start) * alpha, 0.05, 0.01)?;
        last = fresh(&session.gripper, &bus)?;
        if last.status_code != 1 || (last.pos - start).abs() > 0.05 || last.torq.abs() > 0.25 {
            return Err("gripper status/travel/torque guard triggered".into());
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    session.finish()?;
    let after = motors
        .iter()
        .map(|m| fresh(m, &bus))
        .collect::<Result<Vec<_>>>()?;
    if after.iter().any(|s| s.status_code != 0) {
        return Err("not all motors disabled after test".into());
    }
    for i in 0..6 {
        if (after[i].pos - before[i].pos).abs() > 0.01 {
            return Err("arm position changed during gripper-only test".into());
        }
    }
    println!("RESULT start_rad={start:.6} target_rad={target:.6} measured_rad={:.6} measured_delta_rad={:.6} final_mode={} final_status={} reached={}",
        last.pos, last.pos - start, mode(&session.gripper)?, after[6].status_code, (last.pos - target).abs() < 0.005);
    controller.close_bus()?;
    if (last.pos - target).abs() >= 0.005 {
        return Err(
            "target not reached at 1% current; test stopped without increasing current".into(),
        );
    }
    Ok(())
}
fn main() {
    // POSIX signal handlers only set a lock-free flag; normal unwinding disables
    // motor7 and restores its mode. SIGKILL/power loss cannot run cleanup.
    unsafe {
        signal(2, cancel);
        signal(15, cancel);
    }
    if let Err(error) = run() {
        eprintln!("gripper_probe: {error}");
        std::process::exit(1);
    }
}
