//! Private ROS hardware adapter; CAN framing/transport and Damiao protocol belong upstream.
//! No CoreController/background worker: bounded manual RX preserves every fault in a batch.
use motor_core::bus::{CanBus, CanFrame};
use motor_core::dm_serial::DmSerialBus;
use motor_vendor_damiao::protocol as dm;
use std::cell::RefCell;
use std::ffi::{c_char, CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::{Duration, Instant};

type Result<T> = std::result::Result<T, String>;
thread_local! { static ERROR: RefCell<CString> = RefCell::new(CString::new("").unwrap()); }
fn error(message: impl ToString) {
    ERROR.with(|e| *e.borrow_mut() = CString::new(message.to_string().replace('\0', "?")).unwrap());
}
fn ffi(f: impl FnOnce() -> Result<()>) -> i32 {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => 0,
        Ok(Err(e)) => {
            error(e);
            -1
        }
        Err(_) => {
            error("motorbridge backend panic");
            -1
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct State {
    position: f64,
    velocity: f64,
    torque: f64,
    age_ns: u64,
    mode_age_ns: u64,
    sequence: u64,
    mode: i32,
    status: u8,
    fault_status: u8,
    mode_changed: u8,
}
struct Motor {
    id: u16,
    feedback_id: u16,
    limits: dm::Limits,
    expected_mode: i32,
    state: State,
    received: Option<Instant>,
    mode_received: Option<Instant>,
}
pub struct HeiMotorbridge {
    // Dropping the only transport closes the port without sending disable_all.
    bus: Option<DmSerialBus>,
    motors: Vec<Motor>,
}
impl HeiMotorbridge {
    fn send(&mut self, id: u32, data: [u8; 8]) -> Result<()> {
        let result = self
            .bus
            .as_ref()
            .ok_or("transport is closed")?
            .send(CanFrame {
                arbitration_id: id,
                data,
                dlc: 8,
                is_extended: false,
                is_rx: false,
            })
            .map_err(|e| e.to_string());
        // A failed write may have accepted a partial frame. Never append another
        // frame (including disable) to that ambiguous byte stream.
        if result.is_err() {
            self.bus = None;
        }
        result
    }
    fn poll(&mut self) -> Result<()> {
        // ~4 KiB maximum valid CAN feedback per update; no unbounded drain loop.
        let mut empty = 0;
        for _ in 0..256 {
            let frame = match self
                .bus
                .as_ref()
                .ok_or("transport is closed")?
                .recv(Duration::ZERO)
                .map_err(|e| e.to_string())
            {
                Ok(Some(frame)) => {
                    empty = 0;
                    frame
                }
                // Upstream recv(0) can buffer bytes and return None before
                // parsing them. Give it one more pass without sleeping.
                Ok(None) => {
                    empty += 1;
                    if empty >= 2 {
                        return Ok(());
                    }
                    continue;
                }
                Err(e) => {
                    self.bus = None;
                    return Err(e);
                }
            };
            if !frame.is_rx || frame.is_extended || frame.dlc != 8 || frame.arbitration_id > 0x7ff {
                continue;
            }
            for motor in &mut self.motors {
                if frame.arbitration_id != u32::from(motor.feedback_id) {
                    continue;
                }
                let d = frame.data;
                if dm::is_register_reply(&d) || dm::is_register_write_ack(&d) {
                    // Register replies must not count as state feedback.
                    if u16::from_le_bytes([d[0], d[1]]) == motor.id {
                        if let Ok((10, bytes)) = dm::decode_register_value(d) {
                            let mode = u32::from_le_bytes(bytes) as i32;
                            motor.state.mode = mode;
                            motor.mode_received = Some(Instant::now());
                            if motor.expected_mode != 0 && mode != motor.expected_mode {
                                motor.state.mode_changed = 1;
                            }
                        }
                    }
                    break;
                }
                if d[0] & 15 != motor.id as u8 {
                    break;
                }
                let state = dm::decode_sensor_feedback(d, motor.limits);
                motor.state.position = f64::from(state.pos);
                motor.state.velocity = f64::from(state.vel);
                motor.state.torque = f64::from(state.torq);
                motor.state.status = state.status_code;
                if state.status_code > 1 {
                    motor.state.fault_status = state.status_code;
                }
                motor.state.sequence = motor.state.sequence.saturating_add(1);
                motor.received = Some(Instant::now());
                break;
            }
        }
        Ok(())
    }
}
unsafe fn session<'a>(ptr: *mut HeiMotorbridge) -> Result<&'a mut HeiMotorbridge> {
    ptr.as_mut().ok_or("null motorbridge session".into())
}
#[no_mangle]
pub extern "C" fn hei_mb_error() -> *const c_char {
    ERROR.with(|e| e.borrow().as_ptr())
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_open(path: *const c_char, baud: u32) -> *mut HeiMotorbridge {
    let mut ptr = std::ptr::null_mut();
    ffi(|| {
        if path.is_null() {
            return Err("null serial path".into());
        }
        let path = CStr::from_ptr(path).to_str().map_err(|e| e.to_string())?;
        let bus = DmSerialBus::open(path, baud).map_err(|e| e.to_string())?;
        ptr = Box::into_raw(Box::new(HeiMotorbridge {
            bus: Some(bus),
            motors: Vec::new(),
        }));
        Ok(())
    });
    ptr
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_close(ptr: *mut HeiMotorbridge) {
    if !ptr.is_null() {
        let _ = ffi(|| {
            drop(Box::from_raw(ptr));
            Ok(())
        });
    }
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_add(
    ptr: *mut HeiMotorbridge,
    id: u16,
    feedback_id: u16,
    pmax: f64,
    vmax: f64,
    tmax: f64,
    expected_mode: i32,
) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        if !(1..=15).contains(&id)
            || !(1..=0x7fe).contains(&feedback_id)
            || ![0, 2, 3, 4].contains(&expected_mode)
            || (expected_mode == 4 && (id != 7 || feedback_id != 0x17))
            || [pmax, vmax, tmax]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.0 || *v > f32::MAX as f64)
            || s.motors
                .iter()
                .any(|m| m.id == id || m.feedback_id == feedback_id)
        {
            return Err("invalid or duplicate motor parameters".into());
        }
        s.motors.push(Motor {
            id,
            feedback_id,
            expected_mode,
            limits: dm::Limits {
                p_min: -pmax as f32,
                p_max: pmax as f32,
                v_min: -vmax as f32,
                v_max: vmax as f32,
                t_min: -tmax as f32,
                t_max: tmax as f32,
            },
            state: State {
                mode: -1,
                ..State::default()
            },
            received: None,
            mode_received: None,
        });
        Ok(())
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_poll(ptr: *mut HeiMotorbridge) -> i32 {
    ffi(|| session(ptr)?.poll())
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_refresh(ptr: *mut HeiMotorbridge) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        for index in 0..s.motors.len() {
            let m = &s.motors[index];
            let (id, mode) = (m.id, m.expected_mode);
            s.send(0x7ff, dm::encode_feedback_request_cmd(id))?;
            if mode != 0 {
                s.send(0x7ff, dm::encode_register_read_cmd(id, 10))?;
            }
        }
        Ok(())
    })
}
fn age(time: Option<Instant>) -> u64 {
    time.map(|t| t.elapsed().as_nanos().min(u64::MAX as u128) as u64)
        .unwrap_or(u64::MAX)
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_state(
    ptr: *mut HeiMotorbridge,
    index: u32,
    out: *mut State,
) -> i32 {
    ffi(|| {
        let m = session(ptr)?
            .motors
            .get(index as usize)
            .ok_or("invalid motor index")?;
        let out = out.as_mut().ok_or("null state output")?;
        *out = State {
            age_ns: age(m.received),
            mode_age_ns: age(m.mode_received),
            ..m.state
        };
        Ok(())
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_enable(ptr: *mut HeiMotorbridge, index: u32, enable: u8) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let m = s.motors.get(index as usize).ok_or("invalid motor index")?;
        if m.expected_mode == 0 {
            return Err("read-only motor".into());
        }
        if enable != 0 && m.expected_mode == 4 {
            force_ready(m)?;
            if m.state.status != 0 {
                return Err("gripper must be disabled before enable".into());
            }
        }
        let id = m.id;
        s.send(
            u32::from(id),
            if enable == 0 {
                dm::encode_disable_cmd()
            } else {
                dm::encode_enable_cmd()
            },
        )
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_command(
    ptr: *mut HeiMotorbridge,
    index: u32,
    value: f64,
    speed: f64,
    position: u8,
) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let m = s.motors.get(index as usize).ok_or("invalid motor index")?;
        let is_position = position != 0;
        if m.expected_mode != (if is_position { 2 } else { 3 })
            || !value.is_finite()
            || !speed.is_finite()
            || speed <= 0.0
            || speed > m.limits.v_max as f64
            || value.abs()
                > (if is_position {
                    m.limits.p_max
                } else {
                    m.limits.v_max
                }) as f64
        {
            return Err("invalid motor command".into());
        }
        let id = m.id;
        if is_position {
            s.send(
                0x100 + u32::from(id),
                dm::encode_pos_vel_cmd(value as f32, speed as f32),
            )
        } else {
            s.send(0x200 + u32::from(id), dm::encode_vel_cmd(value as f32))
        }
    })
}

// Explicit motor7-only operations for the independent gripper action server.
fn force_ready(m: &Motor) -> Result<()> {
    if m.id != 7
        || m.expected_mode != 4
        || m.state.mode != 4
        || m.state.mode_changed != 0
        || m.state.fault_status != 0
        || age(m.received) > 250_000_000
        || age(m.mode_received) > 250_000_000
    {
        return Err("gripper mode/state is stale, faulted or unconfigured".into());
    }
    Ok(())
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_read_mode(ptr: *mut HeiMotorbridge, index: u32) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let id = s
            .motors
            .get(index as usize)
            .ok_or("invalid motor index")?
            .id;
        s.send(0x7ff, dm::encode_register_read_cmd(id, 10))
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_configure_gripper(ptr: *mut HeiMotorbridge, index: u32) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let m = s
            .motors
            .get_mut(index as usize)
            .ok_or("invalid motor index")?;
        if m.id != 7
            || m.expected_mode != 4
            || m.state.status != 0
            || m.state.fault_status != 0
            || age(m.received) > 250_000_000
        {
            return Err("mode configuration requires fresh disabled gripper7 feedback".into());
        }
        // Once fresh readback confirms mode4, clear the startup mismatch latch.
        if m.state.mode == 4 && age(m.mode_received) <= 250_000_000 {
            m.state.mode_changed = 0;
            return Ok(());
        }
        m.state.mode = -1;
        m.state.mode_changed = 0;
        m.mode_received = None;
        s.send(
            0x7ff,
            dm::encode_register_write_cmd(7, 10, 4u32.to_le_bytes()),
        )
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_configure_arm_mode(ptr: *mut HeiMotorbridge, index: u32) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let m = s
            .motors
            .get_mut(index as usize)
            .ok_or("invalid motor index")?;
        if ![2, 3].contains(&m.expected_mode)
            || ![1, 2, 3].contains(&m.state.mode)
            || m.state.status != 0
            || m.state.fault_status != 0
            || age(m.received) > 250_000_000
            || age(m.mode_received) > 250_000_000
        {
            return Err("arm mode configuration requires fresh disabled, fault-free state and mode feedback".into());
        }
        if m.state.mode == m.expected_mode {
            m.state.mode_changed = 0;
            return Ok(());
        }
        let id = m.id;
        let mode = m.expected_mode as u32;
        // Configure RAM only. No enable, zeroing, fault-clear or EEPROM-save command.
        m.state.mode = -1;
        m.state.mode_changed = 0;
        m.mode_received = None;
        s.send(
            0x7ff,
            dm::encode_register_write_cmd(id, 10, mode.to_le_bytes()),
        )
    })
}
#[no_mangle]
pub unsafe extern "C" fn hei_mb_force_position(
    ptr: *mut HeiMotorbridge,
    index: u32,
    position: f64,
    speed: f64,
    current_ratio: f64,
) -> i32 {
    ffi(|| {
        let s = session(ptr)?;
        let m = s.motors.get(index as usize).ok_or("invalid motor index")?;
        force_ready(m)?;
        if !position.is_finite()
            || !(-4.5..=0.0).contains(&position)
            || !speed.is_finite()
            || !(0.01..=10.0).contains(&speed)
            || !current_ratio.is_finite()
            || !(0.0001..=0.05).contains(&current_ratio)
        {
            return Err("gripper target/speed/current exceeds reference limits".into());
        }
        s.send(
            0x307,
            dm::encode_force_pos_cmd(position as f32, speed as f32, current_ratio as f32),
        )
    })
}
