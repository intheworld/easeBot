//! Independent freshness for each physical bus. No command interfaces are used.
use control_msgs::msg::DynamicJointState;
use controller_manager_msgs::srv::{ListControllers_Response, ListHardwareComponents_Response};
use sensor_msgs::msg::JointState;
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const BUSES: [&str; 4] = ["left", "right", "base", "lift"];
fn fresh(at: Option<Instant>, seconds: u64) -> bool {
    at.is_some_and(|at| at.elapsed() < Duration::from_secs(seconds))
}
fn age(at: Option<Instant>) -> Option<f64> {
    at.map(|at| at.elapsed().as_secs_f64() * 1000.)
}
#[derive(Default)]
struct Bus {
    stamp: Option<(i32, u32)>,
    joints_at: Option<Instant>,
    valid: bool,
    positions: BTreeMap<String, f64>,
    controllers_at: Option<Instant>,
    controllers: BTreeMap<String, String>,
    hardware_at: Option<Instant>,
    hardware: BTreeMap<String, (u8, String, String)>,
    dynamic_stamp: Option<(i32, u32)>,
    dynamic_at: Option<Instant>,
    dynamic: BTreeMap<String, f64>,
}
pub struct Hardware {
    buses: BTreeMap<&'static str, Bus>,
    pub read_only: bool,
    pub gaps: BTreeMap<&'static str, (f64, f64)>,
    sequence: u64,
    events: Vec<Value>,
    last_event: String,
}
impl Hardware {
    pub fn new(read_only: bool) -> Self {
        Self {
            buses: BUSES
                .into_iter()
                .map(|name| (name, Bus::default()))
                .collect(),
            read_only,
            gaps: ["left", "right"]
                .into_iter()
                .map(|name| (name, (0., -0.1 / 4.5)))
                .collect(),
            sequence: 0,
            events: vec![],
            last_event: String::new(),
        }
    }
    pub fn joints(&mut self, source: &str, msg: JointState) {
        let Some(bus) = self.buses.get_mut(source) else {
            return;
        };
        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        if bus.stamp == Some(stamp) {
            return;
        }
        bus.stamp = Some(stamp);
        let expected: Vec<String> = match source {
            "left" | "right" => {
                let prefix = if source == "left" {
                    "b_left"
                } else {
                    "a_right"
                };
                (1..=6)
                    .map(|i| format!("{prefix}_joint{i}"))
                    .chain([format!("{source}_gripper_motor")])
                    .collect()
            }
            "base" => ["front_left", "rear_left", "rear_right", "front_right"]
                .into_iter()
                .map(|n| format!("{n}_wheel_joint"))
                .collect(),
            _ => vec!["lift_joint".into()],
        };
        bus.valid = msg.name.len() == msg.position.len()
            && msg.position.iter().all(|p| p.is_finite())
            && msg.name.iter().collect::<BTreeSet<_>>().len() == msg.name.len()
            && expected.iter().all(|n| msg.name.contains(n));
        if bus.valid {
            bus.positions = msg.name.into_iter().zip(msg.position).collect();
            bus.joints_at = Some(Instant::now());
            self.sequence += 1;
        }
    }
    pub fn dynamic(&mut self, source: &str, msg: DynamicJointState) {
        // JointState broadcaster filters resources absent from the visual URDF
        // (notably raw gripper motors). DynamicJointState retains all hardware
        // resources and is the authoritative source for the real monitor.
        let mut positions = JointState {
            header: msg.header.clone(),
            ..Default::default()
        };
        if msg.joint_names.len() == msg.interface_values.len() {
            for (name, interfaces) in msg.joint_names.iter().zip(&msg.interface_values) {
                if let Some(index) = interfaces
                    .interface_names
                    .iter()
                    .position(|n| n == "position")
                {
                    if let Some(value) = interfaces.values.get(index) {
                        positions.name.push(name.clone());
                        positions.position.push(*value);
                    }
                }
            }
        }
        self.joints(source, positions);
        let Some(bus) = self.buses.get_mut(source) else {
            return;
        };
        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        if bus.dynamic_stamp == Some(stamp) {
            return;
        }
        bus.dynamic_stamp = Some(stamp);
        let resource = if source == "lift" {
            "lift_status".into()
        } else {
            format!("{source}_gripper_motor")
        };
        if msg.joint_names.len() != msg.interface_values.len() {
            return;
        }
        for (name, interfaces) in msg.joint_names.into_iter().zip(msg.interface_values) {
            if name == resource
                && interfaces.interface_names.len() == interfaces.values.len()
                && interfaces.values.iter().all(|v| v.is_finite())
            {
                let values: BTreeMap<_, _> = interfaces
                    .interface_names
                    .into_iter()
                    .zip(interfaces.values)
                    .collect();
                let required: &[&str] = if source == "lift" {
                    &["homed", "phase", "io_online", "upper", "lower"]
                } else {
                    &["enabled", "position", "effort"]
                };
                if required.iter().all(|key| values.contains_key(*key)) {
                    bus.dynamic = values;
                    bus.dynamic_at = Some(Instant::now());
                }
            }
        }
    }
    pub fn controllers(&mut self, source: &str, response: ListControllers_Response) {
        if let Some(bus) = self.buses.get_mut(source) {
            bus.controllers = response
                .controller
                .into_iter()
                .map(|c| (c.name, c.state))
                .collect();
            bus.controllers_at = Some(Instant::now());
        }
    }
    pub fn hardware(&mut self, source: &str, response: ListHardwareComponents_Response) {
        if let Some(bus) = self.buses.get_mut(source) {
            bus.hardware = response
                .component
                .into_iter()
                .map(|c| (c.name, (c.state.id, c.state.label, c.plugin_name)))
                .collect();
            bus.hardware_at = Some(Instant::now());
        }
    }
    pub fn snapshot(&mut self) -> Value {
        let mut state = BTreeMap::<String, Value>::new();
        let mut controllers = BTreeMap::<String, String>::new();
        let mut buses = BTreeMap::new();
        let mut grippers = BTreeMap::new();
        let mut faults = vec![];
        let mut connected = true;
        let mut valid = true;
        let mut ready = true;
        let mut any_active = false;
        let mut explicit_fault = false;
        let mut oldest: Option<f64> = Some(0.);
        for (name, bus) in &self.buses {
            let feedback = bus.valid && fresh(bus.joints_at, 1);
            let manager = fresh(bus.controllers_at, 3);
            let hardware_fresh = fresh(bus.hardware_at, 3);
            let expected_plugin = if *name == "lift" {
                "hei_hardware/LiftSystem"
            } else {
                "hei_hardware/DamiaoSystem"
            };
            let active_hardware = hardware_fresh
                && bus.hardware.len() == 1
                && bus
                    .hardware
                    .values()
                    .all(|(id, _, plugin)| *id == 3 && plugin == expected_plugin);
            let bus_connected = feedback && active_hardware;
            connected &= bus_connected;
            valid &= bus_connected && manager;
            oldest = oldest.zip(age(bus.joints_at)).map(|(a, b)| a.max(b));
            if !feedback {
                faults.push(format!("{name}: feedback missing, invalid or stale"));
            }
            if !manager {
                faults.push(format!("{name}: controller query unavailable or stale"));
            }
            if !active_hardware {
                faults.push(format!("{name}: hardware unavailable, inactive or stale"));
            }
            explicit_fault |= hardware_fresh && bus.hardware.values().any(|(id, _, _)| *id == 15);
            let motion: Vec<String> = if *name == "left" || *name == "right" {
                vec![
                    format!("{name}_arm_controller"),
                    format!("{name}_gripper_controller"),
                ]
            } else {
                vec![format!("{name}_controller")]
            };
            ready &= manager
                && motion
                    .iter()
                    .all(|c| bus.controllers.get(c).is_some_and(|s| s == "active"));
            any_active |= manager
                && motion
                    .iter()
                    .any(|c| bus.controllers.get(c).is_some_and(|s| s == "active"));
            for (joint, value) in &bus.positions {
                state.insert(
                    joint.clone(),
                    if bus_connected {
                        json!(value)
                    } else {
                        Value::Null
                    },
                );
            }
            let current_controllers: BTreeMap<_, _> = bus
                .controllers
                .iter()
                .map(|(n, s)| {
                    (
                        n.clone(),
                        if manager { s.clone() } else { "unknown".into() },
                    )
                })
                .collect();
            controllers.extend(current_controllers.clone());
            buses.insert(*name, json!({"connected":bus_connected,"valid":bus_connected && manager,"state_age_ms":age(bus.joints_at),
                "controllers_age_ms":age(bus.controllers_at),"hardware_age_ms":age(bus.hardware_at),"controllers":current_controllers,
                "hardware":bus.hardware.iter().map(|(n,(id,label,plugin))| (n,json!({"id":if hardware_fresh {Some(*id)} else {None},"state":if hardware_fresh {label.as_str()} else {"unknown"},"plugin":plugin}))).collect::<BTreeMap<_,_>>(),
                "motor_enabled":null}));
            if *name == "left" || *name == "right" {
                let dynamic = bus_connected && fresh(bus.dynamic_at, 1);
                let angle = bus
                    .positions
                    .get(&format!("{name}_gripper_motor"))
                    .copied()
                    .filter(|_| bus_connected);
                let (zero, slope) = self.gaps[name];
                grippers.insert(*name, json!({"position_rad":angle,"gap_m":angle.map(|p| zero + slope*p),
                    "enabled":if dynamic { bus.dynamic.get("enabled").map(|v| *v == 1.) } else {None},
                    "torque_nm":if dynamic {bus.dynamic.get("effort")} else {None},"force_n":null}));
            }
        }
        let lift = &self.buses["lift"];
        let lift_live = fresh(lift.dynamic_at, 1);
        let io_live = lift_live
            && lift.dynamic.get("io_online") == Some(&1.)
            && buses["lift"]["connected"].as_bool() == Some(true);
        let homed = if io_live && lift.dynamic.get("phase") != Some(&3.) {
            lift.dynamic.get("homed").map(|v| *v == 1.)
        } else {
            None
        };
        if !io_live {
            faults.push("lift: limit IO status missing, offline or stale".into());
        }
        let lift_fault = lift_live && lift.dynamic.get("phase") == Some(&3.);
        explicit_fault |= lift_fault;
        if lift_fault {
            faults.push("lift: hardware reports fault".into());
        }
        valid &= io_live && !lift_fault;
        connected &= io_live && !lift_fault;
        if let Some(bus) = buses.get_mut("lift") {
            bus["valid"] = json!(bus["valid"].as_bool() == Some(true) && io_live && !lift_fault);
            bus["connected"] =
                json!(bus["connected"].as_bool() == Some(true) && io_live && !lift_fault);
        }
        ready &= valid && homed == Some(true);
        let mode = if explicit_fault {
            "FAULT"
        } else if !valid {
            "NOT_READY"
        } else if self.read_only {
            "READ_ONLY"
        } else if lift.dynamic.get("phase") == Some(&1.) {
            "HOMING"
        } else if ready {
            "READY"
        } else if any_active {
            "PARTIAL"
        } else {
            "DISABLED"
        };
        let fault = faults.join("; ");
        let event = format!("{mode} / {fault} / {controllers:?}");
        if event != self.last_event {
            self.events.push(json!({"time":SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_secs_f64(),
                "event":format!("{mode}{}", if fault.is_empty() {String::new()} else {format!(" / {fault}")})}));
            if self.events.len() > 100 {
                self.events.remove(0);
            }
            self.last_event = event;
        }
        json!({"robot_id":"HEI ReBot Lift","backend":"ros2_control_damiao","hardware_connected":connected,
            "mode":mode,"enabled":null,"controller_active":any_active,"controllers_ready":ready,"read_only":self.read_only,
            "homed":homed,"lift": {"valid":io_live,"age_ms":age(lift.dynamic_at),"state": if lift_live {json!(lift.dynamic)} else {Value::Null}},
            "sequence":self.sequence,"sample_id":self.sequence,"state":state,"state_age_ms":oldest,"valid":valid,
            "controllers":controllers,"buses":buses,"grippers":grippers,"fault":fault,"events":self.events})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use controller_manager_msgs::msg::{ControllerState, HardwareComponentState};
    fn populated() -> Hardware {
        let mut h = Hardware::new(false);
        for source in BUSES {
            let names: Vec<String> = match source {
                "left" | "right" => (1..=6)
                    .map(|i| {
                        format!(
                            "{}_joint{i}",
                            if source == "left" {
                                "b_left"
                            } else {
                                "a_right"
                            }
                        )
                    })
                    .chain([format!("{source}_gripper_motor")])
                    .collect(),
                "base" => ["front_left", "rear_left", "rear_right", "front_right"]
                    .into_iter()
                    .map(|n| format!("{n}_wheel_joint"))
                    .collect(),
                _ => vec!["lift_joint".into()],
            };
            let mut msg = JointState {
                position: vec![-0.1; names.len()],
                name: names,
                ..Default::default()
            };
            msg.header.stamp.sec = 1;
            h.joints(source, msg);
            let mut hw = HardwareComponentState::default();
            hw.name = source.into();
            hw.state.id = 3;
            hw.state.label = "active".into();
            hw.plugin_name = if source == "lift" {
                "hei_hardware/LiftSystem"
            } else {
                "hei_hardware/DamiaoSystem"
            }
            .into();
            let mut response = ListHardwareComponents_Response::default();
            response.component = vec![hw];
            h.hardware(source, response);
            let names = if source == "left" || source == "right" {
                vec![
                    format!("{source}_arm_controller"),
                    format!("{source}_gripper_controller"),
                ]
            } else {
                vec![format!("{source}_controller")]
            };
            let mut response = ListControllers_Response::default();
            response.controller = names
                .into_iter()
                .map(|name| ControllerState {
                    name,
                    state: "active".into(),
                    ..Default::default()
                })
                .collect();
            h.controllers(source, response);
        }
        let bus = h.buses.get_mut("lift").unwrap();
        bus.dynamic_at = Some(Instant::now());
        bus.dynamic = [
            ("homed".into(), 1.),
            ("phase".into(), 2.),
            ("io_online".into(), 1.),
        ]
        .into();
        h
    }
    #[test]
    fn all_buses_merge_without_claiming_unknown_motor_enable() {
        let mut h = populated();
        let data = h.snapshot();
        assert_eq!(data["hardware_connected"], true);
        assert_eq!(data["controllers_ready"], true);
        assert_eq!(data["homed"], true);
        assert_eq!(data["state"].as_object().unwrap().len(), 19);
        assert!(data["enabled"].is_null());
        assert!(data["grippers"]["left"]["enabled"].is_null());
        assert!(
            (data["grippers"]["right"]["gap_m"].as_f64().unwrap() - 0.1 * 0.1 / 4.5).abs() < 1e-10
        );
    }
    #[test]
    fn one_stale_bus_cannot_be_refreshed_by_other_buses_or_duplicate_stamps() {
        let mut h = populated();
        let bus = h.buses.get_mut("left").unwrap();
        bus.joints_at = Some(Instant::now() - Duration::from_secs(2));
        let msg = JointState {
            header: sensor_msgs::msg::JointState::default().header,
            ..Default::default()
        };
        // A repeated valid stamp must not refresh even a partial bus.
        let mut duplicate = msg;
        duplicate.header.stamp.sec = 1;
        h.joints("left", duplicate);
        let data = h.snapshot();
        assert_eq!(data["hardware_connected"], false);
        assert_eq!(data["buses"]["left"]["valid"], false);
        assert_eq!(data["buses"]["right"]["valid"], true);
        assert!(data["state"]["b_left_joint1"].is_null());
        assert!(data["state"]["a_right_joint1"].is_number());
    }
    #[test]
    fn stale_queries_and_gpio_do_not_leave_active_or_homed_claims() {
        let mut h = populated();
        h.buses.get_mut("right").unwrap().controllers_at =
            Some(Instant::now() - Duration::from_secs(4));
        h.buses.get_mut("lift").unwrap().dynamic_at = Some(Instant::now() - Duration::from_secs(2));
        let data = h.snapshot();
        assert_eq!(data["valid"], false);
        assert_eq!(data["controllers"]["right_arm_controller"], "unknown");
        assert!(data["homed"].is_null());
        assert!(data["lift"]["state"].is_null());
    }
    #[test]
    fn lift_fault_and_invalid_positions_are_reported_without_fake_data() {
        let mut h = populated();
        h.buses
            .get_mut("lift")
            .unwrap()
            .dynamic
            .insert("phase".into(), 3.);
        assert_eq!(h.snapshot()["mode"], "FAULT");
        let mut msg = JointState {
            name: vec!["lift_joint".into()],
            position: vec![f64::NAN],
            ..Default::default()
        };
        msg.header.stamp.sec = 2;
        h.joints("lift", msg);
        let data = h.snapshot();
        assert!(data["state"]["lift_joint"].is_null());
        assert_eq!(data["valid"], false);
    }
}
