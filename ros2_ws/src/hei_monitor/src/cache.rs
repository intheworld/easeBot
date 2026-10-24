use bytes::Bytes;
use diagnostic_msgs::msg::DiagnosticArray;
use nav_msgs::msg::Odometry;
use sensor_msgs::msg::{CompressedImage, JointState};
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

pub const NAMES: [&str; 3] = ["front", "left_wrist", "right_wrist"];
type Stamp = (i32, u32);
struct Frame {
    data: Bytes,
    received: Instant,
    stamp: Stamp,
}
struct CameraDiagnostic {
    values: BTreeMap<String, String>,
    message: String,
    received: Instant,
}
pub struct Cache {
    pub hardware: Option<crate::hardware::Hardware>,
    positions: BTreeMap<String, f64>,
    last_joint: Option<Instant>,
    joint_stamp: Option<Stamp>,
    joint_valid: bool,
    sample: u64,
    manager: BTreeMap<String, String>,
    mode: String,
    last_diag: Option<Instant>,
    frames: BTreeMap<String, Frame>,
    camera_diagnostics: BTreeMap<String, CameraDiagnostic>,
    base: Option<([f64; 3], Instant)>,
    events: Vec<Value>,
    last_event: Option<(String, String)>,
    started: Instant,
}
fn fresh(at: Option<Instant>, seconds: u64) -> bool {
    at.is_some_and(|at| at.elapsed() < Duration::from_secs(seconds))
}
fn age(at: Option<Instant>) -> Option<f64> {
    at.map(|at| at.elapsed().as_secs_f64() * 1000.0)
}
impl Default for Cache {
    fn default() -> Self {
        Self {
            hardware: None,
            positions: BTreeMap::new(),
            last_joint: None,
            joint_stamp: None,
            joint_valid: false,
            sample: 0,
            manager: BTreeMap::new(),
            mode: "NOT_READY".into(),
            last_diag: None,
            frames: BTreeMap::new(),
            camera_diagnostics: BTreeMap::new(),
            base: None,
            events: vec![],
            last_event: None,
            started: Instant::now(),
        }
    }
}
impl Cache {
    pub fn joints(&mut self, msg: JointState) {
        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        if self.joint_stamp == Some(stamp) {
            return;
        }
        self.joint_stamp = Some(stamp);
        self.joint_valid = !msg.name.is_empty()
            && msg.name.len() == msg.position.len()
            && msg.position.iter().all(|x| x.is_finite());
        if self.joint_valid {
            self.positions = msg.name.into_iter().zip(msg.position).collect();
            self.last_joint = Some(Instant::now());
            self.sample += 1;
        }
    }
    pub fn diagnostics(&mut self, msg: DiagnosticArray) {
        for status in msg.status {
            let values: BTreeMap<_, _> = status
                .values
                .into_iter()
                .map(|v| (v.key, v.value))
                .collect();
            if status.name == "hei/robot_manager" {
                let key = (
                    status.message.clone(),
                    values.get("fault").cloned().unwrap_or_default(),
                );
                if self.last_event.as_ref() != Some(&key) {
                    let event = if key.1.is_empty() {
                        key.0.clone()
                    } else {
                        format!("{} / {}", key.0, key.1)
                    };
                    self.events.push(json!({"time": SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_secs_f64(), "event": event}));
                    if self.events.len() > 100 {
                        self.events.remove(0);
                    }
                    self.last_event = Some(key);
                }
                self.manager = values;
                self.mode = status.message;
                self.last_diag = Some(Instant::now());
            } else if let Some(name) = status.name.strip_prefix("hei/camera/") {
                self.camera_diagnostics.insert(
                    name.into(),
                    CameraDiagnostic {
                        values,
                        message: status.message,
                        received: Instant::now(),
                    },
                );
            }
        }
    }
    pub fn image(&mut self, name: &str, msg: CompressedImage) {
        if !msg.format.to_lowercase().contains("jpeg") || msg.data.len() > 2 * 1024 * 1024 {
            return;
        }
        let stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec);
        if self.frames.get(name).is_some_and(|f| f.stamp == stamp) {
            return;
        }
        self.frames.insert(
            name.into(),
            Frame {
                data: Bytes::from(msg.data),
                received: Instant::now(),
                stamp,
            },
        );
    }
    pub fn odometry(&mut self, msg: Odometry) {
        let v = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ];
        if v.iter().all(|x| x.is_finite()) {
            self.base = Some((v, Instant::now()));
        }
    }
    pub fn frame(&self, name: &str) -> Option<Bytes> {
        self.frames
            .get(name)
            .filter(|f| fresh(Some(f.received), 1))
            .map(|f| f.data.clone())
    }
    pub fn snapshot(&mut self) -> Value {
        let live = fresh(self.last_diag, 2);
        let controllers: BTreeMap<String, String> = self
            .manager
            .get("controllers")
            .and_then(|s| serde_json::from_str(s).ok())
            .unwrap_or_default();
        let cameras: BTreeMap<_,_> = NAMES.into_iter().map(|name| {
            let frame = self.frames.get(name); let diag = self.camera_diagnostics.get(name);
            (name, json!({"backend":diag.and_then(|d| d.values.get("backend")).map(String::as_str).unwrap_or("unknown"),
                "age_ms":age(frame.map(|f| f.received)), "valid":frame.is_some_and(|f| fresh(Some(f.received),1)),
                "error":diag.filter(|d| fresh(Some(d.received),2)).map(|d| d.message.as_str()).unwrap_or("diagnostics unavailable")}))
        }).collect();
        if let Some(hardware) = &mut self.hardware {
            let mut data = hardware.snapshot();
            data["base"] = json!(self
                .base
                .filter(|(_, at)| fresh(Some(*at), 1))
                .map(|(v, _)| v));
            data["uptime_s"] = json!(self.started.elapsed().as_secs_f64());
            return json!({"system":{"version":"0.2.0 / ROS 2 / Rust", "monitor_read_only":true},
                "robot":{"available":true,"data":data},"sensors":{"available":true,"data":{"cameras":cameras}}});
        }
        json!({"system":{"version":"0.2.0 / ROS 2 / Rust", "monitor_read_only":true},
            "robot":{"available":!self.positions.is_empty(),"data":{
                "robot_id":"HEI ReBot Lift", "backend":self.manager.get("backend").map(String::as_str).unwrap_or("unknown"),
                "hardware_connected":false, "mode":if live { &self.mode } else { "UNKNOWN" }, "enabled":live && self.mode=="ENABLED", "homed":null,
                "controller_active":live && controllers.iter().any(|(n,s)| n!="joint_state_broadcaster" && s=="active"),
                "sequence":self.sample,"sample_id":self.sample,"state":self.positions,"state_age_ms":age(self.last_joint),
                "valid":self.joint_valid && fresh(self.last_joint,1) && live,"controllers":controllers,
                "base":self.base.filter(|(_,at)| fresh(Some(*at),1)).map(|(v,_)| v),"uptime_s":self.started.elapsed().as_secs_f64(),
                "fault":if live { self.manager.get("fault").map(String::as_str).unwrap_or("") } else { "robot_manager diagnostics unavailable" },"events":self.events}},
            "sensors":{"available":true,"data":{"cameras":cameras}}})
    }
}
// Read operating-system counters outside the shared ROS cache lock.
pub fn system_metrics(status: &mut Value) {
    let mut memory = BTreeMap::new();
    if let Ok(data) = std::fs::read_to_string("/proc/meminfo") {
        for line in data.lines() {
            if let Some((key, value)) = line.split_once(':') {
                if key == "MemTotal" || key == "MemAvailable" {
                    if let Some(kb) = value
                        .split_whitespace()
                        .next()
                        .and_then(|v| v.parse::<u64>().ok())
                    {
                        memory.insert(key.to_owned(), kb * 1024);
                    }
                }
            }
        }
    }
    status["system"]["memory"] = json!(memory);
    status["system"]["runtime_disk"] =
        json!({"free":fs2::available_space("/tmp").ok(),"total":fs2::total_space("/tmp").ok()});
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn duplicate_data_does_not_refresh_age() {
        let mut cache = Cache::default();
        let mut image = CompressedImage {
            format: "jpeg".into(),
            data: vec![255, 216],
            ..Default::default()
        };
        image.header.stamp.sec = 1;
        cache.image("front", image.clone());
        cache.frames.get_mut("front").unwrap().received = Instant::now() - Duration::from_secs(3);
        cache.image("front", image);
        assert!(cache.frame("front").is_none());
        let mut joints = JointState {
            name: vec!["lift_joint".into()],
            position: vec![-0.2],
            ..Default::default()
        };
        joints.header.stamp.sec = 1;
        cache.joints(joints.clone());
        cache.last_joint = Some(Instant::now() - Duration::from_secs(3));
        cache.joints(joints);
        assert!(!fresh(cache.last_joint, 1));
        assert_eq!(cache.sample, 1);
    }
    #[test]
    fn invalid_and_missing_feedback_stays_unknown() {
        let mut cache = Cache::default();
        cache.joints(JointState {
            name: vec!["lift_joint".into()],
            position: vec![f64::NAN],
            ..Default::default()
        });
        let result = cache.snapshot();
        assert_eq!(result["robot"]["data"]["valid"], false);
        assert_eq!(result["robot"]["data"]["mode"], "UNKNOWN");
        assert!(result["robot"]["data"]["homed"].is_null());
        assert!(NAMES
            .iter()
            .all(|n| result["sensors"]["data"]["cameras"][n]["valid"] == false));
    }
    #[test]
    fn manager_expiry_and_malformed_controller_data_are_safe() {
        let mut cache = Cache::default();
        cache.mode = "ENABLED".into();
        cache.last_diag = Some(Instant::now());
        cache.manager.insert("controllers".into(), "[]".into());
        assert_eq!(cache.snapshot()["robot"]["data"]["controllers"], json!({}));
        cache.last_diag = Some(Instant::now() - Duration::from_secs(3));
        assert_eq!(cache.snapshot()["robot"]["data"]["enabled"], false);
    }
}
