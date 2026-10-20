use crate::{cache::NAMES, http::SharedCache};
use diagnostic_msgs::msg::DiagnosticArray;
use nav_msgs::msg::Odometry;
use rclrs::*;
use sensor_msgs::msg::{CompressedImage, JointState};
use std::sync::Arc;
// Keep the node, parameter handles, and subscriptions alive for the entire process.
pub struct Subscriptions {
    _node: Node,
    _host: ReadOnlyParameter<Arc<str>>,
    _port: ReadOnlyParameter<i64>,
    _joints: Subscription<JointState>,
    _diagnostics: Subscription<DiagnosticArray>,
    _odom: Subscription<Odometry>,
    _images: Vec<Subscription<CompressedImage>>,
}
pub fn setup(
    cache: SharedCache,
) -> Result<(Executor, Subscriptions, String, u16), Box<dyn std::error::Error + Send + Sync>> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("monitor")?;
    let host: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("host")
        .default("127.0.0.1".into())
        .read_only()?;
    let port: ReadOnlyParameter<i64> = node.declare_parameter("port").default(8081).read_only()?;
    let bind_host = host.get().to_string();
    let bind_port = u16::try_from(port.get())?;
    let c = cache.clone();
    let joints = node.create_subscription(
        "joint_states".qos(QoSProfile::sensor_data_default()),
        move |msg: JointState| c.lock().expect("cache lock poisoned").joints(msg),
    )?;
    let c = cache.clone();
    let diagnostics = node.create_subscription("diagnostics", move |msg: DiagnosticArray| {
        c.lock().expect("cache lock poisoned").diagnostics(msg)
    })?;
    let c = cache.clone();
    let odom = node.create_subscription(
        "base_controller/odom".qos(QoSProfile::sensor_data_default()),
        move |msg: Odometry| c.lock().expect("cache lock poisoned").odometry(msg),
    )?;
    let mut images = vec![];
    for name in NAMES {
        let c = cache.clone();
        images.push(node.create_subscription(
            format!("cameras/{name}/image_raw/compressed").qos(QoSProfile::sensor_data_default()),
            move |msg: CompressedImage| c.lock().expect("cache lock poisoned").image(name, msg),
        )?);
    }
    Ok((
        executor,
        Subscriptions {
            _node: node,
            _host: host,
            _port: port,
            _joints: joints,
            _diagnostics: diagnostics,
            _odom: odom,
            _images: images,
        },
        bind_host,
        bind_port,
    ))
}
