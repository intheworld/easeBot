use crate::{
    cache::NAMES,
    hardware::{Hardware, BUSES},
    http::SharedCache,
};
use control_msgs::msg::DynamicJointState;
use controller_manager_msgs::srv::{
    ListControllers, ListControllers_Request, ListControllers_Response, ListHardwareComponents,
    ListHardwareComponents_Request, ListHardwareComponents_Response,
};
use diagnostic_msgs::msg::DiagnosticArray;
use nav_msgs::msg::Odometry;
use rclrs::*;
use sensor_msgs::msg::{CompressedImage, JointState};
use std::{sync::Arc, time::Duration};

pub struct Subscriptions {
    node: Node,
    real: bool,
    _host: ReadOnlyParameter<Arc<str>>,
    _port: ReadOnlyParameter<i64>,
    _backend: ReadOnlyParameter<Arc<str>>,
    _readonly: ReadOnlyParameter<bool>,
    _gaps: Vec<ReadOnlyParameter<f64>>,
    _joints: Vec<Subscription<JointState>>,
    _dynamic: Vec<Subscription<DynamicJointState>>,
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
    let backend: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("backend")
        .default("mock".into())
        .read_only()?;
    let readonly = node
        .declare_parameter("read_only")
        .default(false)
        .read_only()?;
    let real = match backend.get().as_ref() {
        "damiao" => true,
        "mock" => false,
        _ => return Err("backend must be mock or damiao".into()),
    };
    let mut gaps = vec![];
    if real {
        let mut hardware = Hardware::new(readonly.get());
        for side in ["left", "right"] {
            let zero = node
                .declare_parameter::<f64>(format!("{side}_gap_at_zero_m"))
                .default(0.)
                .read_only()?;
            let slope = node
                .declare_parameter::<f64>(format!("{side}_gap_per_motor_rad_m"))
                .default(-0.1 / 4.5)
                .read_only()?;
            if !zero.get().is_finite() || !slope.get().is_finite() || slope.get() == 0. {
                return Err("invalid gripper gap mapping".into());
            }
            hardware.gaps.insert(side, (zero.get(), slope.get()));
            gaps.extend([zero, slope]);
        }
        cache.lock().expect("cache lock poisoned").hardware = Some(hardware);
    }
    let bind_host = host.get().to_string();
    let bind_port = u16::try_from(port.get())?;
    let mut joints = vec![];
    let mut dynamic = vec![];
    if real {
        for source in BUSES {
            let c = cache.clone();
            dynamic.push(node.create_subscription(
                format!("{source}_dynamic_joint_states").qos(QoSProfile::sensor_data_default()),
                move |msg: DynamicJointState| {
                    if let Some(h) = &mut c.lock().expect("cache lock poisoned").hardware {
                        h.dynamic(source, msg);
                    }
                },
            )?);
        }
    } else {
        let c = cache.clone();
        joints.push(node.create_subscription(
            "joint_states".qos(QoSProfile::sensor_data_default()),
            move |msg: JointState| c.lock().expect("cache lock poisoned").joints(msg),
        )?);
    }
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
            node,
            real,
            _host: host,
            _port: port,
            _backend: backend,
            _readonly: readonly,
            _gaps: gaps,
            _joints: joints,
            _dynamic: dynamic,
            _diagnostics: diagnostics,
            _odom: odom,
            _images: images,
        },
        bind_host,
        bind_port,
    ))
}
// Ok(None) means the service is not advertised yet: retain the client so
// discovery can settle without creating and destroying graph endpoints.
async fn query_controllers(
    client: &Client<ListControllers>,
) -> Result<Option<ListControllers_Response>, ()> {
    if !client.service_is_ready().map_err(|_| ())? {
        return Ok(None);
    }
    let promise: Promise<ListControllers_Response> = client
        .call(ListControllers_Request::default())
        .map_err(|_| ())?;
    let response = tokio::time::timeout(Duration::from_millis(800), promise)
        .await
        .map_err(|_| ())?
        .map_err(|_| ())?;
    Ok(Some(response))
}
async fn query_hardware(
    client: &Client<ListHardwareComponents>,
) -> Result<Option<ListHardwareComponents_Response>, ()> {
    if !client.service_is_ready().map_err(|_| ())? {
        return Ok(None);
    }
    let promise: Promise<ListHardwareComponents_Response> = client
        .call(ListHardwareComponents_Request::default())
        .map_err(|_| ())?;
    let response = tokio::time::timeout(Duration::from_millis(800), promise)
        .await
        .map_err(|_| ())?
        .map_err(|_| ())?;
    Ok(Some(response))
}
pub fn start_status_polling(
    subscriptions: &Subscriptions,
    cache: SharedCache,
) -> Result<Vec<tokio::task::JoinHandle<()>>, RclrsError> {
    let mut tasks = vec![];
    if !subscriptions.real {
        return Ok(tasks);
    }
    for source in BUSES {
        let node = subscriptions.node.clone();
        let c = cache.clone();
        let controller_service = format!("{source}_controller_manager/list_controllers");
        let hardware_service = format!("{source}_controller_manager/list_hardware_components");
        let mut controllers = node.create_client::<ListControllers>(controller_service.as_str())?;
        let mut hardware =
            node.create_client::<ListHardwareComponents>(hardware_service.as_str())?;
        tasks.push(tokio::spawn(async move {
            loop {
                let (cs, hs) =
                    tokio::join!(query_controllers(&controllers), query_hardware(&hardware));
                let controllers_failed = cs.is_err();
                let hardware_failed = hs.is_err();
                {
                    let mut cache = c.lock().expect("cache lock poisoned");
                    if let Some(h) = &mut cache.hardware {
                        if let Ok(Some(cs)) = cs {
                            h.controllers(source, cs);
                        }
                        if let Ok(Some(hs)) = hs {
                            h.hardware(source, hs);
                        }
                        // Record transitions even while no browser is polling.
                        let _ = h.snapshot();
                    }
                }
                // Dropping timed-out clients also drops their request boards, so a
                // nonresponding service cannot accumulate outstanding requests.
                if controllers_failed {
                    match node.create_client::<ListControllers>(controller_service.as_str()) {
                        Ok(client) => controllers = client,
                        Err(_) => return,
                    }
                }
                if hardware_failed {
                    match node.create_client::<ListHardwareComponents>(hardware_service.as_str()) {
                        Ok(client) => hardware = client,
                        Err(_) => return,
                    }
                }
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
        }));
    }
    Ok(tasks)
}
