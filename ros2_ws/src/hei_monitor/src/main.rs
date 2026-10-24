mod cache;
mod hardware;
mod http;
mod ros;
use rclrs::RclrsErrorFilter;
use std::sync::{Arc, Mutex};

#[tokio::main(worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let cache = Arc::new(Mutex::new(cache::Cache::default()));
    let (mut executor, _subscriptions, host, port) = ros::setup(cache.clone())?;
    // Bind before spinning so invalid host/occupied ports fail startup immediately.
    let listener = tokio::net::TcpListener::bind((host.as_str(), port)).await?;
    eprintln!(
        "Read-only Rust ROS 2 monitor: http://{}",
        listener.local_addr()?
    );
    let mut interrupt = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?;
    let mut terminate = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    let status_tasks = ros::start_status_polling(&_subscriptions, cache.clone())?;
    let commands = executor.commands().clone();
    let (done_tx, mut done_rx) = tokio::sync::oneshot::channel();
    let ros_thread = std::thread::spawn(move || {
        let result = executor.spin(rclrs::SpinOptions::default()).first_error();
        let _ = done_tx.send(());
        result
    });
    let server = axum::serve(listener, http::router(cache))
        .with_graceful_shutdown(async move {
            tokio::select! {
                _ = interrupt.recv() => {},
                _ = terminate.recv() => {},
                _ = &mut done_rx => {},
            }
        })
        .await;
    for task in status_tasks {
        task.abort();
        let _ = task.await;
    }
    commands.halt_spinning();
    let ros_result = ros_thread
        .join()
        .map_err(|_| std::io::Error::other("ROS executor panicked"))?;
    server?;
    ros_result?;
    Ok(())
}
