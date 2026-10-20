use crate::cache::{system_metrics, Cache, NAMES};
use axum::{
    body::Body,
    extract::State,
    http::{header, Method, StatusCode, Uri},
    response::{IntoResponse, Response},
    Router,
};
use std::sync::{Arc, Mutex};
pub type SharedCache = Arc<Mutex<Cache>>;
pub fn router(cache: SharedCache) -> Router {
    Router::new().fallback(handler).with_state(cache)
}
async fn handler(State(cache): State<SharedCache>, method: Method, uri: Uri) -> Response {
    let mut response = if method != Method::GET {
        (
            StatusCode::METHOD_NOT_ALLOWED,
            axum::Json(serde_json::json!({"error":"read-only monitor"})),
        )
            .into_response()
    } else {
        match uri.path() {
            "/" => (
                [(header::CONTENT_TYPE, "text/html; charset=utf-8")],
                include_str!("../web/index.html"),
            )
                .into_response(),
            "/style.css" => (
                [(header::CONTENT_TYPE, "text/css; charset=utf-8")],
                include_str!("../web/style.css"),
            )
                .into_response(),
            "/app.js" => (
                [(header::CONTENT_TYPE, "text/javascript; charset=utf-8")],
                include_str!("../web/app.js"),
            )
                .into_response(),
            "/api/v1/status" => {
                let mut status = cache.lock().expect("cache lock poisoned").snapshot();
                system_metrics(&mut status);
                axum::Json(status).into_response()
            }
            path if path.starts_with("/api/v1/cameras/") => {
                let name = path.trim_start_matches("/api/v1/cameras/");
                if !NAMES.contains(&name) {
                    (
                        StatusCode::NOT_FOUND,
                        axum::Json(serde_json::json!({"error":"unknown camera"})),
                    )
                        .into_response()
                } else {
                    let frame = cache.lock().expect("cache lock poisoned").frame(name);
                    match frame {
                        Some(frame) => ([(header::CONTENT_TYPE, "image/jpeg")], Body::from(frame))
                            .into_response(),
                        None => (
                            StatusCode::SERVICE_UNAVAILABLE,
                            axum::Json(serde_json::json!({"error":"camera unavailable or stale"})),
                        )
                            .into_response(),
                    }
                }
            }
            _ => (
                StatusCode::NOT_FOUND,
                axum::Json(serde_json::json!({"error":"not found"})),
            )
                .into_response(),
        }
    };
    response
        .headers_mut()
        .insert(header::CACHE_CONTROL, "no-store".parse().unwrap());
    response
        .headers_mut()
        .insert(header::X_CONTENT_TYPE_OPTIONS, "nosniff".parse().unwrap());
    response
}
