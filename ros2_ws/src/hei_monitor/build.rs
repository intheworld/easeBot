fn main() {
    // rclrs 0.7 caches its native paths without watching AMENT_PREFIX_PATH.
    // Always link this binary against the active overlay, including message
    // libraries added after rclrs itself was first compiled.
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    if let Some(prefixes) = std::env::var_os("AMENT_PREFIX_PATH") {
        for prefix in std::env::split_paths(&prefixes) {
            let lib = prefix.join("lib");
            if lib.is_dir() {
                println!("cargo:rustc-link-search=native={}", lib.display());
            }
        }
    }
}
