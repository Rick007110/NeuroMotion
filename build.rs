//! Embeds the app icon in the Windows executable, so Explorer, the taskbar and
//! shortcuts show it. Other platforms take the icon from the window at run time.

fn main() {
    println!("cargo:rerun-if-changed=assets/icon.ico");
    #[cfg(windows)]
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        let mut resource = winresource::WindowsResource::new();
        resource.set_icon("assets/icon.ico");
        resource
            .compile()
            .expect("embedding the icon in the executable failed");
    }
}
