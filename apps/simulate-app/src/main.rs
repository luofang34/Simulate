//! Simulate: one executable for desktop (Metal/Vulkan/DX12) and the browser
//! (WebGPU, falling back to WebGL2), flown with keyboard, gamepad, joystick,
//! or yoke, with number keys to pick a scripted scenario.

mod app;
mod camera_view;
mod flight;
mod gamepad;
mod keyboard;
mod platform;
mod telemetry;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> std::process::ExitCode {
    platform::run()
}

#[cfg(target_arch = "wasm32")]
fn main() {
    platform::run();
}
