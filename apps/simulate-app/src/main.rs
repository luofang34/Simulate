//! Simulate: composite split-viewport cockpit for desktop (`Metal/Vulkan/DX12`)
//! and the browser (`WebGPU`, falling back to `WebGL2`), integrating `maplibre-rs`
//! (`TruthSceneMap` + avionics-preset instrument maps), `Indicate` PFD/HSI/HUD and 3D
//! volumetric scenes, `SimSupervisor`, JSBSim aircraft, and Tier-0 `Aviate`.

mod app;
mod avionics;
mod camera_view;
mod chain_avionics;
mod cockpit_rig;
mod controls;
mod environment;
mod error;
mod esc_menu;
mod flight;
#[cfg(not(target_arch = "wasm32"))]
mod gallery;
mod gamepad;
mod hangar;
mod host_guidance;
mod keyboard;
mod livery_session;
mod map_rig;
mod page_map;
mod page_status;
mod platform;
mod sensor_bus;
mod sky_rig;
mod supervisor_rig;
mod telemetry;
mod vehicles;
mod world_surface;

#[cfg(test)]
mod tests;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> std::process::ExitCode {
    platform::run()
}

#[cfg(target_arch = "wasm32")]
fn main() {
    platform::run();
}
