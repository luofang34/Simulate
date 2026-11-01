//! Simulate: composite split-viewport cockpit for desktop (`Metal/Vulkan/DX12`)
//! and the browser (`WebGPU`, falling back to `WebGL2`), integrating Dual-World
//! `maplibre-rs` (`TruthSceneMap` + `BeliefAvionicsMap`), `Indicate` PFD/HSI/HUD
//! and 3D volumetric scenes, `SimSupervisor`, JSBSim aircraft, and Tier-0 `Aviate`.

mod app;
mod avionics;
mod camera_view;
mod cockpit_rig;
mod error;
mod flight;
mod gamepad;
mod keyboard;
mod map_rig;
mod platform;
mod supervisor_rig;
mod telemetry;
mod vehicles;

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
