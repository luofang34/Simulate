//! Which device tier the maps run at, chosen from what the host reports about itself
//! rather than assumed: a desktop or laptop browser gets the browser tier, a phone, a
//! small machine or a software GPU the handheld one.

use sim_maplibre::CockpitDeviceTier;

#[cfg(test)]
mod tests;

/// Device memory below which a browser counts as small, GiB: the browser reports it in
/// coarse steps, so this is the step under a common laptop's.
#[cfg(any(target_arch = "wasm32", test))]
const SMALL_DEVICE_GIB: f64 = 8.0;
/// Largest GPU buffer below which the GPU counts as small, bytes.
#[cfg(any(target_arch = "wasm32", test))]
const SMALL_GPU_BUFFER_BYTES: u64 = 1 << 30;

#[cfg(any(target_arch = "wasm32", test))]
/// What a browser says about the device the page runs on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeviceFacts {
    /// Whether the browser says it is a mobile one, when it says.
    pub mobile: Option<bool>,
    /// The device memory the browser reports, GiB, when it reports one.
    pub device_memory_gib: Option<f64>,
    /// Whether the GPU adapter is a software fallback.
    pub fallback_adapter: bool,
    /// The largest GPU buffer the adapter allows, bytes.
    pub max_buffer_bytes: u64,
}

/// The tier the maps run at, and why.
#[derive(Debug, Clone, PartialEq)]
pub struct TierChoice {
    /// The tier.
    pub tier: CockpitDeviceTier,
    /// The facts that decided it, in plain words.
    pub reasons: Vec<String>,
}

impl TierChoice {
    /// The choice with its reasons, for the log and a page's tooltip.
    #[must_use]
    pub fn words(&self) -> String {
        format!(
            "map detail: {} tier ({})",
            self.tier_name(),
            self.reasons.join("; ")
        )
    }

    /// The choice in a short line, for a page's status.
    #[must_use]
    pub fn line(&self) -> String {
        format!("Map detail: {} tier", self.tier_name())
    }

    const fn tier_name(&self) -> &'static str {
        match self.tier {
            CockpitDeviceTier::Desktop => "desktop",
            CockpitDeviceTier::Browser => "browser",
            CockpitDeviceTier::Handheld => "handheld",
        }
    }
}

#[cfg(any(target_arch = "wasm32", test))]
/// The browser tier unless the device is a phone, small, or without a real GPU; then
/// the handheld tier, whose maps load less at once.
#[must_use]
pub fn choose_tier(facts: &DeviceFacts) -> TierChoice {
    let mut small = Vec::new();
    if facts.mobile == Some(true) {
        small.push("the browser says it is a mobile one".to_owned());
    }
    if let Some(gib) = facts
        .device_memory_gib
        .filter(|gib| *gib < SMALL_DEVICE_GIB)
    {
        small.push(format!("{gib} GiB of device memory"));
    }
    if facts.fallback_adapter {
        small.push("a software GPU".to_owned());
    }
    if facts.max_buffer_bytes < SMALL_GPU_BUFFER_BYTES {
        small.push(format!(
            "GPU buffers of at most {} MiB",
            facts.max_buffer_bytes >> 20
        ));
    }
    if !small.is_empty() {
        return TierChoice {
            tier: CockpitDeviceTier::Handheld,
            reasons: small,
        };
    }
    let memory = facts.device_memory_gib.map_or_else(
        || "device memory not reported".to_owned(),
        |gib| format!("{gib} GiB of device memory"),
    );
    TierChoice {
        tier: CockpitDeviceTier::Browser,
        reasons: vec![
            "not a mobile browser".to_owned(),
            memory,
            format!("GPU buffers up to {} MiB", facts.max_buffer_bytes >> 20),
        ],
    }
}

/// The tier for maps on `adapter` in this browser.
#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn choose_for(adapter: &wgpu::Adapter) -> TierChoice {
    let navigator = web_sys::window().map(|window| window.navigator());
    let read = |path: &[&str]| -> Option<wasm_bindgen::JsValue> {
        let mut value: wasm_bindgen::JsValue = navigator.clone()?.into();
        for key in path {
            value = js_sys::Reflect::get(&value, &(*key).into()).ok()?;
        }
        (!value.is_undefined() && !value.is_null()).then_some(value)
    };
    choose_tier(&DeviceFacts {
        mobile: read(&["userAgentData", "mobile"]).and_then(|value| value.as_bool()),
        device_memory_gib: read(&["deviceMemory"]).and_then(|value| value.as_f64()),
        fallback_adapter: adapter.get_info().device_type == wgpu::DeviceType::Cpu,
        max_buffer_bytes: adapter.limits().max_buffer_size,
    })
}

/// The tier for maps on a native host: the desktop one.
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn choose_for(_adapter: &wgpu::Adapter) -> TierChoice {
    TierChoice {
        tier: CockpitDeviceTier::Desktop,
        reasons: vec!["a native host".to_owned()],
    }
}
