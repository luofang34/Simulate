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
    /// Whether the primary pointer is coarse (touch screen).
    pub coarse_pointer: bool,
    /// Maximum hardware touch points reported by the browser.
    pub max_touch_points: u32,
    /// Smaller CSS viewport edge in pixels, when reported by the browser.
    pub min_viewport_css_px: Option<f64>,
    /// Whether the User-Agent string identifies iPhone, iPad, iPod, Android, or Mobile.
    pub ios_like_ua: bool,
}

/// Whether a browser `User-Agent` string identifies a phone, tablet, or mobile browser.
#[cfg(any(target_arch = "wasm32", test))]
#[must_use]
pub fn is_mobile_user_agent(ua: &str) -> bool {
    let lower = ua.to_ascii_lowercase();
    ["iphone", "ipad", "ipod", "android", "mobile"]
        .into_iter()
        .any(|needle| lower.contains(needle))
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
    if facts.ios_like_ua {
        small.push("a mobile or iOS browser user agent".to_owned());
    }
    if facts.coarse_pointer && facts.max_touch_points > 0 {
        small.push("a touch-first pointer".to_owned());
    }
    if facts.max_touch_points > 1 && facts.min_viewport_css_px.is_some_and(|px| px <= 1024.0) {
        small.push("a compact touch display".to_owned());
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

#[cfg(target_arch = "wasm32")]
fn read_browser_device_facts(fallback_adapter: bool, max_buffer_bytes: u64) -> DeviceFacts {
    let window = web_sys::window();
    let navigator = window.as_ref().map(web_sys::Window::navigator);
    let read = |path: &[&str]| -> Option<wasm_bindgen::JsValue> {
        let mut value: wasm_bindgen::JsValue = navigator.clone()?.into();
        for key in path {
            value = js_sys::Reflect::get(&value, &(*key).into()).ok()?;
        }
        (!value.is_undefined() && !value.is_null()).then_some(value)
    };
    let coarse_pointer = window
        .as_ref()
        .and_then(|w| w.match_media("(pointer: coarse)").ok().flatten())
        .is_some_and(|mq| mq.matches());
    let max_touch_points = navigator.as_ref().map_or(0, |nav| {
        u32::try_from(nav.max_touch_points().max(0)).unwrap_or(0)
    });
    let min_viewport_css_px = window.as_ref().and_then(|w| {
        let width = w.inner_width().ok().and_then(|v| v.as_f64())?;
        let height = w.inner_height().ok().and_then(|v| v.as_f64())?;
        (width.is_finite() && height.is_finite() && width > 0.0 && height > 0.0)
            .then(|| width.min(height))
    });
    let ios_like_ua = navigator
        .as_ref()
        .and_then(|nav| nav.user_agent().ok())
        .is_some_and(|ua| is_mobile_user_agent(&ua));
    DeviceFacts {
        mobile: read(&["userAgentData", "mobile"]).and_then(|value| value.as_bool()),
        device_memory_gib: read(&["deviceMemory"]).and_then(|value| value.as_f64()),
        fallback_adapter,
        max_buffer_bytes,
        coarse_pointer,
        max_touch_points,
        min_viewport_css_px,
        ios_like_ua,
    }
}

/// Whether the current browser should run in the handheld tier based on Web APIs alone
/// (before a `wgpu::Adapter` is requested).
#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn browser_is_handheld() -> bool {
    let facts = read_browser_device_facts(false, u64::MAX);
    choose_tier(&facts).tier == CockpitDeviceTier::Handheld
}

/// The tier for maps on `adapter` in this browser.
#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn choose_for(adapter: &wgpu::Adapter) -> TierChoice {
    choose_tier(&read_browser_device_facts(
        adapter.get_info().device_type == wgpu::DeviceType::Cpu,
        adapter.limits().max_buffer_size,
    ))
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
