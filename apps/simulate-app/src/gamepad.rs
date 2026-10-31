use gilrs::{Axis, Button, EventType, GamepadId, Gilrs, MappingSource};
use sim_input::{AxisSource, DeviceId, DeviceInfo, GamepadAxis, InputMapper, Layout, RawInput};
use tracing::{info, warn};

/// Offset for gilrs `Unknown` axes so their raw codes cannot collide with the
/// six indices assigned to gilrs's named axes.
const UNKNOWN_AXIS_BASE: u32 = 16;

/// Reads gamepads, joysticks, and yokes through gilrs (OS HID natively, the
/// Gamepad API in the browser) and feeds them to the input mapper.
pub struct Gamepads {
    gilrs: Option<Gilrs>,
}

impl Gamepads {
    pub fn new() -> Self {
        match Gilrs::new() {
            Ok(gilrs) => Self { gilrs: Some(gilrs) },
            Err(err) => {
                warn!(%err, "controller input unavailable; keyboard only");
                Self { gilrs: None }
            }
        }
    }

    /// Drains pending controller events into `mapper`.
    pub fn poll(&mut self, mapper: &mut InputMapper) {
        let Some(gilrs) = self.gilrs.as_mut() else {
            return;
        };
        while let Some(event) = gilrs.next_event() {
            let device = device_id(event.id);
            if !mapper.is_bound(device) && !matches!(event.event, EventType::Disconnected) {
                mapper.handle(&RawInput::Connected(device_info(gilrs, event.id)));
            }
            if let Some(raw) = translate(gilrs, event.id, event.event) {
                mapper.handle(&raw);
            }
        }
    }
}

fn device_id(id: GamepadId) -> DeviceId {
    DeviceId(u32::try_from(usize::from(id)).unwrap_or(u32::MAX))
}

fn layout(gilrs: &Gilrs, id: GamepadId) -> Layout {
    match gilrs.gamepad(id).mapping_source() {
        MappingSource::None => Layout::NonStandard,
        MappingSource::SdlMappings | MappingSource::Driver => Layout::Standard,
    }
}

fn device_info(gilrs: &Gilrs, id: GamepadId) -> DeviceInfo {
    let pad = gilrs.gamepad(id);
    let info = DeviceInfo {
        id: device_id(id),
        name: pad.name().to_owned(),
        vendor_id: pad.vendor_id(),
        product_id: pad.product_id(),
        layout: layout(gilrs, id),
    };
    info!(name = %info.name, vendor = ?info.vendor_id, product = ?info.product_id, layout = ?info.layout, "controller connected");
    info
}

fn translate(gilrs: &Gilrs, id: GamepadId, event: EventType) -> Option<RawInput> {
    let device = device_id(id);
    let standard = layout(gilrs, id) == Layout::Standard;
    let source = match event {
        EventType::Disconnected => return Some(RawInput::Disconnected(device)),
        EventType::AxisChanged(axis, value, code) => {
            (axis_source(axis, code.into_u32(), standard)?, value)
        }
        EventType::ButtonChanged(Button::LeftTrigger2, value, _) => {
            (AxisSource::Gamepad(GamepadAxis::LeftTrigger), value)
        }
        EventType::ButtonChanged(Button::RightTrigger2, value, _) => {
            (AxisSource::Gamepad(GamepadAxis::RightTrigger), value)
        }
        _ => return None,
    };
    Some(RawInput::Axis {
        device,
        source: source.0,
        value: source.1,
    })
}

/// Named gilrs axes become semantic axes on standard-layout pads. On other
/// devices gilrs's names carry no meaning, so they are numbered in HID order
/// (X, Y, Z, Rx, Ry, Rz), the order the generic joystick profile assumes.
fn axis_source(axis: Axis, code: u32, standard: bool) -> Option<AxisSource> {
    if standard {
        let semantic = match axis {
            Axis::LeftStickX => GamepadAxis::LeftStickX,
            Axis::LeftStickY => GamepadAxis::LeftStickY,
            Axis::RightStickX => GamepadAxis::RightStickX,
            Axis::RightStickY => GamepadAxis::RightStickY,
            _ => return None,
        };
        return Some(AxisSource::Gamepad(semantic));
    }
    let index = match axis {
        Axis::LeftStickX => 0,
        Axis::LeftStickY => 1,
        Axis::LeftZ => 2,
        Axis::RightStickX => 3,
        Axis::RightStickY => 4,
        Axis::RightZ => 5,
        Axis::DPadX | Axis::DPadY => return None,
        Axis::Unknown => UNKNOWN_AXIS_BASE.saturating_add(code),
    };
    Some(AxisSource::Raw(index))
}
