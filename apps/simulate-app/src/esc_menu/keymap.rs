//! Keyboard keys the open hub understands.

use winit::event::{ElementState, KeyEvent};
use winit::keyboard::{KeyCode, PhysicalKey};

use super::{HubTab, MenuInput};

/// Maps a key press to a hub input: `Escape` always, the rest only while the hub is open.
/// `shift` selects `Shift+Tab`; `text` is the key's typed text, for the aerodrome search.
#[must_use]
pub fn menu_input_for_key(event: &KeyEvent, menu_open: bool, shift: bool) -> Option<MenuInput> {
    if event.state != ElementState::Pressed {
        return None;
    }
    let PhysicalKey::Code(code) = event.physical_key else {
        return None;
    };
    if code == KeyCode::Escape {
        return (!event.repeat).then_some(MenuInput::Toggle);
    }
    if !menu_open {
        return None;
    }
    if code == KeyCode::Tab {
        return (!event.repeat).then_some(if shift {
            MenuInput::PrevTab
        } else {
            MenuInput::NextTab
        });
    }
    menu_input_for_code(code, event.repeat).or_else(|| {
        let ch = event.text.as_ref()?.chars().next()?;
        (ch.is_ascii_alphanumeric() || ch == ' ').then_some(MenuInput::Char(ch))
    })
}

/// The open-hub input bound to `code`; `Enter` and tab jumps never auto-repeat. Letters
/// are left to the search box, so only the arrows navigate.
#[must_use]
pub fn menu_input_for_code(code: KeyCode, repeat: bool) -> Option<MenuInput> {
    match code {
        KeyCode::ArrowUp => Some(MenuInput::Up),
        KeyCode::ArrowDown => Some(MenuInput::Down),
        KeyCode::ArrowLeft => Some(MenuInput::Left),
        KeyCode::ArrowRight => Some(MenuInput::Right),
        KeyCode::Backspace => Some(MenuInput::Backspace),
        KeyCode::PageUp => Some(MenuInput::Scroll(-4)),
        KeyCode::PageDown => Some(MenuInput::Scroll(4)),
        KeyCode::Enter | KeyCode::NumpadEnter => (!repeat).then_some(MenuInput::Confirm),
        KeyCode::F1 => (!repeat).then_some(MenuInput::Tab(HubTab::Flight)),
        KeyCode::F2 => (!repeat).then_some(MenuInput::Tab(HubTab::Hangar)),
        KeyCode::F3 => (!repeat).then_some(MenuInput::Tab(HubTab::World)),
        KeyCode::F4 => (!repeat).then_some(MenuInput::Tab(HubTab::Scenarios)),
        KeyCode::F5 => (!repeat).then_some(MenuInput::Tab(HubTab::Controls)),
        KeyCode::Escape => (!repeat).then_some(MenuInput::Toggle),
        _ => None,
    }
}
