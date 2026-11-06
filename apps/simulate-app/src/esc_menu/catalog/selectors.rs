//! Values the Flight tab's selectors cycle.

use sim_scenario::SimTimeScale;

/// Simulation-rate selector values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RateChoice {
    /// Clock frozen.
    Paused,
    /// Half speed.
    Half,
    /// Real time.
    Real,
    /// Double speed.
    Double,
    /// Four times speed.
    Quad,
}

impl RateChoice {
    /// Every rate in display order.
    pub const ALL: [Self; 5] = [
        Self::Paused,
        Self::Half,
        Self::Real,
        Self::Double,
        Self::Quad,
    ];

    /// The time scale this rate sets.
    #[must_use]
    pub const fn time_scale(self) -> SimTimeScale {
        match self {
            Self::Paused => SimTimeScale::Paused,
            Self::Half => SimTimeScale::Warp(0.5),
            Self::Real => SimTimeScale::RealTime,
            Self::Double => SimTimeScale::Warp(2.0),
            Self::Quad => SimTimeScale::Warp(4.0),
        }
    }

    /// The rate nearest `scale`.
    #[must_use]
    pub fn of(scale: SimTimeScale) -> Self {
        let factor = scale.factor();
        Self::ALL
            .into_iter()
            .min_by(|a, b| {
                let da = (a.time_scale().factor() - factor).abs();
                let db = (b.time_scale().factor() - factor).abs();
                da.total_cmp(&db)
            })
            .unwrap_or(Self::Real)
    }

    /// Label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Paused => "PAUSED",
            Self::Half => "0.5X",
            Self::Real => "1X",
            Self::Double => "2X",
            Self::Quad => "4X",
        }
    }
}

/// Instrument fit selector values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AvionicsLayout {
    /// Synthetic-vision PFD, moving map, and HSI.
    FullGlass,
    /// Attitude and HSI panels without maps.
    BasicSix,
    /// Conformal HUD only, no panel dock.
    HudOnly,
}

impl AvionicsLayout {
    /// Every layout in display order.
    pub const ALL: [Self; 3] = [Self::FullGlass, Self::BasicSix, Self::HudOnly];

    /// Label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::FullGlass => "FULL GLASS",
            Self::BasicSix => "BASIC SIX",
            Self::HudOnly => "HUD ONLY",
        }
    }
}

/// Tile source selector values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SceneryMode {
    /// Real-world imagery, DEM, and vectors over HTTP.
    Online,
    /// The bundled synthetic regional catalog.
    Offline,
}

impl SceneryMode {
    /// Label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Online => "ONLINE REAL-WORLD",
            Self::Offline => "OFFLINE CATALOG",
        }
    }
}
