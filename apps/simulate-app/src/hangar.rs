//! The hub hangar's live 3D vehicle preview: one [`HangarPreviewPass`] on the window's
//! device, the built-in models it can show, the orbit camera, and articulation driven by
//! the pilot's live stick input so control surfaces and rotors move while the hub is open.

use std::collections::HashMap;
use std::sync::Arc;

use sim_core::{AirframeArticulation, ControlInputs, VehicleState};
use sim_geo::Geodetic;
use sim_maplibre::{HangarOrbit, HangarPreviewPass, UsdAircraftAsset};

use crate::error::AppSimError;
use crate::esc_menu::catalog::HangarModel;

/// Preview pass, model cache, and orbit camera.
pub struct HangarRig {
    pass: HangarPreviewPass,
    orbit: HangarOrbit,
    assets: HashMap<HangarModel, Arc<UsdAircraftAsset>>,
    model: Option<HangarModel>,
    elapsed_s: f64,
    shown: bool,
}

impl HangarRig {
    /// Compiles the preview pipelines on `device`.
    pub fn new(device: &wgpu::Device) -> Result<Self, AppSimError> {
        Ok(Self {
            pass: HangarPreviewPass::new(device, sim_maplibre::HANGAR_DEFAULT_SIZE)?,
            orbit: HangarOrbit::default(),
            assets: HashMap::new(),
            model: None,
            elapsed_s: 0.0,
            shown: false,
        })
    }

    /// Whether the turntable is on.
    #[must_use]
    pub const fn auto_rotate(&self) -> bool {
        self.orbit.auto_rotate
    }

    /// Toggles the turntable.
    pub fn toggle_auto_rotate(&mut self) {
        self.orbit.auto_rotate = !self.orbit.auto_rotate;
    }

    /// Returns the camera to its starting orbit, distance, and turntable.
    pub fn reset_view(&mut self) {
        self.orbit = HangarOrbit::default();
    }

    /// Orbits by a mouse drag of `[dx, dy]` logical units.
    pub fn drag(&mut self, [dx, dy]: [f64; 2]) {
        self.orbit.auto_rotate = false;
        self.orbit.nudge(dx * 0.01, dy * 0.01);
    }

    /// Zooms by mouse-wheel rows.
    pub fn zoom(&mut self, rows: f64) {
        self.orbit.zoom_by(1.1_f64.powf(-rows));
    }

    /// Renders `model` at `size_px`, posed by `controls`; the view is shown this frame.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        model: HangarModel,
        size_px: [u32; 2],
        controls: ControlInputs,
        elapsed_s: f64,
    ) -> Result<(), AppSimError> {
        self.pass.ensure_size(device, size_px)?;
        if self.model != Some(model) {
            let asset = self.asset(model)?;
            self.pass.set_asset(device, queue, &asset)?;
            self.model = Some(model);
        }
        self.orbit.advance(elapsed_s);
        if elapsed_s.is_finite() && elapsed_s > 0.0 {
            self.elapsed_s += elapsed_s.min(0.25);
        }
        let articulation = preview_articulation(controls, self.elapsed_s);
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("hangar preview"),
        });
        self.pass
            .render(device, queue, &mut encoder, &articulation, &self.orbit)?;
        queue.submit([encoder.finish()]);
        self.shown = true;
        Ok(())
    }

    /// Hides the preview until the next [`Self::render`].
    pub fn hide(&mut self) {
        self.shown = false;
    }

    /// The resolved `Rgba8UnormSrgb` preview, when rendered this frame.
    #[must_use]
    pub fn view(&self) -> Option<&wgpu::TextureView> {
        self.shown.then(|| self.pass.view())
    }

    fn asset(&mut self, model: HangarModel) -> Result<Arc<UsdAircraftAsset>, AppSimError> {
        if let Some(asset) = self.assets.get(&model) {
            return Ok(Arc::clone(asset));
        }
        let asset = Arc::new(load_model(model)?);
        self.assets.insert(model, Arc::clone(&asset));
        Ok(asset)
    }
}

/// The built-in asset behind `model`.
pub fn load_model(model: HangarModel) -> Result<UsdAircraftAsset, AppSimError> {
    Ok(match model {
        HangarModel::C172 => UsdAircraftAsset::embedded_n642sp()?,
        HangarModel::P38g => UsdAircraftAsset::p38g()?,
        HangarModel::X500 => UsdAircraftAsset::x500_quadrotor()?,
        HangarModel::FixedWingUav => UsdAircraftAsset::fixed_wing_uav()?,
        HangarModel::LunarLander => UsdAircraftAsset::lunar_lander()?,
        HangarModel::Raketa => UsdAircraftAsset::embedded_raketa()?,
    })
}

/// Surfaces deflected and rotors spun as `controls` would command them.
#[must_use]
pub fn preview_articulation(controls: ControlInputs, elapsed_s: f64) -> AirframeArticulation {
    let pose = VehicleState::level_flight(&Geodetic::from_degrees(0.0, 0.0, 0.0), 0.0, 40.0);
    AirframeArticulation::from_kinematic(&pose, controls, elapsed_s)
}
