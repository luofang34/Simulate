//! Latched device readings, camera exposure metrics, and read-only trial
//! inspection accessors shared with [`crate::telemetry`].

use std::collections::VecDeque;

use glam::{DQuat, DVec3};
use sim_core::VehicleState;
use sim_devices::{AirDataSample, DeviceSamples, GnssReading};
use sim_flight_chain::{FcAvionics, NavigationDisplay};
use sim_geo::{Geodetic, LocalFrame, ecef_to_geodetic};
use sim_navigate::trial::{SensorKind, SensorLog, TerrainMatch};

use super::sets::SetSlot;
use super::spoof::ActiveSpoof;
use super::{C172Chain, ChainTrial, Steered, TruthSnapshot, feed};
use crate::{DeviceOutage, FrameRequest, GnssSpoofSpec, Retained};

/// Latest GNSS reading paired with the true antenna position at its fix instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LatchedGnssFix {
    /// Route time when the fix was measured, seconds.
    pub(crate) at_s: f64,
    /// Raw receiver fix (including noise and any active spoofing offset/pull).
    pub(crate) measured: GnssReading,
    /// True WGS84 antenna position at the exact cycle the fix was measured.
    pub(crate) true_position: Geodetic,
}

/// Metadata of the latest delivered terrain-camera frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct LatchedCameraFrame {
    /// Monotonic frame exposure index (from 1).
    pub(crate) index: u64,
    /// Route time of mid-exposure, seconds.
    pub(crate) mid_s: f64,
    /// Angle between the camera optical axis (`-Z_cam`) and local nadir, degrees.
    pub(crate) nadir_obliquity_deg: f64,
    /// Normalized RMS adjacent-pixel luminance contrast in `[0.0, 1.0]`.
    pub(crate) local_contrast: f64,
}

#[derive(Debug, Clone, Copy)]
struct PendingExposure {
    index: u64,
    mid_s: f64,
    size: (u32, u32),
    nadir_obliquity_deg: f64,
}

/// Latches slower device samples across faster flight-controller cycles so
/// telemetry sampling at any rate sees the freshest physical sensor states.
#[derive(Debug, Default)]
pub(crate) struct SensorLatch {
    pub(crate) air_data: Option<AirDataSample>,
    pub(crate) mag_body_ut: Option<DVec3>,
    pub(crate) gnss_fix: Option<LatchedGnssFix>,
    pub(crate) camera_frame: Option<LatchedCameraFrame>,
    pending_exposures: VecDeque<PendingExposure>,
}

impl SensorLatch {
    pub(super) fn on_cycle(
        &mut self,
        at_s: f64,
        after: &VehicleState,
        samples: &DeviceSamples,
        gnss_out: bool,
        baro_out: bool,
    ) {
        if baro_out {
            self.air_data = None;
        } else if let Some(air) = samples.air_data {
            self.air_data = Some(air);
        }
        if let Some(mag) = samples.magnetic_field_body_ut {
            self.mag_body_ut = Some(mag);
        }
        if gnss_out {
            self.gnss_fix = None;
        } else if let Some(measured) = samples.gnss {
            self.gnss_fix = Some(LatchedGnssFix {
                at_s,
                measured,
                true_position: ecef_to_geodetic(after.position_ecef),
            });
        }
    }

    pub(super) fn on_exposure(&mut self, request: &FrameRequest) {
        self.pending_exposures.push_back(PendingExposure {
            index: request.index,
            mid_s: request.mid_s,
            size: request.size,
            nadir_obliquity_deg: nadir_obliquity_deg(request),
        });
    }

    pub(super) fn on_frame_delivered(&mut self, index: u64, luma: &[u8]) {
        while let Some(front) = self.pending_exposures.front() {
            if front.index >= index {
                break;
            }
            self.pending_exposures.pop_front();
        }
        let pending = match self.pending_exposures.front().copied() {
            Some(front) if front.index == index => {
                self.pending_exposures.pop_front();
                Some(front)
            }
            _ => None,
        };
        if let Some(exp) = pending {
            self.camera_frame = Some(LatchedCameraFrame {
                index: exp.index,
                mid_s: exp.mid_s,
                nadir_obliquity_deg: exp.nadir_obliquity_deg,
                local_contrast: rms_neighbor_contrast(luma, exp.size.0),
            });
        }
    }
}

fn nadir_obliquity_deg(request: &FrameRequest) -> f64 {
    let [w, x, y, z] = request.camera_to_ecef_wxyz;
    let camera_to_ecef = DQuat::from_xyzw(x, y, z, w);
    let optical_axis = (camera_to_ecef * (-DVec3::Z)).normalize_or_zero();
    let eye = DVec3::from_array(request.eye_ecef);
    let nadir = -LocalFrame::at(&ecef_to_geodetic(eye)).up();
    optical_axis.dot(nadir).clamp(-1.0, 1.0).acos().to_degrees()
}

fn rms_neighbor_contrast(luma: &[u8], width: u32) -> f64 {
    let stride = width as usize;
    if stride == 0 || luma.is_empty() {
        return 0.0;
    }
    let mut sum_sq = 0.0_f64;
    let mut count = 0_u64;
    let mut prev_row: Option<&[u8]> = None;
    for row in luma.chunks_exact(stride) {
        for pair in row.windows(2) {
            if let [a, b] = pair {
                let diff = f64::from(*b) - f64::from(*a);
                sum_sq += diff * diff;
                count = count.wrapping_add(1);
            }
        }
        if let Some(prev) = prev_row {
            for (a, b) in prev.iter().zip(row.iter()) {
                let diff = f64::from(*b) - f64::from(*a);
                sum_sq += diff * diff;
                count = count.wrapping_add(1);
            }
        }
        prev_row = Some(row);
    }
    if count == 0 {
        0.0
    } else {
        (sum_sq / count as f64).sqrt() / 255.0
    }
}

impl ChainTrial {
    /// The control chain the trial flies: the board, its airframe and
    /// Pilotage's host, for drawing and readouts.
    pub fn chain(&self) -> &C172Chain {
        &self.chain
    }

    /// The flight controller's account of its last cycle: what the
    /// aeroplane's own displays may show.
    pub fn avionics(&self) -> FcAvionics {
        feed::avionics(&self.chain)
    }

    /// What the trial's log keeps: as much as its slowest reader has yet to
    /// take, however long the flight.
    pub fn retained(&self) -> Retained {
        self.nav.retained()
    }

    /// What Pilotage's navigation display shows of the mission now: the
    /// plan, the leg flown and the holding pattern after it. It is the
    /// flight-management side, apart from the flight controller's account.
    pub fn navigation(&self) -> Option<&NavigationDisplay> {
        self.chain.status().navigation.as_ref()
    }

    /// Each visual set's latest terrain match, by set name, for a trial built
    /// inspecting them.
    pub fn terrain_matches(&self) -> impl Iterator<Item = (&str, &TerrainMatch)> {
        self.sets.iter().filter_map(|slot| {
            slot.run
                .last_terrain_match()
                .map(|found| (slot.name.as_str(), found))
        })
    }

    /// The true aeroplane now, for drawing the world outside it only.
    pub fn truth(&self) -> TruthSnapshot {
        feed::truth(&self.chain)
    }

    /// The outages so far, those the spec stated and those switched.
    pub fn outages(&self) -> &[DeviceOutage] {
        &self.outages
    }

    /// The spoofer as flown so far: its drag and every span it held the
    /// receiver, those the spec stated and those switched.
    pub const fn gnss_spoof(&self) -> &GnssSpoofSpec {
        self.spoofer.spec()
    }

    /// The navigation Pilotage's mission steers on; `None` when it flies
    /// on the flight controller's own estimate.
    pub fn steered(&self) -> Option<Steered<'_>> {
        let slot = self.sets.get(self.steering?)?;
        Some(Steered {
            set: &slot.name,
            solution: self.offered.as_ref(),
        })
    }

    pub(crate) const fn sensor_latch(&self) -> &SensorLatch {
        &self.latch
    }

    pub(crate) fn spoofer_state(&self) -> ActiveSpoof {
        self.spoofer
            .active_state(self.chain.board().suite().receiver())
    }

    pub(crate) fn is_sensor_out(&self, kind: SensorKind, t_s: f64) -> bool {
        self.outages
            .iter()
            .any(|o| o.sensor == kind && (o.contains(t_s) || o.covers_up_to(t_s)))
    }

    pub(crate) const fn has_camera(&self) -> bool {
        self.camera.is_some()
    }

    pub(crate) fn set_slots_mut(&mut self) -> (&mut [SetSlot], &SensorLog) {
        (&mut self.sets, &self.nav.log)
    }
}
