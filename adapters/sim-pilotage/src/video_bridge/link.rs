use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

use sim_render::ApertureImage;

use crate::PilotageBridgeError;
use crate::video_bridge::wire::BridgeFrame;

/// Canonical pixel format string required by Pilotage's `session-host` MJPEG encoder.
pub const PIXEL_FORMAT_RGB_INT8: &str = "RGB_INT8";

/// Default loopback TCP port Pilotage's accepted-producer camera mode listens on.
pub const DEFAULT_SIDECAR_PORT: u16 = 45990;

/// Video source ID for the onboard forward FPV camera.
pub const CAMERA_ID_FPV: u32 = 0;
/// Video source ID for an external chase camera.
pub const CAMERA_ID_CHASE: u32 = 1;
/// Video source ID for a controllable gimbal payload camera.
pub const CAMERA_ID_GIMBAL: u32 = 2;

/// Strips the alpha channel from tightly packed `RGBA8` bytes (`4` bytes/pixel)
/// to produce `RGB_INT8` (`3` bytes/pixel).
pub fn rgba8_to_rgb_int8(rgba: &[u8]) -> Result<Vec<u8>, PilotageBridgeError> {
    let (pixels, remainder) = rgba.as_chunks::<4>();
    if !remainder.is_empty() {
        return Err(PilotageBridgeError::VideoWire {
            detail: format!("RGBA8 buffer length {} is not a multiple of 4", rgba.len()),
        });
    }
    let mut rgb = Vec::with_capacity(pixels.len() * 3);
    for &[r, g, b, _] in pixels {
        rgb.extend_from_slice(&[r, g, b]);
    }
    Ok(rgb)
}

/// Converts an offscreen [`ApertureImage`] (`RGBA8UnormSrgb`) into a ready-to-send
/// [`BridgeFrame`] stamped with `sim_time_ns` and `camera_id`.
pub fn aperture_image_to_bridge_frame(
    image: &ApertureImage,
    sim_time_ns: u64,
    camera_id: u32,
) -> Result<BridgeFrame, PilotageBridgeError> {
    let expected_rgba = (image.width as usize)
        .saturating_mul(image.height as usize)
        .saturating_mul(4);
    if image.rgba8.len() != expected_rgba {
        return Err(PilotageBridgeError::VideoWire {
            detail: format!(
                "ApertureImage {}x{} expected {} RGBA bytes, got {}",
                image.width,
                image.height,
                expected_rgba,
                image.rgba8.len()
            ),
        });
    }
    Ok(BridgeFrame {
        width: image.width,
        height: image.height,
        pixel_format: PIXEL_FORMAT_RGB_INT8.to_owned(),
        sim_time_ns,
        rgb: rgba8_to_rgb_int8(&image.rgba8)?,
        camera_id,
    })
}

/// Bounded single-slot latest-frame queue between the render loop and the
/// sidecar TCP writer. Submitting a new frame before an older one is sent
/// overwrites the stale frame immediately and increments [`Self::dropped_frames`].
#[derive(Debug, Default)]
pub struct LatestFrameQueue {
    slot: Mutex<Option<BridgeFrame>>,
    ready: Condvar,
    closed: AtomicBool,
    dropped: AtomicU64,
}

impl LatestFrameQueue {
    /// Creates an empty latest-frame queue.
    #[must_use]
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Publishes `frame` as the newest pending frame. Returns `false` if the
    /// queue has been closed.
    pub fn submit(&self, frame: BridgeFrame) -> bool {
        if self.closed.load(Ordering::Acquire) {
            return false;
        }
        let mut guard = match self.slot.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if guard.replace(frame).is_some() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        self.ready.notify_one();
        true
    }

    /// Non-blocking take of the latest pending frame.
    pub fn try_take(&self) -> Option<BridgeFrame> {
        let mut guard = match self.slot.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        guard.take()
    }

    /// Blocks until a frame is available or the queue closes.
    pub fn take_blocking(&self) -> Option<BridgeFrame> {
        let mut guard = match self.slot.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        loop {
            if let Some(frame) = guard.take() {
                return Some(frame);
            }
            if self.closed.load(Ordering::Acquire) {
                return None;
            }
            guard = match self.ready.wait(guard) {
                Ok(g) => g,
                Err(poisoned) => poisoned.into_inner(),
            };
        }
    }

    /// Closes the queue and wakes any blocked consumer.
    pub fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.ready.notify_all();
    }

    /// Number of superseded frames dropped before transmission.
    #[must_use]
    pub fn dropped_frames(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod tcp {
    use std::io::{Read, Write};
    use std::net::{Shutdown, SocketAddr, TcpStream};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    use std::thread::JoinHandle;

    use tracing::{debug, warn};

    use super::LatestFrameQueue;
    use crate::PilotageBridgeError;
    use crate::video_bridge::pose::CameraViewState;
    use crate::video_bridge::wire::{
        BridgeCameraCommand, BridgeControl, BridgeEnvelope, BridgeFrame, BridgeNavSat,
        BridgeOdometry, BridgePayload, BridgeViewPose, decode_length_delimited_envelope,
        encode_length_delimited_envelope,
    };

    #[derive(Debug, Default, Clone)]
    struct InboundState {
        control: Option<BridgeControl>,
        camera_command: Option<BridgeCameraCommand>,
        view_pose: Option<BridgeViewPose>,
        view_state: CameraViewState,
    }

    /// Live localhost TCP producer connected to Pilotage's `sim-video` listener.
    #[derive(Debug)]
    pub struct SimVideoSidecar {
        stream: TcpStream,
        frames: Arc<LatestFrameQueue>,
        telemetry_tx: Arc<Mutex<Vec<BridgeEnvelope>>>,
        inbound: Arc<Mutex<InboundState>>,
        alive: Arc<AtomicBool>,
        reader_thread: Option<JoinHandle<()>>,
        writer_thread: Option<JoinHandle<()>>,
    }

    impl SimVideoSidecar {
        /// Dials `addr` (`127.0.0.1:port`), sets `TCP_NODELAY`, and starts
        /// background reader and writer loops.
        pub fn connect(addr: SocketAddr) -> Result<Self, PilotageBridgeError> {
            let stream = TcpStream::connect(addr)?;
            Self::from_stream(stream)
        }

        /// Wraps an already-connected [`TcpStream`].
        pub fn from_stream(stream: TcpStream) -> Result<Self, PilotageBridgeError> {
            stream.set_nodelay(true)?;
            let read_stream = stream.try_clone()?;
            let write_stream = stream.try_clone()?;

            let frames = LatestFrameQueue::new();
            let telemetry_tx = Arc::new(Mutex::new(Vec::new()));
            let inbound = Arc::new(Mutex::new(InboundState::default()));
            let alive = Arc::new(AtomicBool::new(true));

            let reader_inbound = Arc::clone(&inbound);
            let reader_alive = Arc::clone(&alive);
            let reader_frames = Arc::clone(&frames);
            let reader_thread = std::thread::spawn(move || {
                run_reader(read_stream, &reader_inbound, &reader_alive, &reader_frames);
            });

            let writer_frames = Arc::clone(&frames);
            let writer_telemetry = Arc::clone(&telemetry_tx);
            let writer_alive = Arc::clone(&alive);
            let writer_thread = std::thread::spawn(move || {
                run_writer(
                    write_stream,
                    &writer_frames,
                    &writer_telemetry,
                    &writer_alive,
                );
            });

            Ok(Self {
                stream,
                frames,
                telemetry_tx,
                inbound,
                alive,
                reader_thread: Some(reader_thread),
                writer_thread: Some(writer_thread),
            })
        }

        /// Enqueues `frame` into the single-slot [`LatestFrameQueue`] for transmission.
        #[must_use]
        pub fn submit_frame(&self, frame: BridgeFrame) -> bool {
            if !self.is_connected() {
                return false;
            }
            self.frames.submit(frame)
        }

        /// Enqueues an outbound `BridgeOdometry` sample alongside the next write cycle.
        pub fn send_odometry(&self, odometry: BridgeOdometry) {
            let mut guard = match self.telemetry_tx.lock() {
                Ok(g) => g,
                Err(p) => p.into_inner(),
            };
            guard.push(BridgeEnvelope {
                payload: Some(BridgePayload::Odometry(odometry)),
            });
            self.frames.ready.notify_one();
        }

        /// Enqueues an outbound `BridgeNavSat` fix alongside the next write cycle.
        pub fn send_navsat(&self, navsat: BridgeNavSat) {
            let mut guard = match self.telemetry_tx.lock() {
                Ok(g) => g,
                Err(p) => p.into_inner(),
            };
            guard.push(BridgeEnvelope {
                payload: Some(BridgePayload::NavSat(navsat)),
            });
            self.frames.ready.notify_one();
        }

        /// Current folded [`CameraViewState`] (`mode`, `pan_rad`, `tilt_rad`, `zoom_detent`).
        #[must_use]
        pub fn effective_view_state(&self) -> CameraViewState {
            match self.inbound.lock() {
                Ok(g) => g.view_state,
                Err(p) => p.into_inner().view_state,
            }
        }

        /// Most recent raw `BridgeCameraCommand`, if any arrived.
        #[must_use]
        pub fn latest_camera_command(&self) -> Option<BridgeCameraCommand> {
            match self.inbound.lock() {
                Ok(g) => g.camera_command,
                Err(p) => p.into_inner().camera_command,
            }
        }

        /// Most recent `BridgeControl`, if any arrived.
        #[must_use]
        pub fn latest_control(&self) -> Option<BridgeControl> {
            match self.inbound.lock() {
                Ok(g) => g.control,
                Err(p) => p.into_inner().control,
            }
        }

        /// Most recent external `BridgeViewPose`, if any arrived.
        #[must_use]
        pub fn latest_view_pose(&self) -> Option<BridgeViewPose> {
            match self.inbound.lock() {
                Ok(g) => g.view_pose,
                Err(p) => p.into_inner().view_pose,
            }
        }

        /// Count of camera frames superseded before socket write.
        #[must_use]
        pub fn dropped_frames(&self) -> u64 {
            self.frames.dropped_frames()
        }

        /// Whether the TCP connection is still active.
        #[must_use]
        pub fn is_connected(&self) -> bool {
            self.alive.load(Ordering::Acquire)
        }
    }

    impl Drop for SimVideoSidecar {
        fn drop(&mut self) {
            self.alive.store(false, Ordering::Release);
            self.frames.close();
            if let Err(err) = self.stream.shutdown(Shutdown::Both) {
                debug!(%err, "video bridge shutdown returned non-fatal error");
            }
            if let Some(handle) = self.reader_thread.take()
                && handle.join().is_err()
            {
                warn!("video bridge reader thread panicked on join");
            }
            if let Some(handle) = self.writer_thread.take()
                && handle.join().is_err()
            {
                warn!("video bridge writer thread panicked on join");
            }
        }
    }

    fn run_reader(
        mut stream: TcpStream,
        inbound: &Mutex<InboundState>,
        alive: &AtomicBool,
        frames: &LatestFrameQueue,
    ) {
        let mut accum = Vec::with_capacity(1024);
        let mut chunk = [0_u8; 1024];
        while alive.load(Ordering::Acquire) {
            match stream.read(&mut chunk) {
                Ok(0) => {
                    debug!("video bridge peer closed socket");
                    break;
                }
                Ok(n) => {
                    if let Some(bytes) = chunk.get(..n) {
                        accum.extend_from_slice(bytes);
                    }
                    if !drain_envelopes(&mut accum, inbound) {
                        break;
                    }
                }
                Err(err) => {
                    if alive.load(Ordering::Acquire) {
                        warn!(%err, "video bridge socket read ended");
                    }
                    break;
                }
            }
        }
        alive.store(false, Ordering::Release);
        frames.close();
    }

    fn drain_envelopes(accum: &mut Vec<u8>, inbound: &Mutex<InboundState>) -> bool {
        loop {
            match decode_length_delimited_envelope(accum) {
                Ok(Some((env, consumed))) => {
                    accum.drain(..consumed);
                    apply_inbound(env, inbound);
                }
                Ok(None) => return true,
                Err(err) => {
                    warn!(%err, "malformed envelope on video bridge; closing");
                    return false;
                }
            }
        }
    }

    fn apply_inbound(envelope: BridgeEnvelope, inbound: &Mutex<InboundState>) {
        let mut guard = match inbound.lock() {
            Ok(g) => g,
            Err(p) => p.into_inner(),
        };
        match envelope.payload {
            Some(BridgePayload::Control(c)) => guard.control = Some(c),
            Some(BridgePayload::CameraCommand(cmd)) => {
                guard.camera_command = Some(cmd);
                if let Err(err) = guard.view_state.apply_command(&cmd) {
                    warn!(%err, "ignoring invalid BridgeCameraCommand");
                }
            }
            Some(BridgePayload::ViewPose(pose)) => {
                if let Some(gimbal) = &pose.gimbal
                    && let Err(err) = guard.view_state.apply_gimbal_pose(gimbal)
                {
                    warn!(%err, "ignoring invalid BridgeGimbalPose");
                }
                guard.view_pose = Some(pose);
            }
            _ => {}
        }
    }

    fn run_writer(
        mut stream: TcpStream,
        frames: &LatestFrameQueue,
        telemetry_tx: &Mutex<Vec<BridgeEnvelope>>,
        alive: &AtomicBool,
    ) {
        while alive.load(Ordering::Acquire) {
            let pending_tel: Vec<BridgeEnvelope> = {
                let mut guard = match telemetry_tx.lock() {
                    Ok(g) => g,
                    Err(p) => p.into_inner(),
                };
                std::mem::take(&mut *guard)
            };
            for env in pending_tel {
                let bytes = encode_length_delimited_envelope(&env);
                if stream.write_all(&bytes).is_err() {
                    alive.store(false, Ordering::Release);
                    frames.close();
                    return;
                }
            }
            let Some(frame) = frames.take_blocking() else {
                if telemetry_empty(telemetry_tx) {
                    break;
                }
                continue;
            };
            let env = BridgeEnvelope {
                payload: Some(BridgePayload::Frame(frame)),
            };
            let bytes = encode_length_delimited_envelope(&env);
            if let Err(err) = stream.write_all(&bytes) {
                warn!(%err, "video bridge socket write failed");
                alive.store(false, Ordering::Release);
                frames.close();
                return;
            }
        }
    }

    fn telemetry_empty(telemetry_tx: &Mutex<Vec<BridgeEnvelope>>) -> bool {
        match telemetry_tx.lock() {
            Ok(g) => g.is_empty(),
            Err(p) => p.into_inner().is_empty(),
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use tcp::SimVideoSidecar;
