//! Persistent local disk tile cache (`SIMULATE_TILE_CACHE`) for `link-replay`.
//!
//! Stores Mapzen Terrarium elevation PNGs and satellite orthoimagery JPEGs on
//! disk under `{cache_dir}/{provider}/{z}/{x}/{y}.{ext}` using atomic
//! write-to-temp-file + rename so repeat corridor runs execute with zero
//! network wait (`0 ms` HTTP latency) and `--offline` runs deterministically.

mod synthetic;

#[cfg(test)]
mod tests;

use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use sim_navdata::{HttpAnswer, TileTransport};

pub use synthetic::synthetic_tile_answer;

/// Environment variable overriding the default persistent tile cache root.
pub const TILE_CACHE_ENV_VAR: &str = "SIMULATE_TILE_CACHE";

/// Longest single upstream `curl` tile download allowed, seconds.
#[cfg(not(target_arch = "wasm32"))]
const REQUEST_TIMEOUT_S: &str = "60";

/// Cache miss handling policy for [`DiskCachedTransport`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CachePolicy {
    /// Read from disk first; on miss, download via `curl` and persist atomically.
    #[default]
    Online,
    /// Read exclusively from disk; fail immediately on any uncached tile (`--offline`).
    Offline,
    /// Read from disk first; on miss, synthesize deterministic procedural DEM/imagery
    /// tiles and persist them atomically (for offline integration tests).
    SyntheticFallback,
}

/// Snapshot of cache hit, miss, and network transfer counters for a run.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CacheStatsSnapshot {
    /// Resolved root directory of the tile cache.
    pub cache_dir: String,
    /// Number of tile requests satisfied directly from local disk.
    pub hits: u64,
    /// Number of tile requests not initially present on disk.
    pub misses: u64,
    /// Number of upstream network HTTP requests issued (`0` when warm or `--offline`).
    pub network_fetches: u64,
    /// Number of procedural fallback tiles generated (`0` in normal online/offline runs).
    pub synthetic_tiles: u64,
    /// Total tile payload bytes read from disk.
    pub bytes_read: u64,
    /// Total tile payload bytes written to disk.
    pub bytes_written: u64,
}

#[derive(Debug, Default)]
struct Counters {
    hits: AtomicU64,
    misses: AtomicU64,
    network_fetches: AtomicU64,
    synthetic_tiles: AtomicU64,
    bytes_read: AtomicU64,
    bytes_written: AtomicU64,
    temp_seq: AtomicU64,
}

/// Persistent local disk cache implementing [`sim_navdata::TileTransport`].
#[derive(Debug, Clone)]
pub struct DiskCachedTransport {
    root: PathBuf,
    policy: CachePolicy,
    counters: Arc<Counters>,
}

impl DiskCachedTransport {
    /// Creates a transport rooted at `explicit_dir`, `SIMULATE_TILE_CACHE`, or
    /// `$HOME/.cache/simulate/tiles`.
    #[must_use]
    pub fn new(explicit_dir: Option<PathBuf>, policy: CachePolicy) -> Self {
        Self {
            root: resolve_cache_dir(explicit_dir),
            policy,
            counters: Arc::new(Counters::default()),
        }
    }

    /// Root directory where tiles are cached.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Current hit/miss/network statistics snapshot.
    #[must_use]
    pub fn stats(&self) -> CacheStatsSnapshot {
        CacheStatsSnapshot {
            cache_dir: self.root.display().to_string(),
            hits: self.counters.hits.load(Ordering::Relaxed),
            misses: self.counters.misses.load(Ordering::Relaxed),
            network_fetches: self.counters.network_fetches.load(Ordering::Relaxed),
            synthetic_tiles: self.counters.synthetic_tiles.load(Ordering::Relaxed),
            bytes_read: self.counters.bytes_read.load(Ordering::Relaxed),
            bytes_written: self.counters.bytes_written.load(Ordering::Relaxed),
        }
    }

    /// Canonical cache file path for `url` inside this cache root.
    #[must_use]
    pub fn path_for_url(&self, url: &str) -> PathBuf {
        tile_cache_path(&self.root, url)
    }

    fn try_read_disk(&self, path: &Path) -> Result<Option<HttpAnswer>, String> {
        if let Ok(bytes) = fs::read(path) {
            self.counters.hits.fetch_add(1, Ordering::Relaxed);
            self.counters
                .bytes_read
                .fetch_add(bytes.len() as u64, Ordering::Relaxed);
            return Ok(Some(HttpAnswer {
                status: 200,
                body: bytes,
            }));
        }
        let absent = absent_marker_path(path);
        if absent.exists() {
            self.counters.hits.fetch_add(1, Ordering::Relaxed);
            return Ok(Some(HttpAnswer {
                status: 404,
                body: Vec::new(),
            }));
        }
        Ok(None)
    }

    fn persist_answer(&self, path: &Path, ans: &HttpAnswer) -> Result<(), String> {
        match ans.status {
            200 => {
                self.atomic_write(path, &ans.body)?;
                self.counters
                    .bytes_written
                    .fetch_add(ans.body.len() as u64, Ordering::Relaxed);
            }
            404 => {
                self.atomic_write(&absent_marker_path(path), &[])?;
            }
            _ => {}
        }
        Ok(())
    }

    fn atomic_write(&self, target: &Path, bytes: &[u8]) -> Result<(), String> {
        let parent = target
            .parent()
            .ok_or_else(|| format!("cache path {} has no parent directory", target.display()))?;
        fs::create_dir_all(parent)
            .map_err(|e| format!("creating cache dir {}: {e}", parent.display()))?;
        let seq = self.counters.temp_seq.fetch_add(1, Ordering::Relaxed);
        let pid = std::process::id();
        let file_name = target
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("tile");
        let tmp = parent.join(format!(".{file_name}.tmp-{pid}-{seq}"));
        let write_result = (|| -> std::io::Result<()> {
            let mut f = File::create(&tmp)?;
            f.write_all(bytes)?;
            f.sync_data()?;
            fs::rename(&tmp, target)
        })();
        if let Err(err) = write_result {
            drop(fs::remove_file(&tmp));
            return Err(format!("atomic cache write {}: {err}", target.display()));
        }
        Ok(())
    }
}

impl TileTransport for DiskCachedTransport {
    async fn get(&self, url: &str) -> Result<HttpAnswer, String> {
        let path = self.path_for_url(url);
        if let Some(hit) = self.try_read_disk(&path)? {
            return Ok(hit);
        }
        self.counters.misses.fetch_add(1, Ordering::Relaxed);
        match self.policy {
            CachePolicy::Offline => Err(format!(
                "offline tile cache miss for {url} (expected {})",
                path.display()
            )),
            CachePolicy::SyntheticFallback => {
                let ans = synthetic_tile_answer(url)?;
                self.counters
                    .synthetic_tiles
                    .fetch_add(1, Ordering::Relaxed);
                self.persist_answer(&path, &ans)?;
                Ok(ans)
            }
            CachePolicy::Online => {
                self.counters
                    .network_fetches
                    .fetch_add(1, Ordering::Relaxed);
                let ans = fetch_upstream_async(url).await?;
                self.persist_answer(&path, &ans)?;
                Ok(ans)
            }
        }
    }
}

/// Resolves the tile cache directory from `explicit`, `SIMULATE_TILE_CACHE`, or
/// `$HOME/.cache/simulate/tiles`.
#[must_use]
pub fn resolve_cache_dir(explicit: Option<PathBuf>) -> PathBuf {
    if let Some(dir) = explicit {
        return dir;
    }
    if let Some(env_dir) = std::env::var_os(TILE_CACHE_ENV_VAR).filter(|v| !v.is_empty()) {
        return PathBuf::from(env_dir);
    }
    if let Some(home) = std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .filter(|v| !v.is_empty())
    {
        return PathBuf::from(home).join(".cache/simulate/tiles");
    }
    std::env::temp_dir().join("simulate/tiles")
}

/// Maps an upstream tile URL onto a canonical `{provider}/{z}/{x}/{y}.{ext}`
/// path inside `root` (with SHA-256 fallback for unrecognized URL shapes).
#[must_use]
pub fn tile_cache_path(root: &Path, url: &str) -> PathBuf {
    let clean = url.split('?').next().unwrap_or(url);
    if let Some((z, x, y)) = parse_terrarium_zxy(clean) {
        return root.join(format!("terrarium/{z}/{x}/{y}.png"));
    }
    if let Some((z, x, y)) = parse_esri_zyx(clean) {
        return root.join(format!("esri_world_imagery/{z}/{x}/{y}.jpg"));
    }
    if let Some((z, x, y)) = parse_s2cloudless(clean) {
        return root.join(format!("s2cloudless/{z}/{x}/{y}.jpg"));
    }
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let digest = Sha256::digest(url.as_bytes());
    let mut hex = String::with_capacity(64);
    for byte in digest {
        hex.push(char::from(
            *HEX.get(usize::from(byte >> 4)).unwrap_or(&b'0'),
        ));
        hex.push(char::from(
            *HEX.get(usize::from(byte & 0x0F)).unwrap_or(&b'0'),
        ));
    }
    root.join("sha256").join(format!("{hex}.tile"))
}

fn absent_marker_path(tile_path: &Path) -> PathBuf {
    let mut os = tile_path.as_os_str().to_os_string();
    os.push(".absent");
    PathBuf::from(os)
}

pub(super) fn parse_terrarium_zxy(url: &str) -> Option<(u8, u32, u32)> {
    let tail = url.split("/terrarium/").nth(1)?;
    let trimmed = tail.strip_suffix(".png")?;
    parse_three_coords(trimmed, false)
}

pub(super) fn parse_esri_zyx(url: &str) -> Option<(u8, u32, u32)> {
    let tail = url.split("/World_Imagery/MapServer/tile/").nth(1)?;
    parse_three_coords(tail, true)
}

pub(super) fn parse_s2cloudless(url: &str) -> Option<(u8, u32, u32)> {
    if !url.contains("s2cloudless") {
        return None;
    }
    let trimmed = url
        .strip_suffix(".jpg")
        .or_else(|| url.strip_suffix(".jpeg"))?;
    let mut rev = trimmed.rsplit('/');
    let last = rev.next()?.parse::<u32>().ok()?;
    let mid = rev.next()?.parse::<u32>().ok()?;
    let zoom = rev.next()?.parse::<u8>().ok()?;
    // EOX WMTS uses /{z}/{y}/{x}.jpg; standard XYZ uses /{z}/{x}/{y}.jpg.
    if url.contains("wmts") {
        Some((zoom, last, mid))
    } else {
        Some((zoom, mid, last))
    }
}

fn parse_three_coords(path: &str, zyx_order: bool) -> Option<(u8, u32, u32)> {
    let mut parts = path.split('/');
    let z = parts.next()?.parse::<u8>().ok()?;
    let a = parts.next()?.parse::<u32>().ok()?;
    let b = parts.next()?.parse::<u32>().ok()?;
    if parts.next().is_some() {
        return None;
    }
    if zyx_order {
        Some((z, b, a))
    } else {
        Some((z, a, b))
    }
}

#[cfg(not(target_arch = "wasm32"))]
struct CurlFuture {
    state: Arc<std::sync::Mutex<CurlState>>,
}

#[cfg(not(target_arch = "wasm32"))]
struct CurlState {
    done: Option<Result<HttpAnswer, String>>,
    waker: Option<std::task::Waker>,
}

#[cfg(not(target_arch = "wasm32"))]
impl std::future::Future for CurlFuture {
    type Output = Result<HttpAnswer, String>;

    fn poll(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Self::Output> {
        let Ok(mut guard) = self.state.lock() else {
            return std::task::Poll::Ready(Err("tile fetch mutex poisoned".to_owned()));
        };
        if let Some(res) = guard.done.take() {
            return std::task::Poll::Ready(res);
        }
        guard.waker = Some(cx.waker().clone());
        std::task::Poll::Pending
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_upstream_async(url: &str) -> CurlFuture {
    let owned = url.to_owned();
    let state = Arc::new(std::sync::Mutex::new(CurlState {
        done: None,
        waker: None,
    }));
    let worker_state = Arc::clone(&state);
    std::thread::spawn(move || {
        let res = fetch_upstream_curl(&owned);
        if let Ok(mut guard) = worker_state.lock() {
            guard.done = Some(res);
            if let Some(waker) = guard.waker.take() {
                drop(guard);
                waker.wake();
            }
        }
    });
    CurlFuture { state }
}

#[cfg(target_arch = "wasm32")]
async fn fetch_upstream_async(_url: &str) -> Result<HttpAnswer, String> {
    Err("curl transport is unavailable on wasm32".to_owned())
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_upstream_curl(url: &str) -> Result<HttpAnswer, String> {
    let mut last_err = String::new();
    for attempt in 1_u32..=3 {
        let res = std::process::Command::new("curl")
            .args([
                "-sS",
                "-L",
                "--retry",
                "3",
                "--retry-delay",
                "1",
                "--max-time",
                REQUEST_TIMEOUT_S,
                "-w",
                "\n%{http_code}",
            ])
            .arg(url)
            .output()
            .map_err(|err| format!("curl failed to start: {err}"))
            .and_then(|output| {
                if !output.status.success() {
                    return Err(format!(
                        "curl {}: {}",
                        output.status,
                        String::from_utf8_lossy(&output.stderr).trim()
                    ));
                }
                split_curl_answer(output.stdout)
            });
        match res {
            Ok(ans) if (ans.status >= 500 || ans.status == 429) && attempt < 3 => {
                last_err = format!("HTTP status {}", ans.status);
                std::thread::sleep(std::time::Duration::from_millis(
                    u64::from(attempt).saturating_mul(600),
                ));
            }
            Ok(ans) => return Ok(ans),
            Err(err) if attempt < 3 => {
                last_err = err;
                std::thread::sleep(std::time::Duration::from_millis(
                    u64::from(attempt).saturating_mul(600),
                ));
            }
            Err(err) => return Err(err),
        }
    }
    Err(last_err)
}

#[cfg(any(not(target_arch = "wasm32"), test))]
pub(super) fn split_curl_answer(mut stdout: Vec<u8>) -> Result<HttpAnswer, String> {
    let split = stdout
        .iter()
        .rposition(|b| *b == b'\n')
        .ok_or_else(|| "curl output missing trailing HTTP status line".to_owned())?;
    let status = stdout
        .get(split.saturating_add(1)..)
        .and_then(|tail| std::str::from_utf8(tail).ok())
        .and_then(|s| s.trim().parse::<u16>().ok())
        .ok_or_else(|| "curl output carried an unreadable HTTP status".to_owned())?;
    stdout.truncate(split);
    Ok(HttpAnswer {
        status,
        body: stdout,
    })
}
