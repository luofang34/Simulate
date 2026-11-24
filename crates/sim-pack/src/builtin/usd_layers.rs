//! The OpenUSD layers of the built-in Cessna packages, each embedded once: the packages
//! list them among their assets, and a renderer composes the built-in stages from the
//! same bytes without loading a package.

/// The generic BSD-3-Clause Cessna 172 airframe (`usd/c172.usdc`), the same file in the
/// `c172s` and the `c172p` package.
pub static C172_USDC: &[u8] = include_bytes!("../../../../vehicles/c172s/usd/c172.usdc");
/// The Cessna 172S equipment and bare-gear airframe layer (`usd/c172s_airframe.usda`).
pub static C172S_AIRFRAME_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172s/usd/c172s_airframe.usda");
/// The `N642SP` root layer (`usd/n642sp.usda`).
pub static N642SP_USDA: &[u8] = include_bytes!("../../../../vehicles/c172s/usd/n642sp.usda");
/// The `N642SP` livery layer (`usd/n642sp_livery.usda`).
pub static N642SP_LIVERY_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172s/usd/n642sp_livery.usda");
/// The `N172SP` root layer (`usd/n172sp.usda`).
pub static N172SP_USDA: &[u8] = include_bytes!("../../../../vehicles/c172s/usd/n172sp.usda");
/// The `N172SP` livery layer (`usd/n172sp_livery.usda`).
pub static N172SP_LIVERY_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172s/usd/n172sp_livery.usda");
/// The Cessna 172P / Reims F172P equipment and bare-gear airframe layer
/// (`usd/c172p_airframe.usda`).
pub static C172P_AIRFRAME_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172p/usd/c172p_airframe.usda");
/// The `D-ECJB` root layer (`usd/d_ecjb.usda`).
pub static D_ECJB_USDA: &[u8] = include_bytes!("../../../../vehicles/c172p/usd/d_ecjb.usda");
/// The `D-ECJB` livery layer (`usd/d_ecjb_livery.usda`).
pub static D_ECJB_LIVERY_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172p/usd/d_ecjb_livery.usda");
/// The `N53295` root layer (`usd/n53295.usda`).
pub static N53295_USDA: &[u8] = include_bytes!("../../../../vehicles/c172p/usd/n53295.usda");
/// The `N53295` livery layer (`usd/n53295_livery.usda`).
pub static N53295_LIVERY_USDA: &[u8] =
    include_bytes!("../../../../vehicles/c172p/usd/n53295_livery.usda");
