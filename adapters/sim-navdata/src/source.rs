//! The registry of tile sources and how each is declared in a package.

use navigate_data::{
    Accuracy, DEFAULT_CORRELATION_ZOOM, Descriptor, Encoding, GeoidModel, HorizontalDatum,
    LayerKey, LayerKind, LayerLabel, LayerLimits, LayerSpec, Revision, Sha256Digest, SourceKey,
    TileKey, VerticalDatum,
};

#[cfg(test)]
mod tests;

/// One source of tiles: where its tiles are, what they are, and the terms
/// they come under. Another source is another entry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TileSource {
    /// The source's key in the package identity.
    pub key: SourceKey,
    /// What its tiles hold.
    pub kind: LayerKind,
    /// The content version the provider states.
    pub revision: Revision,
    /// The address of a tile, with `{z}`, `{x}` and `{y}`.
    pub url_template: &'static str,
    /// A name for a reader.
    pub label: &'static str,
    /// The attribution the terms require.
    pub attribution: &'static str,
    /// The licence or terms of use.
    pub licence: &'static str,
    /// How a tile is encoded.
    pub encoding: Encoding,
    /// The reference of its heights, for elevation.
    pub vertical_datum: Option<VerticalDatum>,
    /// Zoom range and tile sizes, for the resource model.
    pub limits: LayerLimits,
    /// Digests of the bodies the source answers with status 200 where it
    /// has no tile: a fixed "no data" picture is a gap, not ground.
    pub no_data_bodies: &'static [Sha256Digest],
}

/// The 2521-byte grey "Map data not yet available" JPEG Esri World Imagery
/// answers, with status 200, for every tile it has no imagery for, at any
/// zoom. Asking with `blankTile=false` turns it into a 404, but that answer
/// carries no CORS header, so a page cannot tell it from a failed
/// connection.
const ESRI_NO_DATA: Sha256Digest = Sha256Digest([
    0x9e, 0xaf, 0xd3, 0x00, 0xd6, 0x13, 0x93, 0x18, 0x4a, 0x4a, 0xbc, 0x1d, 0x45, 0x85, 0x64, 0xcf,
    0xd1, 0xcd, 0x9b, 0x6f, 0x9c, 0x4e, 0x9c, 0x74, 0x68, 0x70, 0x45, 0xc0, 0xa0, 0xe5, 0xb8, 0x58,
]);

impl TileSource {
    /// Esri World Imagery: JPEG orthoimagery.
    pub const ESRI_WORLD_IMAGERY: Self = Self {
        key: SourceKey::EsriWorldImagery,
        kind: LayerKind::Imagery,
        revision: Revision::Unversioned,
        url_template: "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        label: "Esri World Imagery",
        attribution: "Esri, Maxar, Earthstar Geographics, and the GIS User Community",
        licence: "Esri World Imagery terms of use",
        encoding: Encoding::Jpeg,
        vertical_datum: None,
        limits: LayerLimits::imagery(),
        no_data_bodies: &[ESRI_NO_DATA],
    };

    /// Mapzen Terrarium: PNG elevation above mean sea level, the AWS
    /// terrain tiles.
    pub const MAPZEN_TERRARIUM: Self = Self {
        key: SourceKey::MapzenTerrarium,
        kind: LayerKind::Elevation,
        revision: Revision::Unversioned,
        url_template: "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png",
        label: "Mapzen Terrarium elevation",
        attribution: "Mapzen / AWS terrain tiles",
        licence: "Terrain Tiles on the Registry of Open Data on AWS; attribution required",
        encoding: Encoding::TerrariumPng,
        vertical_datum: Some(VerticalDatum::Orthometric {
            geoid: GeoidModel::Egm96,
        }),
        limits: LayerLimits::terrarium(),
        no_data_bodies: &[],
    };

    /// Every source this host knows.
    pub const REGISTRY: [Self; 2] = [Self::ESRI_WORLD_IMAGERY, Self::MAPZEN_TERRARIUM];

    /// How a package declares this source's layer: WGS84, 256-pixel tiles,
    /// unknown accuracy and the default correlation zoom, as Navigate's
    /// package tool declares it.
    pub fn layer_spec(&self) -> LayerSpec {
        LayerSpec {
            key: LayerKey {
                kind: self.kind,
                source: self.key,
            },
            revision: self.revision,
            horizontal_datum: HorizontalDatum::Wgs84,
            vertical_datum: self.vertical_datum,
            encoding: self.encoding,
            tile_pixels: self.limits.tile_pixels,
            accuracy: Accuracy::unknown(),
            correlation_zoom: DEFAULT_CORRELATION_ZOOM,
        }
    }
}

/// The sources of one package: one for imagery, one for elevation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceSet {
    /// The imagery source.
    pub imagery: TileSource,
    /// The elevation source.
    pub elevation: TileSource,
}

impl Default for SourceSet {
    /// Esri World Imagery over Mapzen Terrarium.
    fn default() -> Self {
        Self {
            imagery: TileSource::ESRI_WORLD_IMAGERY,
            elevation: TileSource::MAPZEN_TERRARIUM,
        }
    }
}

impl SourceSet {
    /// The source of a layer kind; elevation for anything but imagery.
    pub const fn of(&self, kind: LayerKind) -> &TileSource {
        match kind {
            LayerKind::Imagery => &self.imagery,
            _ => &self.elevation,
        }
    }

    /// The declarations of both layers.
    pub fn layer_specs(&self) -> [LayerSpec; 2] {
        [self.imagery.layer_spec(), self.elevation.layer_spec()]
    }

    /// The key of the tile of `kind` at `address`.
    pub const fn tile_key(&self, kind: LayerKind, address: navigate_data::TileAddress) -> TileKey {
        let source = self.of(kind);
        TileKey {
            source: source.key,
            revision: source.revision,
            layer: kind,
            address,
        }
    }

    /// What a package of these sources says about itself outside its
    /// identity: labels, attribution and terms.
    pub fn descriptor(&self, release_id: &str) -> Descriptor {
        let label = |source: &TileSource| LayerLabel {
            key: LayerKey {
                kind: source.kind,
                source: source.key,
            },
            label: source.label.to_owned(),
            attribution: source.attribution.to_owned(),
            licence: source.licence.to_owned(),
        };
        Descriptor {
            release_id: release_id.to_owned(),
            layers: vec![label(&self.imagery), label(&self.elevation)],
            note: String::new(),
        }
    }
}

/// The address of the tile `key` names at `source`.
pub fn tile_url(source: &TileSource, key: &TileKey) -> String {
    source
        .url_template
        .replace("{z}", &key.address.zoom.to_string())
        .replace("{x}", &key.address.x.to_string())
        .replace("{y}", &key.address.y.to_string())
}
