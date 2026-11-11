use navigate_data::{
    Accuracy, Encoding, Fetched, GeoidModel, HorizontalDatum, LayerKind, LayerLimits, Revision,
    SourceKey, TileAddress, VerticalDatum,
};

use crate::{HttpAnswer, NavdataError, SourceSet, TileSource, fetched, tile_url};

#[test]
fn layers_are_declared_as_navigates_package_tool_declares_them() {
    let [imagery, elevation] = SourceSet::default().layer_specs();
    assert_eq!(
        (imagery.key.kind, imagery.key.source, imagery.revision),
        (
            LayerKind::Imagery,
            SourceKey::EsriWorldImagery,
            Revision::Unversioned
        )
    );
    assert_eq!(
        (elevation.key.kind, elevation.key.source, elevation.revision),
        (
            LayerKind::Elevation,
            SourceKey::MapzenTerrarium,
            Revision::Unversioned
        )
    );
    assert_eq!(
        (imagery.encoding, imagery.vertical_datum),
        (Encoding::Jpeg, None)
    );
    assert_eq!(
        (elevation.encoding, elevation.vertical_datum),
        (
            Encoding::TerrariumPng,
            Some(VerticalDatum::Orthometric {
                geoid: GeoidModel::Egm96
            })
        )
    );
    for spec in [imagery, elevation] {
        assert_eq!(spec.horizontal_datum, HorizontalDatum::Wgs84);
        assert_eq!(spec.tile_pixels, 256);
        assert_eq!(spec.accuracy, Accuracy::unknown());
        assert_eq!(spec.correlation_zoom, 16);
    }
}

#[test]
fn sources_carry_navigates_measured_limits() {
    assert_eq!(
        TileSource::ESRI_WORLD_IMAGERY.limits,
        LayerLimits::imagery()
    );
    assert_eq!(
        TileSource::MAPZEN_TERRARIUM.limits,
        LayerLimits::terrarium()
    );
    assert_eq!(TileSource::REGISTRY.len(), 2);
}

#[test]
fn each_source_addresses_a_tile_its_own_way() {
    let sources = SourceSet::default();
    let address = TileAddress::new(16, 32_577, 24_319).unwrap();
    let imagery = sources.tile_key(LayerKind::Imagery, address);
    assert_eq!(
        tile_url(sources.of(LayerKind::Imagery), &imagery),
        "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/16/24319/32577"
    );
    let elevation = sources.tile_key(LayerKind::Elevation, address);
    assert_eq!(
        tile_url(sources.of(LayerKind::Elevation), &elevation),
        "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/16/32577/24319.png"
    );
}

#[test]
fn the_descriptor_carries_each_layers_attribution_and_terms() {
    let descriptor = SourceSet::default().descriptor("zaragoza");
    assert_eq!(descriptor.release_id, "zaragoza");
    let [imagery, elevation] = descriptor.layers.as_slice() else {
        panic!("two layers");
    };
    assert_eq!(
        imagery.attribution,
        "Esri, Maxar, Earthstar Geographics, and the GIS User Community"
    );
    assert_eq!(elevation.attribution, "Mapzen / AWS terrain tiles");
    assert!(!imagery.licence.is_empty() && !elevation.licence.is_empty());
}

#[test]
fn an_answer_is_a_tile_no_tile_or_a_failure() {
    let answer = |status: u16| HttpAnswer {
        status,
        body: vec![1, 2, 3],
    };
    let esri = &TileSource::ESRI_WORLD_IMAGERY;
    assert_eq!(
        fetched(esri, "u", answer(200)),
        Ok(Fetched::Tile(vec![1, 2, 3]))
    );
    assert_eq!(fetched(esri, "u", answer(204)), Ok(Fetched::Absent));
    assert_eq!(fetched(esri, "u", answer(404)), Ok(Fetched::Absent));
    for status in [301, 403, 429, 500, 503] {
        assert_eq!(
            fetched(esri, "https://tiles.example/1/2/3", answer(status)),
            Err(NavdataError::Transport {
                url: "https://tiles.example/1/2/3".into(),
                message: format!("HTTP status {status}"),
            })
        );
    }
}

#[test]
fn a_no_data_picture_is_a_gap_of_its_own_source_only() {
    let picture = b"a provider's no-data picture".to_vec();
    let digest = navigate_data::Sha256Digest::of(&picture);
    let declaring = TileSource {
        no_data_bodies: Box::leak(Box::new([digest])),
        ..TileSource::ESRI_WORLD_IMAGERY
    };
    let answer = |body: &[u8]| HttpAnswer {
        status: 200,
        body: body.to_vec(),
    };
    assert_eq!(
        fetched(&declaring, "u", answer(&picture)),
        Ok(Fetched::Absent)
    );
    assert_eq!(
        fetched(&TileSource::MAPZEN_TERRARIUM, "u", answer(&picture)),
        Ok(Fetched::Tile(picture.clone()))
    );
    let mut altered = picture.clone();
    altered.push(0);
    assert_eq!(
        fetched(&declaring, "u", answer(&altered)),
        Ok(Fetched::Tile(altered))
    );
}

#[test]
fn esri_declares_its_grey_no_data_picture() {
    let [declared] = TileSource::ESRI_WORLD_IMAGERY.no_data_bodies else {
        panic!("one no-data body for Esri");
    };
    assert_eq!(
        declared.to_string(),
        "9eafd300d61393184a4abc1d458564cfd1cd9b6f9c4e9c74687045c0a0e5b858"
    );
    assert!(TileSource::MAPZEN_TERRARIUM.no_data_bodies.is_empty());
}
