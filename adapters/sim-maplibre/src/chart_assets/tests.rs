use std::sync::Arc;

use super::*;

#[test]
fn clones_share_charts_served_by_url() {
    let store = ChartAssetStore::new();
    let loader_side = store.clone();
    store.put_chart_png("kewr-ils-22l", Arc::from(vec![1_u8, 2, 3]));
    let url = chart_url("kewr-ils-22l");
    assert_eq!(url, "https://sim.chart/kewr-ils-22l.png");
    assert_eq!(
        loader_side.chart_for_url(&url).as_deref(),
        Some(&[1, 2, 3][..])
    );
    assert_eq!(
        loader_side.chart_for_url(&format!("{url}?v=2")).as_deref(),
        Some(&[1, 2, 3][..])
    );
    assert!(
        loader_side
            .chart_for_url("https://sim.chart/other.png")
            .is_none()
    );
    assert!(
        loader_side
            .chart_for_url("https://sim.catalog/dem/0/0/0.png")
            .is_none()
    );
    assert!(store.remove_chart("kewr-ils-22l"));
    assert!(loader_side.chart_for_url(&url).is_none());
}
