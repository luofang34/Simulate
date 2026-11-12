use super::HeadlessGpu;

#[test]
fn the_device_has_timestamp_queries_wherever_the_adapter_offers_them() {
    let Ok(gpu) = pollster::block_on(HeadlessGpu::new()) else {
        return;
    };
    let offered = gpu
        .adapter
        .features()
        .contains(wgpu::Features::TIMESTAMP_QUERY);
    assert_eq!(
        gpu.device
            .features()
            .contains(wgpu::Features::TIMESTAMP_QUERY),
        offered
    );
}
