// Shader errors otherwise surface only when a GPU device compiles them at
// launch; validating here makes them a test failure.
#[test]
fn globe_shader_parses_and_validates() {
    let source = include_str!("../shaders/globe.wgsl");
    let module = naga::front::wgsl::parse_str(source)
        .unwrap_or_else(|e| panic!("{}", e.emit_to_string(source)));
    naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::empty(),
    )
    .validate(&module)
    .unwrap_or_else(|e| panic!("{e:?}"));
}
