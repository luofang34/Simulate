use naga::back::glsl;

const GLOBE: &str = include_str!("../shaders/globe.wgsl");
const APERTURE: &str = include_str!("../shaders/aperture.wgsl");
const COMPOSITOR: &str = include_str!("../shaders/compositor.wgsl");
const VECTOR_SCENE: &str = include_str!("../shaders/vector_scene.wgsl");

fn parse(source: &str) -> (naga::Module, naga::valid::ModuleInfo) {
    let module = naga::front::wgsl::parse_str(source)
        .unwrap_or_else(|e| panic!("{}", e.emit_to_string(source)));
    let info = naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::empty(),
    )
    .validate(&module)
    .unwrap_or_else(|e| panic!("{e:?}"));
    (module, info)
}

/// WGSL can be valid and still have no WebGL2 form, so each entry point is
/// put through the GLSL ES 3.00 writer the browser fallback uses.
fn writes_webgl2_glsl(source: &str, entry_points: &[(naga::ShaderStage, &str)]) {
    let (module, info) = parse(source);
    for &(stage, entry_point) in entry_points {
        let options = glsl::Options {
            version: glsl::Version::new_gles(300),
            ..Default::default()
        };
        let pipeline = glsl::PipelineOptions {
            shader_stage: stage,
            entry_point: entry_point.to_owned(),
            multiview: None,
        };
        let mut output = String::new();
        let mut writer = glsl::Writer::new(
            &mut output,
            &module,
            &info,
            &options,
            &pipeline,
            naga::proc::BoundsCheckPolicies::default(),
        )
        .unwrap_or_else(|e| panic!("{entry_point}: {e:?}"));
        writer
            .write()
            .unwrap_or_else(|e| panic!("{entry_point}: {e:?}"));
    }
}

// Shader errors otherwise surface only when a GPU device compiles them at
// launch; validating here makes them a test failure.
#[test]
fn globe_shader_parses_and_validates() {
    parse(GLOBE);
}

#[test]
fn aperture_shader_parses_and_validates() {
    parse(APERTURE);
}

#[test]
fn globe_shader_has_a_webgl2_form() {
    writes_webgl2_glsl(
        GLOBE,
        &[
            (naga::ShaderStage::Vertex, "vs_main"),
            (naga::ShaderStage::Fragment, "fs_main"),
            (naga::ShaderStage::Fragment, "fs_thermal"),
        ],
    );
}

#[test]
fn aperture_shader_has_a_webgl2_form() {
    writes_webgl2_glsl(
        APERTURE,
        &[
            (naga::ShaderStage::Vertex, "vs_full_frame"),
            (naga::ShaderStage::Fragment, "fs_stars"),
            (naga::ShaderStage::Fragment, "fs_overlay"),
        ],
    );
}

#[test]
fn compositor_shader_parses_and_validates() {
    parse(COMPOSITOR);
}

#[test]
fn compositor_shader_has_a_webgl2_form() {
    writes_webgl2_glsl(
        COMPOSITOR,
        &[
            (naga::ShaderStage::Vertex, "vs_quad"),
            (naga::ShaderStage::Fragment, "fs_quad"),
        ],
    );
}

#[test]
fn vector_scene_shader_parses_and_validates() {
    parse(VECTOR_SCENE);
}

#[test]
fn vector_scene_shader_has_a_webgl2_form() {
    writes_webgl2_glsl(
        VECTOR_SCENE,
        &[
            (naga::ShaderStage::Vertex, "vs_vector"),
            (naga::ShaderStage::Fragment, "fs_vector"),
        ],
    );
}
