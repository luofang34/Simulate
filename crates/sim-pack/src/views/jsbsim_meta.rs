//! Lightweight JSBSim XML geometry extractors for parked gear clearance and cockpit eyepoint.

/// Extracts the static nose/main bogey ground clearance height (metres below CG)
/// from a JSBSim `<fdm_config>` XML document.
#[must_use]
pub fn jsbsim_parked_gear_height_m(xml: &str) -> Option<f64> {
    let ground_block = tag_body(xml, "ground_reactions")?;
    let cg_z_m = named_location_xyz_m(xml, "CG").map_or(0.0, |[_, _, z]| z);
    let mut deepest_drop_m = 0.0_f64;
    let mut rest = ground_block;
    while let Some((_, after_open)) = split_once_tag(rest, "contact") {
        let Some((tag_head, inner)) = after_open.split_once('>') else {
            break;
        };
        let Some((contact_body, remaining)) = inner.split_once("</contact>") else {
            break;
        };
        rest = remaining;
        let is_bogey =
            attr_value(tag_head, "type").is_some_and(|v| v.trim().eq_ignore_ascii_case("BOGEY"));
        if is_bogey
            && let Some((_, loc_block)) = split_once_tag(contact_body, "location")
            && let Some([_, _, z_m]) = parse_location_xyz_m(loc_block)
        {
            deepest_drop_m = deepest_drop_m.max(cg_z_m - z_m);
        }
    }
    (deepest_drop_m.is_finite() && deepest_drop_m > 0.05).then_some(deepest_drop_m)
}

/// Extracts the pilot eyepoint `[x_aft_in, y_right_in, z_up_in]` in JSBSim structural
/// inches from `<metrics><location name="EYEPOINT" ...>` if present.
#[must_use]
pub fn jsbsim_eyepoint_structural_in(xml: &str) -> Option<[f64; 3]> {
    let [x_m, y_m, z_m] = named_location_xyz_m(xml, "EYEPOINT")?;
    const M_TO_IN: f64 = 1.0 / 0.0254;
    Some([x_m * M_TO_IN, y_m * M_TO_IN, z_m * M_TO_IN])
}

fn tag_body<'a>(xml: &'a str, tag: &str) -> Option<&'a str> {
    let open = format!("<{tag}");
    let close = format!("</{tag}>");
    let (_, after_start) = xml.split_once(&open)?;
    let (_, after_gt) = after_start.split_once('>')?;
    let (body, _) = after_gt.split_once(&close)?;
    Some(body)
}

fn split_once_tag<'a>(xml: &'a str, tag: &str) -> Option<(&'a str, &'a str)> {
    let needle = format!("<{tag}");
    let mut scan = xml;
    loop {
        let (before, after) = scan.split_once(&needle)?;
        let next_ch = after.chars().next();
        if matches!(next_ch, Some(' ' | '\t' | '\r' | '\n' | '>' | '/')) {
            return Some((before, after));
        }
        scan = after;
    }
}

fn named_location_xyz_m(xml: &str, name: &str) -> Option<[f64; 3]> {
    let mut rest = xml;
    while let Some((_, after_open)) = split_once_tag(rest, "location") {
        let Some((tag_head, _)) = after_open.split_once('>') else {
            break;
        };
        if attr_value(tag_head, "name").is_some_and(|v| v.trim().eq_ignore_ascii_case(name))
            && let Some(xyz) = parse_location_xyz_m(after_open)
        {
            return Some(xyz);
        }
        rest = after_open;
    }
    None
}

fn parse_location_xyz_m(after_location_tag: &str) -> Option<[f64; 3]> {
    let (tag_head, after_gt) = after_location_tag.split_once('>')?;
    let (body, _) = after_gt.split_once("</location>")?;
    let scale = unit_to_metres(attr_value(tag_head, "unit").unwrap_or("IN"));
    let x = tag_number(body, "x")? * scale;
    let y = tag_number(body, "y")? * scale;
    let z = tag_number(body, "z")? * scale;
    (x.is_finite() && y.is_finite() && z.is_finite()).then_some([x, y, z])
}

fn unit_to_metres(unit: &str) -> f64 {
    let u = unit.trim();
    if u.eq_ignore_ascii_case("FT") {
        0.3048
    } else if u.eq_ignore_ascii_case("M") {
        1.0
    } else {
        0.0254
    }
}

fn tag_number(body: &str, tag: &str) -> Option<f64> {
    let open = format!("<{tag}>");
    let close = format!("</{tag}>");
    let (_, after) = body.split_once(&open)?;
    let (val, _) = after.split_once(&close)?;
    val.trim().parse::<f64>().ok()
}

fn attr_value<'a>(tag_head: &'a str, key: &str) -> Option<&'a str> {
    let needle = format!("{key}=\"");
    let (_, after) = tag_head.split_once(&needle)?;
    let (val, _) = after.split_once('"')?;
    Some(val)
}
