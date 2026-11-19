//! SVG drawing of a floor plan.

use std::fmt::Write as _;

use super::floorplan::Floorplan;

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// SVG drawing of one floor (100 px per metre, north up); `None` when the
/// plan has no such floor.
pub fn to_svg(plan: &Floorplan, floor: i32) -> Option<String> {
    let level = plan.floors.iter().find(|f| f.floor == floor)?;
    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    for s in level.walls.iter().chain(&level.doors) {
        xs.extend([s[0], s[2]]);
        ys.extend([s[1], s[3]]);
    }
    for r in &level.rooms {
        xs.extend(r.polygon.iter().map(|p| p[0]));
        ys.extend(r.polygon.iter().map(|p| p[1]));
    }
    let fold = |v: &[f64], f: fn(f64, f64) -> f64, init: f64| v.iter().copied().fold(init, f);
    let (x0, x1) = (fold(&xs, f64::min, 0.0), fold(&xs, f64::max, 1.0));
    let (y0, y1) = (fold(&ys, f64::min, 0.0), fold(&ys, f64::max, 1.0));
    let (px, pad) = (100.0, 0.5);
    let (w, h) = ((x1 - x0 + 2.0 * pad) * px, (y1 - y0 + 2.0 * pad) * px);
    let tx = |x: f64| (x - x0 + pad) * px;
    let ty = |y: f64| (y1 - y + pad) * px;
    let mut svg = String::new();
    let _ = write!(
        svg,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.0}\" height=\"{h:.0}\" viewBox=\"0 0 {w:.1} {h:.1}\">\n<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n"
    );
    for r in &level.rooms {
        let pts: Vec<String> = r
            .polygon
            .iter()
            .map(|p| format!("{:.1},{:.1}", tx(p[0]), ty(p[1])))
            .collect();
        let _ = writeln!(
            svg,
            "<polygon points=\"{}\" fill=\"#e8eef7\" stroke=\"#8aa0c0\" stroke-width=\"2\"/>",
            pts.join(" ")
        );
    }
    for s in &level.walls {
        let _ = writeln!(
            svg,
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#202020\" stroke-width=\"6\" stroke-linecap=\"square\"/>",
            tx(s[0]), ty(s[1]), tx(s[2]), ty(s[3])
        );
    }
    for s in &level.doors {
        let _ = writeln!(
            svg,
            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#d08020\" stroke-width=\"4\" stroke-dasharray=\"8 6\"/>",
            tx(s[0]), ty(s[1]), tx(s[2]), ty(s[3])
        );
    }
    for r in &level.rooms {
        let _ = writeln!(
            svg,
            "<text x=\"{:.1}\" y=\"{:.1}\" font-family=\"sans-serif\" font-size=\"22\" text-anchor=\"middle\">{} ({:.1} m²)</text>",
            tx(r.hotspot[0]), ty(r.hotspot[1]), xml_escape(&r.name), r.area_m2
        );
    }
    svg.push_str("</svg>\n");
    Some(svg)
}
