//! PLY reader for point clouds, triangle meshes and 3D Gaussian splat files
//! (ascii, binary little- and big-endian).

use std::path::Path;

use super::cloud::{PointCloud, TriMesh};
use super::ProductError;

/// Zeroth-order spherical-harmonic constant: rgb = 0.5 + SH_C0 * f_dc.
const SH_C0: f64 = 0.282_094_791_773_878_14;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Encoding {
    Ascii,
    Le,
    Be,
}

#[derive(Clone, Copy)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(s: &str) -> Result<Self, ProductError> {
        Ok(match s {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            other => return Err(fmt_err(format!("unknown PLY type {other}"))),
        })
    }

    fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }
}

enum Kind {
    Scalar(Scalar),
    List(Scalar, Scalar),
}

struct Prop {
    name: String,
    kind: Kind,
}

struct Element {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

fn fmt_err(msg: impl Into<String>) -> ProductError {
    ProductError::Format(msg.into())
}

/// Sequential value source over the PLY body.
enum Body<'a> {
    Bin { buf: &'a [u8], pos: usize, be: bool },
    Ascii(std::str::SplitAsciiWhitespace<'a>),
}

impl Body<'_> {
    fn read(&mut self, s: Scalar) -> Result<f64, ProductError> {
        match self {
            Body::Ascii(toks) => toks
                .next()
                .ok_or_else(|| fmt_err("PLY body truncated"))?
                .parse::<f64>()
                .map_err(|e| fmt_err(format!("bad PLY value: {e}"))),
            Body::Bin { buf, pos, be } => {
                let n = s.size();
                let b = buf
                    .get(*pos..*pos + n)
                    .ok_or_else(|| fmt_err("PLY body truncated"))?;
                *pos += n;
                let be = *be;
                macro_rules! num {
                    ($t:ty) => {{
                        let arr = b.try_into().expect("sized slice");
                        (if be {
                            <$t>::from_be_bytes(arr)
                        } else {
                            <$t>::from_le_bytes(arr)
                        }) as f64
                    }};
                }
                Ok(match s {
                    Scalar::I8 => b[0] as i8 as f64,
                    Scalar::U8 => b[0] as f64,
                    Scalar::I16 => num!(i16),
                    Scalar::U16 => num!(u16),
                    Scalar::I32 => num!(i32),
                    Scalar::U32 => num!(u32),
                    Scalar::F32 => num!(f32),
                    Scalar::F64 => num!(f64),
                })
            }
        }
    }

    fn skip(&mut self, kind: &Kind) -> Result<(), ProductError> {
        match kind {
            Kind::Scalar(s) => match self {
                Body::Bin { pos, .. } => {
                    *pos += s.size();
                    Ok(())
                }
                Body::Ascii(_) => self.read(*s).map(|_| ()),
            },
            Kind::List(c, item) => {
                let n = self.read(*c)? as usize;
                for _ in 0..n {
                    self.read(*item)?;
                }
                Ok(())
            }
        }
    }

    /// Skips a whole element whose properties are all scalars.
    fn skip_fixed(&mut self, el: &Element) -> Result<bool, ProductError> {
        let Body::Bin { buf, pos, .. } = self else {
            return Ok(false);
        };
        let mut stride = 0;
        for p in &el.props {
            match p.kind {
                Kind::Scalar(s) => stride += s.size(),
                Kind::List(..) => return Ok(false),
            }
        }
        let end = *pos + stride * el.count;
        if end > buf.len() {
            return Err(fmt_err("PLY body truncated"));
        }
        *pos = end;
        Ok(true)
    }
}

fn parse_header(bytes: &[u8]) -> Result<(Encoding, Vec<Element>, usize), ProductError> {
    const END: &[u8] = b"end_header";
    let at = bytes
        .windows(END.len())
        .position(|w| w == END)
        .ok_or_else(|| fmt_err("PLY header has no end_header"))?;
    let mut body = at + END.len();
    if bytes.get(body) == Some(&b'\r') {
        body += 1;
    }
    if bytes.get(body) == Some(&b'\n') {
        body += 1;
    }
    let text = std::str::from_utf8(&bytes[..at]).map_err(|_| fmt_err("PLY header not text"))?;
    let mut lines = text.lines().map(str::trim);
    if lines.next() != Some("ply") {
        return Err(fmt_err("not a PLY file"));
    }
    let mut encoding = None;
    let mut elements: Vec<Element> = Vec::new();
    for line in lines {
        let tok: Vec<&str> = line.split_ascii_whitespace().collect();
        match tok.as_slice() {
            ["format", f, ..] => {
                encoding = Some(match *f {
                    "ascii" => Encoding::Ascii,
                    "binary_little_endian" => Encoding::Le,
                    "binary_big_endian" => Encoding::Be,
                    other => return Err(fmt_err(format!("unknown PLY format {other}"))),
                })
            }
            ["element", name, count] => elements.push(Element {
                name: (*name).to_string(),
                count: count
                    .parse()
                    .map_err(|_| fmt_err(format!("bad element count {count}")))?,
                props: Vec::new(),
            }),
            ["property", "list", c, item, name] => elements
                .last_mut()
                .ok_or_else(|| fmt_err("PLY property before element"))?
                .props
                .push(Prop {
                    name: (*name).to_string(),
                    kind: Kind::List(Scalar::parse(c)?, Scalar::parse(item)?),
                }),
            ["property", ty, name] => elements
                .last_mut()
                .ok_or_else(|| fmt_err("PLY property before element"))?
                .props
                .push(Prop {
                    name: (*name).to_string(),
                    kind: Kind::Scalar(Scalar::parse(ty)?),
                }),
            _ => {}
        }
    }
    let encoding = encoding.ok_or_else(|| fmt_err("PLY header has no format line"))?;
    Ok((encoding, elements, body))
}

/// What each vertex property feeds.
#[derive(Clone, Copy, PartialEq)]
enum Slot {
    Skip,
    Pos(usize),
    Rgb(usize, f64),
    Sh(usize),
}

fn vertex_slots(el: &Element) -> Result<(Vec<Slot>, bool, bool), ProductError> {
    let mut slots = Vec::with_capacity(el.props.len());
    for p in &el.props {
        let Kind::Scalar(s) = p.kind else {
            slots.push(Slot::Skip);
            continue;
        };
        // Integer colours scale to 16 bits; float colours are 0..1.
        let rgb_scale = match s {
            Scalar::U8 | Scalar::I8 => 257.0,
            Scalar::U16 | Scalar::I16 => 1.0,
            _ => 65535.0,
        };
        slots.push(match p.name.as_str() {
            "x" => Slot::Pos(0),
            "y" => Slot::Pos(1),
            "z" => Slot::Pos(2),
            "red" | "r" => Slot::Rgb(0, rgb_scale),
            "green" | "g" => Slot::Rgb(1, rgb_scale),
            "blue" | "b" => Slot::Rgb(2, rgb_scale),
            "f_dc_0" => Slot::Sh(0),
            "f_dc_1" => Slot::Sh(1),
            "f_dc_2" => Slot::Sh(2),
            _ => Slot::Skip,
        });
    }
    let has = |f: &dyn Fn(Slot) -> bool| slots.iter().filter(|s| f(**s)).count() == 3;
    if !has(&|s| matches!(s, Slot::Pos(_))) {
        return Err(fmt_err("PLY vertex element lacks x/y/z"));
    }
    let rgb = has(&|s| matches!(s, Slot::Rgb(..)));
    let sh = !rgb && has(&|s| matches!(s, Slot::Sh(_)));
    Ok((slots, rgb, sh))
}

fn read_vertices(
    body: &mut Body,
    el: &Element,
    want_color: bool,
) -> Result<PointCloud, ProductError> {
    let (slots, rgb, sh) = vertex_slots(el)?;
    let color = want_color && (rgb || sh);
    let mut points = Vec::with_capacity(el.count);
    let mut colors = Vec::with_capacity(if color { el.count } else { 0 });
    for _ in 0..el.count {
        let mut p = [0.0; 3];
        let mut c = [0.0; 3];
        for (prop, slot) in el.props.iter().zip(&slots) {
            match (slot, &prop.kind) {
                (Slot::Pos(i), Kind::Scalar(s)) => p[*i] = body.read(*s)?,
                (Slot::Rgb(i, k), Kind::Scalar(s)) => c[*i] = body.read(*s)? * k,
                (Slot::Sh(i), Kind::Scalar(s)) => {
                    c[*i] = ((0.5 + SH_C0 * body.read(*s)?).clamp(0.0, 1.0)) * 65535.0
                }
                (_, kind) => body.skip(kind)?,
            }
        }
        points.push(p);
        if color {
            colors.push(c.map(|v| v.round().clamp(0.0, 65535.0) as u16));
        }
    }
    Ok(PointCloud {
        points,
        colors: color.then_some(colors),
    })
}

fn read_faces(body: &mut Body, el: &Element, out: &mut Vec<[u32; 3]>) -> Result<(), ProductError> {
    let idx = el
        .props
        .iter()
        .position(|p| {
            matches!(p.kind, Kind::List(..))
                && (p.name == "vertex_indices" || p.name == "vertex_index")
        })
        .ok_or_else(|| fmt_err("PLY face element lacks vertex_indices"))?;
    let mut poly: Vec<u32> = Vec::new();
    for _ in 0..el.count {
        for (i, prop) in el.props.iter().enumerate() {
            match (&prop.kind, i == idx) {
                (Kind::List(c, item), true) => {
                    let n = body.read(*c)? as usize;
                    poly.clear();
                    for _ in 0..n {
                        poly.push(body.read(*item)? as u32);
                    }
                    for k in 1..n.saturating_sub(1) {
                        out.push([poly[0], poly[k], poly[k + 1]]);
                    }
                }
                (kind, _) => body.skip(kind)?,
            }
        }
    }
    Ok(())
}

fn load(path: &Path, faces: bool) -> Result<(PointCloud, Vec<[u32; 3]>), ProductError> {
    let bytes = std::fs::read(path)?;
    let (encoding, elements, start) = parse_header(&bytes)?;
    let mut body = match encoding {
        Encoding::Ascii => Body::Ascii(
            std::str::from_utf8(&bytes[start..])
                .map_err(|_| fmt_err("ascii PLY body not text"))?
                .split_ascii_whitespace(),
        ),
        Encoding::Le | Encoding::Be => Body::Bin {
            buf: &bytes[start..],
            pos: 0,
            be: encoding == Encoding::Be,
        },
    };
    let mut cloud = None;
    let mut tris = Vec::new();
    for el in &elements {
        match el.name.as_str() {
            "vertex" if cloud.is_none() => cloud = Some(read_vertices(&mut body, el, !faces)?),
            "face" if faces => read_faces(&mut body, el, &mut tris)?,
            _ => {
                if cloud.is_some() && (!faces || !tris.is_empty()) {
                    break;
                }
                if !body.skip_fixed(el)? {
                    for _ in 0..el.count {
                        for p in &el.props {
                            body.skip(&p.kind)?;
                        }
                    }
                }
            }
        }
    }
    let cloud = cloud.ok_or_else(|| fmt_err("PLY has no vertex element"))?;
    Ok((cloud, tris))
}

/// Reads the vertices of a PLY file, with colour from `red/green/blue` or a
/// splat's `f_dc_0..2` (zeroth-order SH) when present.
pub fn read_ply_points(path: &Path) -> Result<PointCloud, ProductError> {
    load(path, false).map(|(c, _)| c)
}

/// Reads a PLY triangle mesh; polygons are fan-triangulated.
pub fn read_ply_mesh(path: &Path) -> Result<TriMesh, ProductError> {
    let (cloud, triangles) = load(path, true)?;
    let n = cloud.points.len() as u32;
    if triangles.iter().flatten().any(|&i| i >= n) {
        return Err(fmt_err("PLY face index out of range"));
    }
    Ok(TriMesh {
        vertices: cloud.points,
        triangles,
    })
}
