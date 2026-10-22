//! Autodesk FBX 7.4 export: one character's skeleton (`LimbNode` models) and its motion
//! as animation curves.
//!
//! The scene is built once as a tree of FBX nodes, then written either as binary (the
//! default; Blender's importer only reads binary FBX) or as ASCII.
//!
//! * Every joint is a `LimbNode` whose static transform is its rest pose: rest offset
//!   in the parent's frame and rest rotation. Blender and Maya therefore show the
//!   character's own rest pose.
//! * Rotation curves hold the full local rotation (rest × animated) in degrees, keyed on
//!   every frame, in a per-joint rotation order chosen to stay clear of gimbal lock
//!   (`RotationActive` is set so importers honour it). Roots, and joints whose offset is
//!   animated, also get translation curves.
//! * The file declares Y up, Z forward, X across (right-handed, NeuroMotion's own
//!   space), and its unit, so importers convert to their own conventions.
//!
//! Details: docs/export.md.

use std::fmt::Write as _;
use std::path::Path;

use glam::{EulerRot, Quat};

use super::IoError;
use super::motion::{Motion, ORDERS, Units, export_names, joint_curves};
use crate::document::{SkeletonRef, euler_near};

pub const FILE_EXTENSION: &str = "fbx";
const VERSION: u32 = 7400;
/// FBX time units per second.
const KTIME_PER_SECOND: f64 = 46_186_158_000.0;
/// Fixed creation time and the file id and footer bytes that go with it; FBX readers
/// check that the three agree, and a fixed time keeps exports reproducible.
const CREATION_TIME: &str = "1970-01-01 10:00:00:000";
const FILE_ID: [u8; 16] = [0x28, 0xb3, 0x2a, 0xeb, 0xb6, 0x24, 0xcc, 0xc2, 0xbf, 0xc8, 0xb0, 0x2a, 0xa9, 0x2b, 0xfc, 0xf1];
const FOOTER_ID: [u8; 16] = [0xfa, 0xbc, 0xab, 0x09, 0xd0, 0xc8, 0xd4, 0x66, 0xb1, 0x76, 0xfb, 0x83, 0x1c, 0xf7, 0x26, 0x7e];
const FOOTER_MAGIC: [u8; 16] = [0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5, 0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3, 0x75, 0x8f, 0x29, 0x0b];
const TAKE: &str = "Take 001";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Encoding {
    /// What Blender, Maya, MotionBuilder, Unity and Unreal all read.
    #[default]
    Binary,
    /// Human-readable; Blender can't import it.
    Ascii,
}

impl Encoding {
    pub const ALL: [Encoding; 2] = [Encoding::Binary, Encoding::Ascii];

    pub fn label(self) -> &'static str {
        match self {
            Encoding::Binary => "Binary (Blender, Maya, game engines)",
            Encoding::Ascii => "ASCII (readable; not for Blender)",
        }
    }
}

pub fn save(path: &Path, skeleton: &SkeletonRef, motion: &Motion, units: Units, encoding: Encoding) -> Result<(), IoError> {
    std::fs::write(path, to_bytes(skeleton, motion, units, encoding))?;
    Ok(())
}

pub fn to_bytes(skeleton: &SkeletonRef, motion: &Motion, units: Units, encoding: Encoding) -> Vec<u8> {
    let nodes = scene(skeleton, motion, units);
    match encoding {
        Encoding::Binary => binary(&nodes),
        Encoding::Ascii => ascii(&nodes).into_bytes(),
    }
}

// ---------------------------------------------------------------------------------
// The node tree

/// One FBX element: a name, typed values, and child elements.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Node {
    pub name: &'static str,
    pub props: Vec<Prop>,
    pub children: Vec<Node>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Prop {
    I32(i32),
    I64(i64),
    F64(f64),
    /// Object names are written `name\0\x01Class` (binary) or `Class::name` (ASCII).
    Str(String),
    Raw(Vec<u8>),
    F32s(Vec<f32>),
    I32s(Vec<i32>),
    I64s(Vec<i64>),
    /// A float array that FBX tools write as raw bit patterns in ASCII files.
    F32Bits(Vec<u32>),
}

fn node(name: &'static str, props: Vec<Prop>, children: Vec<Node>) -> Node {
    Node { name, props, children }
}

fn leaf(name: &'static str, props: Vec<Prop>) -> Node {
    node(name, props, Vec::new())
}

fn s(text: &str) -> Prop {
    Prop::Str(text.to_owned())
}

/// `name\0\x01Class`, FBX's binary spelling of an object name.
fn object_name(name: &str, class: &str) -> Prop {
    Prop::Str(format!("{name}\0\x01{class}"))
}

/// A `P:` entry inside `Properties70`.
fn property(name: &str, kind: &str, label: &str, flags: &str, values: Vec<Prop>) -> Node {
    let mut props = vec![s(name), s(kind), s(label), s(flags)];
    props.extend(values);
    leaf("P", props)
}

fn ktime(frame: i32, fps: f32) -> i64 {
    (frame as f64 / fps.max(1e-3) as f64 * KTIME_PER_SECOND).round() as i64
}

/// FBX's `TimeMode` for a frame rate (14 = custom, using `CustomFrameRate`).
fn time_mode(fps: f32) -> i32 {
    const MODES: [(f32, i32); 11] =
        [(120.0, 1), (100.0, 2), (60.0, 3), (50.0, 4), (48.0, 5), (30.0, 6), (25.0, 10), (24.0, 11), (96.0, 15), (72.0, 16), (1000.0, 12)];
    MODES.iter().find(|(rate, _)| (rate - fps).abs() < 1e-4).map_or(14, |m| m.1)
}

/// FBX rotation order and per-axis values for angles in glam's intrinsic `order`.
/// glam's intrinsic "ABC" (R_A·R_B·R_C) is FBX's "CBA" (apply C first).
fn fbx_rotation(order: EulerRot, angles: [f32; 3]) -> (i32, [f64; 3]) {
    let letters = ORDERS.iter().find(|o| o.0 == order).map_or("XYZ", |o| o.1);
    let reversed: String = letters.chars().rev().collect();
    let enum_value = ["XYZ", "XZY", "YZX", "YXZ", "ZXY", "ZYX"].iter().position(|o| *o == reversed).unwrap_or(0) as i32;
    let mut xyz = [0.0; 3];
    for (letter, angle) in letters.chars().zip(angles) {
        xyz[(letter as u8 - b'X') as usize] = angle as f64;
    }
    (enum_value, xyz)
}

/// The whole scene: header, settings, definitions, objects, connections and take.
pub(crate) fn scene(skeleton: &SkeletonRef, motion: &Motion, units: Units) -> Vec<Node> {
    let curves = joint_curves(skeleton, motion);
    let names = export_names(skeleton);
    let scale = units.scale() as f64;
    let fps = motion.fps;
    let first = motion.start;
    let last = motion.start + motion.frames.len().max(1) as i32 - 1;
    let times: Vec<i64> = (first..=last).map(|f| ktime(f, fps)).collect();
    let (start_time, stop_time) = (ktime(first, fps), ktime(last, fps));

    let mut next_id = 1_000_000i64;
    let mut id = || {
        next_id += 1;
        next_id
    };
    let mut objects = Vec::new();
    let mut connections = Vec::new();
    let connect = |connections: &mut Vec<Node>, child: i64, parent: i64, property: Option<&str>| {
        let mut props = vec![s(if property.is_some() { "OP" } else { "OO" }), Prop::I64(child), Prop::I64(parent)];
        props.extend(property.map(s));
        connections.push(leaf("C", props));
    };

    let stack = id();
    let layer = id();
    objects.push(node(
        "AnimationStack",
        vec![Prop::I64(stack), object_name(TAKE, "AnimStack"), s("")],
        vec![node(
            "Properties70",
            vec![],
            vec![
                property("LocalStart", "KTime", "Time", "", vec![Prop::I64(start_time)]),
                property("LocalStop", "KTime", "Time", "", vec![Prop::I64(stop_time)]),
                property("ReferenceStart", "KTime", "Time", "", vec![Prop::I64(start_time)]),
                property("ReferenceStop", "KTime", "Time", "", vec![Prop::I64(stop_time)]),
            ],
        )],
    ));
    objects.push(leaf("AnimationLayer", vec![Prop::I64(layer), object_name("BaseLayer", "AnimLayer"), s("")]));
    connect(&mut connections, layer, stack, None);

    let mut models = Vec::new();
    let (mut curve_nodes, mut curve_count) = (0, 0);
    for (j, joint) in skeleton.joints.iter().enumerate() {
        let c = &curves[j];
        let (attribute, model) = (id(), id());
        models.push(model);
        let rest = Quat::from_array(joint.rest_rotation).normalize();
        let (order, rest_rotation) = fbx_rotation(c.order, euler_near(rest, c.order, c.rotation[0]));
        let offset = joint.offset.map(|v| v as f64 * scale);
        objects.push(node(
            "NodeAttribute",
            vec![Prop::I64(attribute), object_name(&names[j], "NodeAttribute"), s("LimbNode")],
            vec![
                node("Properties70", vec![], vec![property("Size", "double", "Number", "", vec![Prop::F64(scale * 0.05)])]),
                leaf("TypeFlags", vec![s("Skeleton")]),
            ],
        ));
        objects.push(node(
            "Model",
            vec![Prop::I64(model), object_name(&names[j], "Model"), s("LimbNode")],
            vec![
                leaf("Version", vec![Prop::I32(232)]),
                node(
                    "Properties70",
                    vec![],
                    vec![
                        property("RotationActive", "bool", "", "", vec![Prop::I32(1)]),
                        property("RotationOrder", "enum", "", "", vec![Prop::I32(order)]),
                        property("InheritType", "enum", "", "", vec![Prop::I32(1)]),
                        property("DefaultAttributeIndex", "int", "Integer", "", vec![Prop::I32(0)]),
                        property("Lcl Translation", "Lcl Translation", "", "A", offset.map(Prop::F64).to_vec()),
                        property("Lcl Rotation", "Lcl Rotation", "", "A", rest_rotation.map(Prop::F64).to_vec()),
                        property("Lcl Scaling", "Lcl Scaling", "", "A", vec![Prop::F64(1.0); 3]),
                    ],
                ),
                leaf("Culling", vec![s("CullingOff")]),
            ],
        ));
        connect(&mut connections, attribute, model, None);
        let parent = joint.parent.filter(|&p| p < j).map_or(0, |p| models[p]);
        connect(&mut connections, model, parent, None);

        // Curves: rotation always, translation where the joint moves.
        let rotation: Vec<[f64; 3]> = c.rotation.iter().map(|r| fbx_rotation(c.order, *r).1).collect();
        let translation: Option<Vec<[f64; 3]>> =
            c.translation.as_ref().map(|t| t.iter().map(|v| v.to_array().map(|x| x as f64 * scale)).collect());
        for (label, property_name, values) in [("R", "Lcl Rotation", Some(rotation)), ("T", "Lcl Translation", translation)] {
            let Some(values) = values else { continue };
            let curve_node = id();
            curve_nodes += 1;
            let defaults: Vec<Node> = ["d|X", "d|Y", "d|Z"]
                .iter()
                .enumerate()
                .map(|(axis, d)| property(d, "Number", "", "A", vec![Prop::F64(values[0][axis])]))
                .collect();
            objects.push(node(
                "AnimationCurveNode",
                vec![Prop::I64(curve_node), object_name(label, "AnimCurveNode"), s("")],
                vec![node("Properties70", vec![], defaults)],
            ));
            connect(&mut connections, curve_node, layer, None);
            connect(&mut connections, curve_node, model, Some(property_name));
            for (axis, channel) in ["d|X", "d|Y", "d|Z"].into_iter().enumerate() {
                let curve = id();
                curve_count += 1;
                let keys: Vec<f32> = values.iter().map(|v| v[axis] as f32).collect();
                objects.push(node(
                    "AnimationCurve",
                    vec![Prop::I64(curve), object_name("", "AnimCurve"), s("")],
                    vec![
                        leaf("Default", vec![Prop::F64(values[0][axis])]),
                        leaf("KeyVer", vec![Prop::I32(4009)]),
                        leaf("KeyTime", vec![Prop::I64s(times.clone())]),
                        leaf("KeyValueFloat", vec![Prop::F32s(keys)]),
                        // Linear keys with automatic tangents, as other exporters write them.
                        leaf("KeyAttrFlags", vec![Prop::I32s(vec![24836])]),
                        leaf("KeyAttrDataFloat", vec![Prop::F32Bits(vec![0, 0, 255_790_911, 0])]),
                        leaf("KeyAttrRefCount", vec![Prop::I32s(vec![times.len() as i32])]),
                    ],
                ));
                connect(&mut connections, curve, curve_node, Some(channel));
            }
        }
    }

    let joints = skeleton.joints.len() as i32;
    let definition = |kind: &str, count: i32| node("ObjectType", vec![s(kind)], vec![leaf("Count", vec![Prop::I32(count)])]);
    let definitions = vec![
        ("GlobalSettings", 1),
        ("NodeAttribute", joints),
        ("Model", joints),
        ("AnimationStack", 1),
        ("AnimationLayer", 1),
        ("AnimationCurveNode", curve_nodes),
        ("AnimationCurve", curve_count),
    ];
    let total: i32 = definitions.iter().map(|d| d.1).sum();
    let mut definition_nodes = vec![leaf("Version", vec![Prop::I32(100)]), leaf("Count", vec![Prop::I32(total)])];
    definition_nodes.extend(definitions.iter().map(|(kind, count)| definition(kind, *count)));

    let creator = format!("NeuroMotion {}", env!("CARGO_PKG_VERSION"));
    let int = |v: i32| vec![Prop::I32(v)];
    let stamp = [("Version", 1000), ("Year", 1970), ("Month", 1), ("Day", 1), ("Hour", 10), ("Minute", 0), ("Second", 0), ("Millisecond", 0)];
    vec![
        node(
            "FBXHeaderExtension",
            vec![],
            vec![
                leaf("FBXHeaderVersion", int(1003)),
                leaf("FBXVersion", int(VERSION as i32)),
                leaf("EncryptionType", int(0)),
                node("CreationTimeStamp", vec![], stamp.iter().map(|(name, v)| leaf(name, int(*v))).collect()),
                leaf("Creator", vec![s(&creator)]),
            ],
        ),
        leaf("FileId", vec![Prop::Raw(FILE_ID.to_vec())]),
        leaf("CreationTime", vec![s(CREATION_TIME)]),
        leaf("Creator", vec![s(&creator)]),
        node(
            "GlobalSettings",
            vec![],
            vec![
                leaf("Version", int(1000)),
                node(
                    "Properties70",
                    vec![],
                    vec![
                        property("UpAxis", "int", "Integer", "", int(1)),
                        property("UpAxisSign", "int", "Integer", "", int(1)),
                        property("FrontAxis", "int", "Integer", "", int(2)),
                        property("FrontAxisSign", "int", "Integer", "", int(1)),
                        property("CoordAxis", "int", "Integer", "", int(0)),
                        property("CoordAxisSign", "int", "Integer", "", int(1)),
                        property("OriginalUpAxis", "int", "Integer", "", int(1)),
                        property("OriginalUpAxisSign", "int", "Integer", "", int(1)),
                        // Centimetres per file unit.
                        property("UnitScaleFactor", "double", "Number", "", vec![Prop::F64(100.0 / scale)]),
                        property("OriginalUnitScaleFactor", "double", "Number", "", vec![Prop::F64(100.0 / scale)]),
                        property("TimeMode", "enum", "", "", int(time_mode(fps))),
                        property("TimeSpanStart", "KTime", "Time", "", vec![Prop::I64(start_time)]),
                        property("TimeSpanStop", "KTime", "Time", "", vec![Prop::I64(stop_time)]),
                        property("CustomFrameRate", "double", "Number", "", vec![Prop::F64(fps as f64)]),
                    ],
                ),
            ],
        ),
        node(
            "Documents",
            vec![],
            vec![
                leaf("Count", int(1)),
                node(
                    "Document",
                    vec![Prop::I64(id()), s("Scene"), s("Scene")],
                    vec![
                        node(
                            "Properties70",
                            vec![],
                            vec![
                                property("SourceObject", "object", "", "", vec![]),
                                property("ActiveAnimStackName", "KString", "", "", vec![s(TAKE)]),
                            ],
                        ),
                        leaf("RootNode", vec![Prop::I64(0)]),
                    ],
                ),
            ],
        ),
        leaf("References", vec![]),
        node("Definitions", vec![], definition_nodes),
        node("Objects", vec![], objects),
        node("Connections", vec![], connections),
        node(
            "Takes",
            vec![],
            vec![
                leaf("Current", vec![s(TAKE)]),
                node(
                    "Take",
                    vec![s(TAKE)],
                    vec![
                        leaf("FileName", vec![s("Take_001.tak")]),
                        leaf("LocalTime", vec![Prop::I64(start_time), Prop::I64(stop_time)]),
                        leaf("ReferenceTime", vec![Prop::I64(start_time), Prop::I64(stop_time)]),
                    ],
                ),
            ],
        ),
    ]
}

/// Elements that always end with a null record, even when empty (as FBX's SDK writes them).
const ALWAYS_BLOCK: [&str; 2] = ["AnimationStack", "AnimationLayer"];

// ---------------------------------------------------------------------------------
// Binary

fn binary(nodes: &[Node]) -> Vec<u8> {
    let mut out = b"Kaydara FBX Binary  \0\x1a\0".to_vec();
    out.extend(VERSION.to_le_bytes());
    write_children(&mut out, nodes, true);
    out.extend(FOOTER_ID);
    out.extend([0; 4]);
    let padding = match out.len() % 16 {
        0 => 16,
        r => 16 - r,
    };
    out.extend(std::iter::repeat_n(0, padding));
    out.extend(VERSION.to_le_bytes());
    out.extend([0; 120]);
    out.extend(FOOTER_MAGIC);
    out
}

/// Writes a list of elements and, when `block` says the list needs one, the 13-byte
/// null record that closes it.
fn write_children(out: &mut Vec<u8>, nodes: &[Node], block: bool) {
    for (i, node) in nodes.iter().enumerate() {
        write_node(out, node, i + 1 == nodes.len());
    }
    if block {
        out.extend([0; 13]);
    }
}

fn write_node(out: &mut Vec<u8>, node: &Node, is_last: bool) {
    let start = out.len();
    out.extend([0; 12]); // end offset, property count, property bytes: filled in below
    out.push(node.name.len() as u8);
    out.extend(node.name.as_bytes());
    let props_start = out.len();
    for prop in &node.props {
        write_prop(out, prop);
    }
    let props_len = (out.len() - props_start) as u32;
    let block = !node.children.is_empty()
        || (node.props.is_empty() && !is_last)
        || ALWAYS_BLOCK.contains(&node.name);
    write_children(out, &node.children, block);
    let end = out.len() as u32;
    out[start..start + 4].copy_from_slice(&end.to_le_bytes());
    out[start + 4..start + 8].copy_from_slice(&(node.props.len() as u32).to_le_bytes());
    out[start + 8..start + 12].copy_from_slice(&props_len.to_le_bytes());
}

fn write_prop(out: &mut Vec<u8>, prop: &Prop) {
    fn array<T>(out: &mut Vec<u8>, kind: u8, values: &[T], bytes: impl Fn(&T) -> Vec<u8>) {
        out.push(kind);
        let data: Vec<u8> = values.iter().flat_map(bytes).collect();
        out.extend((values.len() as u32).to_le_bytes());
        out.extend(0u32.to_le_bytes()); // not compressed
        out.extend((data.len() as u32).to_le_bytes());
        out.extend(data);
    }
    match prop {
        Prop::I32(v) => {
            out.push(b'I');
            out.extend(v.to_le_bytes());
        }
        Prop::I64(v) => {
            out.push(b'L');
            out.extend(v.to_le_bytes());
        }
        Prop::F64(v) => {
            out.push(b'D');
            out.extend(v.to_le_bytes());
        }
        Prop::Str(text) => {
            out.push(b'S');
            out.extend((text.len() as u32).to_le_bytes());
            out.extend(text.as_bytes());
        }
        Prop::Raw(bytes) => {
            out.push(b'R');
            out.extend((bytes.len() as u32).to_le_bytes());
            out.extend(bytes);
        }
        Prop::F32s(v) => array(out, b'f', v, |x| x.to_le_bytes().to_vec()),
        Prop::F32Bits(v) => array(out, b'f', v, |x| x.to_le_bytes().to_vec()),
        Prop::I32s(v) => array(out, b'i', v, |x| x.to_le_bytes().to_vec()),
        Prop::I64s(v) => array(out, b'l', v, |x| x.to_le_bytes().to_vec()),
    }
}

// ---------------------------------------------------------------------------------
// ASCII

fn ascii(nodes: &[Node]) -> String {
    let mut out = String::from("; FBX 7.4.0 project file\n; Created by NeuroMotion\n; ----------------------------------------------------\n\n");
    for node in nodes.iter().filter(|n| !n.props.iter().any(|p| matches!(p, Prop::Raw(_)))) {
        write_ascii(&mut out, node, 0);
        out.push('\n');
    }
    out
}

fn write_ascii(out: &mut String, node: &Node, depth: usize) {
    let indent = "\t".repeat(depth);
    let _ = write!(out, "{indent}{}: ", node.name);
    // A single array value is written as its own block.
    if let [prop] = node.props.as_slice() {
        if let Some((count, values)) = ascii_array(prop) {
            let _ = writeln!(out, "*{count} {{\n{indent}\ta: {values}\n{indent}}}");
            return;
        }
    }
    let props: Vec<String> = node.props.iter().map(ascii_value).collect();
    out.push_str(&props.join(", "));
    let block = !node.children.is_empty() || node.props.is_empty() || ALWAYS_BLOCK.contains(&node.name);
    if block {
        out.push_str(" {\n");
        for child in &node.children {
            write_ascii(out, child, depth + 1);
        }
        let _ = write!(out, "{indent}}}");
    }
    out.push('\n');
}

fn ascii_value(prop: &Prop) -> String {
    match prop {
        Prop::I32(v) => v.to_string(),
        Prop::I64(v) => v.to_string(),
        Prop::F64(v) => format_f64(*v),
        Prop::Str(text) => {
            let text = match text.split_once("\0\x01") {
                Some((name, class)) => format!("{class}::{name}"),
                None => text.clone(),
            };
            format!("\"{}\"", text.replace('"', "&quot;"))
        }
        Prop::Raw(_) | Prop::F32s(_) | Prop::F32Bits(_) | Prop::I32s(_) | Prop::I64s(_) => {
            ascii_array(prop).map_or_else(String::new, |(count, values)| format!("*{count} {{ a: {values} }}"))
        }
    }
}

fn ascii_array(prop: &Prop) -> Option<(usize, String)> {
    fn join<T>(values: &[T], f: impl Fn(&T) -> String) -> (usize, String) {
        (values.len(), values.iter().map(f).collect::<Vec<_>>().join(","))
    }
    Some(match prop {
        Prop::F32s(v) => join(v, |x| format_f64(*x as f64)),
        Prop::F32Bits(v) => join(v, u32::to_string),
        Prop::I32s(v) => join(v, i32::to_string),
        Prop::I64s(v) => join(v, i64::to_string),
        _ => return None,
    })
}

/// Shortest text that reads back as the same value, always with a decimal point.
fn format_f64(v: f64) -> String {
    let text = format!("{v}");
    if text.contains(['.', 'e', 'E', 'N', 'i']) { text } else { format!("{text}.0") }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use glam::{Quat, Vec3};

    use super::*;
    use crate::document::Project;
    use crate::document::sample::{physics_demo_project, placeholder_project};
    use crate::io::bvh::tests::rotated_rest_project;
    use crate::io::motion::{sample, simulate};

    /// An element read back from a file, with values reduced to a few plain kinds.
    #[derive(Clone, Debug, PartialEq)]
    struct Parsed {
        name: String,
        values: Vec<Value>,
        children: Vec<Parsed>,
    }

    #[derive(Clone, Debug, PartialEq)]
    enum Value {
        Int(i64),
        Float(f64),
        Text(String),
        Bytes(Vec<u8>),
        Ints(Vec<i64>),
        Floats(Vec<f64>),
    }

    impl Parsed {
        fn child(&self, name: &str) -> &Parsed {
            self.children.iter().find(|c| c.name == name).unwrap_or_else(|| panic!("no {name} in {}", self.name))
        }

        fn all<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Parsed> {
            self.children.iter().filter(move |c| c.name == name)
        }

        fn int(&self, i: usize) -> i64 {
            match &self.values[i] {
                Value::Int(v) => *v,
                other => panic!("{} value {i} is {other:?}", self.name),
            }
        }

        fn text(&self, i: usize) -> &str {
            match &self.values[i] {
                Value::Text(v) => v,
                other => panic!("{} value {i} is {other:?}", self.name),
            }
        }

        fn floats(&self) -> Vec<f64> {
            match &self.values[0] {
                Value::Floats(v) => v.clone(),
                other => panic!("{} isn't a float array: {other:?}", self.name),
            }
        }

        /// A `Properties70` entry's values (after its four descriptive strings).
        fn property(&self, name: &str) -> Option<&[Value]> {
            let props = self.children.iter().find(|c| c.name == "Properties70")?;
            props.all("P").find(|p| p.text(0) == name).map(|p| &p.values[4..])
        }
    }

    fn number(v: &Value) -> f64 {
        match v {
            Value::Int(i) => *i as f64,
            Value::Float(f) => *f,
            other => panic!("not a number: {other:?}"),
        }
    }

    /// A binary FBX reader, checking every offset and the footer as it goes.
    fn parse_binary(bytes: &[u8]) -> Vec<Parsed> {
        assert_eq!(&bytes[..23], b"Kaydara FBX Binary  \0\x1a\0");
        assert_eq!(u32::from_le_bytes(bytes[23..27].try_into().unwrap()), 7400);
        let mut pos = 27;
        let nodes = read_list(bytes, &mut pos);
        assert_eq!(&bytes[pos..pos + 16], FOOTER_ID);
        assert_eq!(&bytes[bytes.len() - 16..], FOOTER_MAGIC);
        assert_eq!((bytes.len() - 16 - 120 - 4) % 16, 0, "footer padding");
        nodes
    }

    fn u32_at(bytes: &[u8], pos: usize) -> u32 {
        u32::from_le_bytes(bytes[pos..pos + 4].try_into().unwrap())
    }

    /// Reads elements until a null record.
    fn read_list(bytes: &[u8], pos: &mut usize) -> Vec<Parsed> {
        let mut nodes = Vec::new();
        loop {
            let end = u32_at(bytes, *pos) as usize;
            if end == 0 {
                assert!(bytes[*pos..*pos + 13].iter().all(|b| *b == 0), "malformed null record");
                *pos += 13;
                return nodes;
            }
            let count = u32_at(bytes, *pos + 4) as usize;
            let props_len = u32_at(bytes, *pos + 8) as usize;
            let name_len = bytes[*pos + 12] as usize;
            let name = String::from_utf8(bytes[*pos + 13..*pos + 13 + name_len].to_vec()).unwrap();
            *pos += 13 + name_len;
            let props_end = *pos + props_len;
            let values: Vec<Value> = (0..count).map(|_| read_value(bytes, pos)).collect();
            assert_eq!(*pos, props_end, "{name}: property bytes don't add up");
            let children = if *pos < end { read_list(bytes, pos) } else { Vec::new() };
            assert_eq!(*pos, end, "{name}: end offset is wrong");
            nodes.push(Parsed { name, values, children });
            // The last element of a list may end without a null record after it.
            if bytes[*pos..].len() >= 13 && u32_at(bytes, *pos) == 0 {
                *pos += 13;
                return nodes;
            }
        }
    }

    fn read_value(bytes: &[u8], pos: &mut usize) -> Value {
        let kind = bytes[*pos];
        *pos += 1;
        let take = |pos: &mut usize, n: usize| {
            let slice = &bytes[*pos..*pos + n];
            *pos += n;
            slice
        };
        match kind {
            b'I' => Value::Int(i32::from_le_bytes(take(pos, 4).try_into().unwrap()) as i64),
            b'L' => Value::Int(i64::from_le_bytes(take(pos, 8).try_into().unwrap())),
            b'D' => Value::Float(f64::from_le_bytes(take(pos, 8).try_into().unwrap())),
            b'S' | b'R' => {
                let len = u32_at(bytes, *pos) as usize;
                *pos += 4;
                let raw = take(pos, len).to_vec();
                if kind == b'R' {
                    return Value::Bytes(raw);
                }
                let text = String::from_utf8(raw).unwrap();
                Value::Text(match text.split_once("\0\x01") {
                    Some((name, class)) => format!("{class}::{name}"),
                    None => text,
                })
            }
            b'f' | b'i' | b'l' => {
                let (count, encoding, len) = (u32_at(bytes, *pos) as usize, u32_at(bytes, *pos + 4), u32_at(bytes, *pos + 8) as usize);
                *pos += 12;
                assert_eq!(encoding, 0);
                let data = take(pos, len);
                let size = if kind == b'l' { 8 } else { 4 };
                assert_eq!(len, count * size);
                let chunks = data.chunks(size);
                match kind {
                    b'f' => Value::Floats(chunks.map(|c| f32::from_le_bytes(c.try_into().unwrap()) as f64).collect()),
                    b'i' => Value::Ints(chunks.map(|c| i32::from_le_bytes(c.try_into().unwrap()) as i64).collect()),
                    _ => Value::Ints(chunks.map(|c| i64::from_le_bytes(c.try_into().unwrap())).collect()),
                }
            }
            other => panic!("unexpected property type {}", other as char),
        }
    }

    /// An ASCII FBX reader.
    fn parse_ascii(text: &str) -> Vec<Parsed> {
        #[derive(Debug, PartialEq)]
        enum Token {
            Name(String),
            Text(String),
            Number(String),
            Count,
            Open,
            Close,
            Comma,
        }
        let mut tokens = Vec::new();
        let mut chars = text.chars().peekable();
        while let Some(&c) = chars.peek() {
            match c {
                _ if c.is_whitespace() => {
                    chars.next();
                }
                ';' => {
                    while chars.next().is_some_and(|c| c != '\n') {}
                }
                '"' => {
                    chars.next();
                    let text: String = chars.by_ref().take_while(|&c| c != '"').collect();
                    tokens.push(Token::Text(text.replace("&quot;", "\"")));
                }
                '{' | '}' | ',' => {
                    chars.next();
                    tokens.push(match c {
                        '{' => Token::Open,
                        '}' => Token::Close,
                        _ => Token::Comma,
                    });
                }
                '*' => {
                    chars.next();
                    while chars.peek().is_some_and(char::is_ascii_digit) {
                        chars.next();
                    }
                    tokens.push(Token::Count);
                }
                _ => {
                    let mut word = String::new();
                    while let Some(&c) = chars.peek().filter(|c| !c.is_whitespace() && !matches!(c, ',' | '{' | '}')) {
                        word.push(c);
                        chars.next();
                    }
                    tokens.push(match word.strip_suffix(':') {
                        Some(name) => Token::Name(name.to_owned()),
                        None => Token::Number(word),
                    });
                }
            }
        }

        fn scalar(token: &Token) -> Option<Value> {
            match token {
                Token::Text(t) => Some(Value::Text(t.clone())),
                Token::Number(n) if n.contains('.') => Some(Value::Float(n.parse().unwrap())),
                Token::Number(n) => Some(Value::Int(n.parse().unwrap())),
                _ => None,
            }
        }
        fn list(tokens: &[Token], i: &mut usize) -> Vec<Parsed> {
            let mut nodes = Vec::new();
            while let Some(Token::Name(name)) = tokens.get(*i) {
                *i += 1;
                let mut values = Vec::new();
                let mut children = Vec::new();
                if tokens.get(*i) == Some(&Token::Count) {
                    // *N { a: v,v,v }
                    assert_eq!(tokens[*i + 1], Token::Open);
                    assert_eq!(tokens[*i + 2], Token::Name("a".into()));
                    *i += 3;
                    let mut items = Vec::new();
                    while let Some(v) = tokens.get(*i).and_then(scalar) {
                        items.push(v);
                        *i += 1;
                        if tokens.get(*i) == Some(&Token::Comma) {
                            *i += 1;
                        }
                    }
                    assert_eq!(tokens[*i], Token::Close);
                    *i += 1;
                    let floats = items.iter().any(|v| matches!(v, Value::Float(_)));
                    values.push(if floats {
                        Value::Floats(items.iter().map(number).collect())
                    } else {
                        Value::Ints(items.iter().map(|v| number(v) as i64).collect())
                    });
                } else {
                    while let Some(v) = tokens.get(*i).and_then(scalar) {
                        values.push(v);
                        *i += 1;
                        if tokens.get(*i) == Some(&Token::Comma) {
                            *i += 1;
                        }
                    }
                    if tokens.get(*i) == Some(&Token::Open) {
                        *i += 1;
                        children = list(tokens, i);
                        assert_eq!(tokens[*i], Token::Close, "unclosed {name}");
                        *i += 1;
                    }
                }
                nodes.push(Parsed { name: name.clone(), values, children });
            }
            nodes
        }
        let mut i = 0;
        let nodes = list(&tokens, &mut i);
        assert_eq!(i, tokens.len(), "trailing tokens: {:?}", &tokens[i..]);
        nodes
    }

    fn export(project: &Project, units: Units, encoding: Encoding) -> (Motion, Vec<u8>) {
        let motion = sample(project, 0, simulate(project).as_ref());
        let bytes = to_bytes(&project.characters[0].skeleton, &motion, units, encoding);
        (motion, bytes)
    }

    fn top<'a>(nodes: &'a [Parsed], name: &str) -> &'a Parsed {
        nodes.iter().find(|n| n.name == name).unwrap_or_else(|| panic!("no {name}"))
    }

    /// Rebuilds every joint's world position at every frame from the models, connections
    /// and curves, the way an importer would, in file units.
    fn evaluate(nodes: &[Parsed], frames: usize) -> HashMap<String, Vec<Vec3>> {
        let objects = top(nodes, "Objects");
        let connections: Vec<&Parsed> = top(nodes, "Connections").all("C").collect();
        let parent_of = |child: i64| connections.iter().find(|c| c.text(0) == "OO" && c.int(1) == child).map(|c| c.int(2));
        let models: Vec<&Parsed> = objects.all("Model").collect();
        let vec3 = |model: &Parsed, name: &str| {
            let v = model.property(name).unwrap();
            [number(&v[0]), number(&v[1]), number(&v[2])]
        };
        // Per model and property, the X/Y/Z key values.
        let curves_of = |model: i64, property: &str| -> Option<[Vec<f64>; 3]> {
            let node = connections.iter().find(|c| c.text(0) == "OP" && c.int(2) == model && c.text(3) == property)?.int(1);
            let curve = |axis: &str| {
                let id = connections.iter().find(|c| c.text(0) == "OP" && c.int(2) == node && c.text(3) == axis).unwrap().int(1);
                objects.all("AnimationCurve").find(|c| c.int(0) == id).unwrap().child("KeyValueFloat").floats()
            };
            Some([curve("d|X"), curve("d|Y"), curve("d|Z")])
        };
        let mut world: HashMap<i64, Vec<(Quat, Vec3)>> = HashMap::new();
        let mut out = HashMap::new();
        // Parents are written before children.
        for model in &models {
            let id = model.int(0);
            let order = ["XYZ", "XZY", "YZX", "YXZ", "ZXY", "ZYX"][model.property("RotationOrder").map_or(0, |v| number(&v[0]) as usize)];
            let (rotation_curves, translation_curves) = (curves_of(id, "Lcl Rotation"), curves_of(id, "Lcl Translation"));
            let (static_rotation, static_translation) = (vec3(model, "Lcl Rotation"), vec3(model, "Lcl Translation"));
            let parent = parent_of(id).filter(|p| *p != 0);
            let transforms: Vec<(Quat, Vec3)> = (0..frames)
                .map(|f| {
                    let pick = |curves: &Option<[Vec<f64>; 3]>, fixed: [f64; 3]| match curves {
                        Some(c) => [c[0][f], c[1][f], c[2][f]],
                        None => fixed,
                    };
                    let r = pick(&rotation_curves, static_rotation);
                    let t = pick(&translation_curves, static_translation);
                    // FBX order "ABC" applies A first: R = R_C · R_B · R_A.
                    let rotation = order.chars().fold(Quat::IDENTITY, |q, axis| {
                        let i = (axis as u8 - b'X') as usize;
                        Quat::from_axis_angle(Vec3::AXES[i], (r[i] as f32).to_radians()) * q
                    });
                    let local = Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32);
                    match parent {
                        Some(p) => {
                            let (pr, pp) = world[&p][f];
                            (pr * rotation, pp + pr * local)
                        }
                        None => (rotation, local),
                    }
                })
                .collect();
            let name = model.text(1).trim_start_matches("Model::").to_owned();
            out.insert(name, transforms.iter().map(|t| t.1).collect());
            world.insert(id, transforms);
        }
        out
    }

    fn assert_round_trip(project: &Project, tolerance: f32) {
        let (motion, bytes) = export(project, Units::Metres, Encoding::Binary);
        let nodes = parse_binary(&bytes);
        let skeleton = &project.characters[0].skeleton;
        let names = export_names(skeleton);
        let positions = evaluate(&nodes, motion.frames.len());
        assert_eq!(positions.len(), skeleton.joints.len());
        let mut worst = 0.0f32;
        for (frame, joints) in motion.frames.iter().enumerate() {
            for (j, (_, expected)) in joints.iter().enumerate() {
                let error = positions[&names[j]][frame].distance(*expected);
                worst = worst.max(error);
                assert!(error < tolerance, "{} is {error} m off at frame {}", names[j], motion.start + frame as i32);
            }
        }
        eprintln!("worst joint error {worst} m");
    }

    #[test]
    fn binary_file_is_well_formed() {
        let project = placeholder_project();
        let (motion, bytes) = export(&project, Units::Metres, Encoding::Binary);
        let nodes = parse_binary(&bytes);
        let names: Vec<&str> = nodes.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(
            names,
            ["FBXHeaderExtension", "FileId", "CreationTime", "Creator", "GlobalSettings", "Documents", "References", "Definitions", "Objects", "Connections", "Takes"]
        );
        let settings = top(&nodes, "GlobalSettings");
        assert_eq!(settings.property("UpAxis").map(|v| number(&v[0])), Some(1.0));
        assert_eq!(settings.property("UnitScaleFactor").map(|v| number(&v[0])), Some(100.0));
        assert_eq!(settings.property("TimeMode").map(|v| number(&v[0])), Some(6.0), "30 fps");

        let skeleton = &project.characters[0].skeleton;
        let objects = top(&nodes, "Objects");
        let models: Vec<&Parsed> = objects.all("Model").collect();
        assert_eq!(models.len(), skeleton.joints.len());
        assert!(models.iter().all(|m| m.text(2) == "LimbNode"));
        assert_eq!(models[0].text(1), "Model::hips");
        assert_eq!(objects.all("NodeAttribute").count(), skeleton.joints.len());
        assert_eq!(objects.all("AnimationStack").count(), 1);

        // The hierarchy mirrors the skeleton.
        let connections: Vec<&Parsed> = top(&nodes, "Connections").all("C").collect();
        let id_of = |name: &str| models.iter().find(|m| m.text(1) == format!("Model::{name}")).unwrap().int(0);
        for joint in &skeleton.joints {
            let parent = connections.iter().find(|c| c.text(0) == "OO" && c.int(1) == id_of(&joint.name)).unwrap().int(2);
            assert_eq!(parent, joint.parent.map_or(0, |p| id_of(&skeleton.joints[p].name)), "{}", joint.name);
        }
        // One key per frame, at the frame's time.
        let curves: Vec<&Parsed> = objects.all("AnimationCurve").collect();
        let joints = skeleton.joints.len();
        assert_eq!(curves.len(), joints * 3 + 3, "rotation for every joint, translation for the root");
        let times = match &curves[0].child("KeyTime").values[0] {
            Value::Ints(t) => t.clone(),
            other => panic!("{other:?}"),
        };
        assert_eq!(times.len(), motion.frames.len());
        assert_eq!(times[30], 46_186_158_000, "frame 30 at 30 fps is one second");
    }

    #[test]
    fn ascii_and_binary_say_the_same() {
        let project = rotated_rest_project();
        let binary = parse_binary(&export(&project, Units::Centimetres, Encoding::Binary).1);
        let text = String::from_utf8(export(&project, Units::Centimetres, Encoding::Ascii).1).unwrap();
        assert!(text.starts_with("; FBX 7.4.0 project file"));
        let ascii = parse_ascii(&text);
        // ASCII files carry no file id, and write the curve attribute floats as bit patterns.
        let mut expected: Vec<Parsed> = binary.into_iter().filter(|n| n.name != "FileId").collect();
        fn bits_as_ints(node: &mut Parsed) {
            if node.name == "KeyAttrDataFloat" {
                if let Value::Floats(v) = &node.values[0] {
                    node.values[0] = Value::Ints(v.iter().map(|x| (*x as f32).to_bits() as i64).collect());
                }
            }
            node.children.iter_mut().for_each(bits_as_ints);
        }
        expected.iter_mut().for_each(bits_as_ints);
        assert_eq!(ascii, expected);
        let settings = top(&ascii, "GlobalSettings");
        assert_eq!(settings.property("UnitScaleFactor").map(|v| number(&v[0])), Some(1.0), "centimetres");
        assert_eq!(settings.property("TimeMode").map(|v| number(&v[0])), Some(11.0), "24 fps");
    }

    #[test]
    fn placeholder_animation_round_trips() {
        assert_round_trip(&placeholder_project(), 1e-4);
    }

    #[test]
    fn physics_demo_round_trips() {
        assert_round_trip(&physics_demo_project(), 1e-3);
    }

    #[test]
    fn rotated_rest_frames_round_trip() {
        let project = rotated_rest_project();
        assert_round_trip(&project, 1e-4);
        // The static transforms are the rest pose, so importers show it as such:
        // evaluate the file with its curves disconnected.
        let (_, bytes) = export(&project, Units::Metres, Encoding::Binary);
        let nodes = parse_binary(&bytes);
        let skeleton = &project.characters[0].skeleton;
        let expected = crate::document::world_transforms(skeleton, &crate::document::Pose::rest(skeleton.joints.len()));
        let without_curves: Vec<Parsed> = nodes
            .iter()
            .map(|n| {
                let mut n = n.clone();
                if n.name == "Connections" {
                    n.children.retain(|c| c.text(0) == "OO");
                }
                n
            })
            .collect();
        let rest = evaluate(&without_curves, 1);
        let names = export_names(skeleton);
        for (j, (_, position)) in expected.iter().enumerate() {
            assert!(rest[&names[j]][0].distance(*position) < 1e-5, "{} rest position", names[j]);
        }
    }
}
