//! Generates the contract types from `../../contract/`.
//!
//! JSON Schemas: every `schema/*.schema.json` becomes one definition named by
//! its `title`, and its `$defs` become sibling definitions; cross-file `$ref`s
//! are rewritten to those definitions and the set is handed to typify. Only
//! structural keywords reach typify: value constraints (`pattern`, length and
//! range bounds, `if`/`then` rules) are stripped, so the Rust types describe
//! shape and full validation stays with the schema files themselves.
//!
//! Protobuf: the contract `.proto` files plus the vendored Foxglove schemas are
//! compiled in-process with protox (no system `protoc`) and generated with
//! prost. The encoded `FileDescriptorSet` is also written out, because an MCAP
//! protobuf channel carries it as its schema data.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use prost::Message;
use serde_json::{Map, Value};

const STRIPPED: &[&str] = &[
    "pattern",
    "minLength",
    "maxLength",
    "minimum",
    "maximum",
    "exclusiveMinimum",
    "exclusiveMaximum",
    "if",
    "then",
    "else",
    "$schema",
    "$id",
];

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let contract = manifest_dir.join("../../contract");
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    println!(
        "cargo:rerun-if-changed={}",
        contract.join("schema").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        contract.join("proto").display()
    );
    println!("cargo:rerun-if-changed=vendor/foxglove-schemas");
    gen_schema_types(&contract.join("schema"), &out);
    gen_proto_types(
        &contract.join("proto"),
        &manifest_dir.join("vendor/foxglove-schemas"),
        &out,
    );
}

fn schema_files(dir: &Path) -> Vec<PathBuf> {
    let mut files: Vec<PathBuf> = fs::read_dir(dir)
        .expect("contract schema dir")
        .map(|e| e.unwrap().path())
        .filter(|p| p.to_string_lossy().ends_with(".schema.json"))
        .collect();
    files.sort();
    for f in &files {
        println!("cargo:rerun-if-changed={}", f.display());
    }
    files
}

fn gen_schema_types(dir: &Path, out: &Path) {
    let files = schema_files(dir);
    let docs: Vec<(String, Value)> = files
        .iter()
        .map(|p| {
            let name = p.file_name().unwrap().to_string_lossy().into_owned();
            let v: Value = serde_json::from_str(&fs::read_to_string(p).unwrap())
                .unwrap_or_else(|e| panic!("{name}: {e}"));
            (name, v)
        })
        .collect();
    // File name -> definition name (its title).
    let titles: BTreeMap<String, String> = docs
        .iter()
        .map(|(name, v)| {
            let title = v["title"]
                .as_str()
                .unwrap_or_else(|| panic!("{name}: no title"));
            (name.clone(), title.to_string())
        })
        .collect();

    let mut defs: BTreeMap<String, Value> = BTreeMap::new();
    for (name, mut doc) in docs {
        let obj = doc.as_object_mut().unwrap();
        if let Some(Value::Object(local)) = obj.remove("$defs") {
            for (k, v) in local {
                assert!(
                    defs.insert(k.clone(), v).is_none(),
                    "duplicate $defs name {k}"
                );
            }
        }
        let title = titles[&name].clone();
        assert!(
            defs.insert(title.clone(), doc).is_none(),
            "duplicate title {title}"
        );
    }
    let defs: BTreeMap<String, schemars::schema::Schema> = defs
        .into_iter()
        .map(|(k, mut v)| {
            structural(&mut v, &titles);
            let s = serde_json::from_value(v).unwrap_or_else(|e| panic!("{k}: {e}"));
            (k, s)
        })
        .collect();

    let mut settings = typify::TypeSpaceSettings::default();
    settings
        .with_struct_builder(false)
        .with_derive("PartialEq".to_string());
    let mut space = typify::TypeSpace::new(&settings);
    space.add_ref_types(defs).expect("typify");
    let file: syn::File = syn::parse2(space.to_stream()).expect("typify output parses");
    fs::write(out.join("schema_types.rs"), prettyplease::unparse(&file)).unwrap();
}

/// Rewrite `$ref`s to local definitions and strip value constraints. The keys
/// of a name -> schema map (`properties`, `definitions`) are field or type
/// names, never keywords, so a field named like a keyword survives.
fn structural(v: &mut Value, titles: &BTreeMap<String, String>) {
    match v {
        Value::Object(map) => {
            for k in STRIPPED {
                map.remove(*k);
            }
            if let Some(Value::String(r)) = map.get_mut("$ref") {
                *r = rewrite_ref(r, titles);
            }
            drop_empty_all_of(map);
            for (k, child) in map.iter_mut() {
                match (k.as_str(), child) {
                    ("properties" | "definitions" | "$defs", Value::Object(named)) => named
                        .values_mut()
                        .for_each(|schema| structural(schema, titles)),
                    (_, child) => structural(child, titles),
                }
            }
            drop_empty_all_of(map);
        }
        Value::Array(items) => items.iter_mut().for_each(|i| structural(i, titles)),
        _ => {}
    }
}

fn drop_empty_all_of(map: &mut Map<String, Value>) {
    let empty = matches!(map.get("allOf"), Some(Value::Array(items))
        if items.iter().all(|i| i.as_object().is_some_and(|o| o.is_empty())));
    if empty {
        map.remove("allOf");
    }
}

fn rewrite_ref(r: &str, titles: &BTreeMap<String, String>) -> String {
    let (file, frag) = r.split_once('#').unwrap_or((r, ""));
    let def = if let Some(name) = frag.strip_prefix("/$defs/") {
        name.to_string()
    } else if frag.is_empty() {
        titles
            .get(file)
            .unwrap_or_else(|| panic!("unknown $ref {r}"))
            .clone()
    } else {
        panic!("unsupported $ref {r}");
    };
    format!("#/definitions/{def}")
}

fn gen_proto_types(contract_proto: &Path, foxglove: &Path, out: &Path) {
    let mut files = Vec::new();
    for dir in [
        contract_proto.join("ados/capture/v1"),
        foxglove.join("foxglove"),
    ] {
        let mut names: Vec<PathBuf> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().path())
            .filter(|p| p.extension().is_some_and(|e| e == "proto"))
            .collect();
        names.sort();
        for p in names {
            println!("cargo:rerun-if-changed={}", p.display());
            let root = if p.starts_with(contract_proto) {
                contract_proto
            } else {
                foxglove
            };
            files.push(
                p.strip_prefix(root)
                    .unwrap()
                    .to_string_lossy()
                    .replace('\\', "/"),
            );
        }
    }
    let fds = protox::compile(&files, [contract_proto, foxglove]).expect("protox");
    // MCAP schema data: the same set without comments and source positions,
    // since every segment embeds it once per protobuf schema.
    let mut lean = fds.clone();
    for f in &mut lean.file {
        f.source_code_info = None;
    }
    fs::write(out.join("capture_fds.bin"), lean.encode_to_vec()).unwrap();
    prost_build::Config::new()
        .enable_type_names()
        // Contract messages also cross the msgpack Atlas envelope (an IMU
        // window inside a keyframe), so they carry serde as well.
        .type_attribute(
            ".ados.capture.v1",
            "#[derive(::serde::Serialize, ::serde::Deserialize)]",
        )
        .out_dir(out)
        .compile_fds(fds)
        .expect("prost-build");
}
