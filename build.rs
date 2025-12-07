use std::env;
use std::path::PathBuf;

fn main() {
    // Compute paths relative to the crate directory (FAST_LIO/).
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let src_dir = manifest_dir.join("src");
    // Use a relative bridge path so the generated header has a stable include path.
    let bridge_rs = PathBuf::from("src/fastlio_bindings.rs");
    let bridge_cxx = src_dir.join("fastlio_bridge.cxx");
    let bridge_hxx = src_dir.join("fastlio_bridge.hxx");
    let include_dir = manifest_dir.join("include");

    // Link the prebuilt CMake-shared library and libc++.
    println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path/../..");
    let mapping_dylib = manifest_dir.join("libfastlio_mapping.dylib");
    if !mapping_dylib.exists() {
        panic!(
            "libfastlio_mapping.dylib not found at {}. Run CMake/make first.",
            mapping_dylib.display()
        );
    }
    println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path/..");
    println!(
        "cargo:rustc-link-arg=-Wl,-force_load,{}",
        mapping_dylib.display()
    );

    // Build the CXX bridge.
    let mut bridge = cxx_build::bridge(bridge_rs.to_str().unwrap());
    bridge
        .file(bridge_cxx.to_str().unwrap())
        .include(&src_dir)       // for fastlio_bridge.hxx
        .include(&include_dir)   // for msg.h
        // generated headers live under $OUT_DIR/cxxbridge/include
        .include(out_dir.join("cxxbridge").join("include"))
        .flag_if_supported("-std=c++17");
    bridge.compile("fastlio_bridge");

    // Link the generated bridge archive.
    let bridge_archive = out_dir.join("libfastlio_bridge.a");
    if !bridge_archive.exists() {
        panic!("expected bridge archive at {}", bridge_archive.display());
    }
    println!("cargo:rustc-link-search=native={}", out_dir.display());
    println!("cargo:rustc-link-lib=static=fastlio_bridge");
    // force-load so dead_strip doesn’t drop the symbols
    println!("cargo:rustc-link-arg=-Wl,-force_load,{}", bridge_archive.display());

    // Rebuild when any inputs change.
    println!("cargo:rerun-if-changed={}", manifest_dir.join(&bridge_rs).display());
    println!("cargo:rerun-if-changed={}", bridge_cxx.display());
    println!("cargo:rerun-if-changed={}", bridge_hxx.display());
    println!("cargo:rerun-if-changed={}", include_dir.join("msg.h").display());
}
