use std::env;
use std::path::PathBuf;

fn main() {
    //Paths
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let src_dir = manifest_dir.join("src");
    let bridge_rs = PathBuf::from("src/fastlio_bindings.rs");
    let bridge_cxx = src_dir.join("fastlio_bridge.cxx");
    let bridge_hxx = src_dir.join("fastlio_bridge.hxx");
    let include_dir = manifest_dir.join("include");

    let dst = cmake::Config::new(&manifest_dir)
        .profile("Release").build();

    //Link the library (has to be precompiled. make a build folder, then cmake .. and make inside it.)
    println!("cargo:rustc-link-search=native={}", dst.join("lib").display());
    //This line seems to do nothing
    println!("cargo:rustc-link-lib=dylib=fastlio_mapping");
    //The -lfastlio_mapping at the end seems to be required, even though in theory it's identical to the previous line
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dst.join("lib").display());
    //Build bridge
    cxx_build::bridge(bridge_rs.to_str().unwrap())
        .file(bridge_cxx.to_str().unwrap())
        .include(&src_dir) // for fastlio_bridge.hxx
        .include(&include_dir) // for msg.h
        // generated headers live under $OUT_DIR/cxxbridge/include
        .include(out_dir.join("cxxbridge").join("include"))
        .flag_if_supported("-std=c++17")
        .compile("fastlio_bridge");

    //Link bridge
    //Probably the same sort of issue going on here. cxxbridge should handle adding this automatically but it doesn't work.

    //Reruns
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join(&bridge_rs).display()
    );
    println!("cargo:rerun-if-changed={}", bridge_cxx.display());
    println!("cargo:rerun-if-changed={}", bridge_hxx.display());
    println!(
        "cargo:rerun-if-changed={}",
        include_dir.join("msg.h").display()
    );
}
