use std::{env, fs, path::PathBuf, process::Command};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // Mailbox sections in shared SRAM2a (from embassy-stm32-wpan's build.rs).
    println!("cargo:rustc-link-arg-bins=-Ttl_mbox.x");
    // defmt's string table, which probe-rs decodes the RTT log stream with.
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // The build time, in two forms: as a Unix timestamp, which identifies
    // the build when updates are compared, and as readable text, to show
    // which image is running after a reflash.
    let date = |args: &[&str]| {
        let output = Command::new("date").args(args).output().ok()?;
        Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
    };
    let id = date(&["+%s"]).unwrap_or_else(|| "0".into());
    let stamp = date(&["-d", &format!("@{id}"), "+%Y-%m-%d %H:%M:%S"]);
    println!("cargo:rustc-env=BUILD_ID={id}");
    println!(
        "cargo:rustc-env=BUILD_STAMP={}",
        stamp.unwrap_or_else(|| "unknown".into())
    );
}
