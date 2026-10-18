use std::{env, fs, path::PathBuf, process::Command};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=ddsl");
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

    println!("cargo:rustc-env=BUILD_COMMIT={}", commit());
}

/// The Git commit that the firmware is built from: the abbreviated hash,
/// with `-dirty` if the firmware or `protocol` has uncommitted changes.
/// `unknown` outside a Git checkout.
fn commit() -> String {
    let git = |args: &[&str]| {
        let output = Command::new("git").args(args).output().ok()?;
        output
            .status
            .success()
            .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
    };
    let Some(hash) = git(&["rev-parse", "--short=12", "HEAD"]) else {
        return "unknown".into();
    };

    // Run again when the checked-out commit or the staged files change: a
    // commit changes neither the sources nor their modification times.
    for path in ["HEAD", "logs/HEAD", "index"] {
        if let Some(path) = git(&["rev-parse", "--git-path", path]) {
            println!("cargo:rerun-if-changed={path}");
        }
    }
    println!("cargo:rerun-if-changed=../protocol");

    match git(&["status", "--porcelain", "--", ".", "../protocol"]) {
        Some(changes) if changes.is_empty() => hash,
        _ => format!("{hash}-dirty"),
    }
}
