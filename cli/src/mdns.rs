//! Sensor boards that are advertised over mDNS on this machine's network.
//!
//! A board registers a service with the SRP server of its Thread network,
//! which runs on the border router, and the border router advertises it on
//! the LAN: the scrape endpoint of a board on aux power, and the board
//! service of a board on battery. Both are named after the board's serial
//! number. This is the only way to find a board on battery from outside its
//! network, because the border router's REST API only lists routers.
//!
//! The lookup runs `avahi-browse`, so it needs avahi's tools and its daemon.

use std::{net::Ipv6Addr, process::Command};

use anyhow::{Context, Result, bail};
use protocol::{BOARD_SERVICE, HOST_NAME_PREFIX, SCRAPE_SERVICE};

/// A board that is advertised over mDNS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Advertised {
    pub serial: String,
    pub address: Ipv6Addr,
}

/// The boards that are advertised on this machine's network. A board that
/// has restarted can be missing for as long as its previous registration
/// lasts: ten minutes for a board on battery.
pub async fn boards() -> Result<Vec<Advertised>> {
    let lookups = [SCRAPE_SERVICE, BOARD_SERVICE]
        .map(|kind| tokio::task::spawn_blocking(move || browse(kind)));
    let mut boards = Vec::new();
    for lookup in lookups {
        for board in lookup.await?? {
            if !boards.contains(&board) {
                boards.push(board);
            }
        }
    }
    Ok(boards)
}

/// The boards that advertise a service of type `kind`. Blocks for about a
/// second, until avahi has no more answers.
fn browse(kind: &str) -> Result<Vec<Advertised>> {
    let output = Command::new("avahi-browse")
        .args(["--resolve", "--parsable", "--terminate", kind])
        .output()
        .context("could not run avahi-browse")?;
    if !output.status.success() {
        let error = String::from_utf8_lossy(&output.stderr);
        bail!("avahi-browse failed: {}", error.trim());
    }
    Ok(parse(&String::from_utf8_lossy(&output.stdout)))
}

/// The boards in the output of `avahi-browse --resolve --parsable`. A line
/// that starts with `=` is a resolved service. Its fields are separated by
/// semicolons: the interface, the protocol, the service's name, type and
/// domain, the host's name, its address, the port and the TXT record. The
/// same service appears once per interface and protocol it was found on.
fn parse(output: &str) -> Vec<Advertised> {
    let mut boards = Vec::new();
    for line in output.lines() {
        let fields: Vec<&str> = line.split(';').collect();
        let ["=", _, _, name, _, _, _, address, ..] = fields[..] else {
            continue;
        };
        // Other devices can advertise the same type of service.
        let Some(serial) = name.strip_prefix(HOST_NAME_PREFIX) else {
            continue;
        };
        let Ok(address) = address.parse() else {
            continue;
        };
        let board = Advertised {
            serial: serial.to_owned(),
            address,
        };
        if !boards.contains(&board) {
            boards.push(board);
        }
    }
    boards
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boards_are_parsed_once_each() {
        let output = "\
+;wlp5s0;IPv6;sensor-board-4C0045000350475532303120;_prometheus-http._tcp;local
+;wlp5s0;IPv4;sensor-board-4C0045000350475532303120;_prometheus-http._tcp;local
=;wlp5s0;IPv6;sensor-board-4C0045000350475532303120;_prometheus-http._tcp;local;\
sensor-board-4C0045000350475532303120.local;fdf7:2e7f:cbf2:1:ba8a:ba21:ebef:11c2;9469;
=;wlp5s0;IPv4;sensor-board-4C0045000350475532303120;_prometheus-http._tcp;local;\
sensor-board-4C0045000350475532303120.local;fdf7:2e7f:cbf2:1:ba8a:ba21:ebef:11c2;9469;
=;wlp5s0;IPv4;node-exporter;_prometheus-http._tcp;local;cerberus.local;10.0.0.1;9100;
=;wlp5s0;IPv6;sensor-board-4B0041000350475532303120;_prometheus-http._tcp;local;\
sensor-board-4B0041000350475532303120.local;fdf7:2e7f:cbf2:1:1dd5:1769:5a3d:2b17;9469;
";
        let board = |serial: &str, address: &str| Advertised {
            serial: serial.to_owned(),
            address: address.parse().unwrap(),
        };
        assert_eq!(
            parse(output),
            [
                board(
                    "4C0045000350475532303120",
                    "fdf7:2e7f:cbf2:1:ba8a:ba21:ebef:11c2"
                ),
                board(
                    "4B0041000350475532303120",
                    "fdf7:2e7f:cbf2:1:1dd5:1769:5a3d:2b17"
                ),
            ]
        );
    }
}
