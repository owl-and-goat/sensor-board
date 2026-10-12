//! The REST API of an OpenThread border router, as far as the CLI uses it:
//! to find the devices on the border router's network.

use std::{net::Ipv6Addr, time::Duration};

use anyhow::{Context, Result};
use serde::{Deserialize, de::DeserializeOwned};

use crate::mdns;

/// The border router to ask when none is named: the host and port of its
/// REST API.
pub const DEFAULT_ADDRESS: &str = "cerberus:8081";

/// Timeout for a request. `/diagnostics` takes several seconds, because the
/// border router queries every router and waits for the replies.
const TIMEOUT: Duration = Duration::from_secs(30);

/// The border router's own Thread node, from `/node`.
#[derive(Debug, Deserialize)]
pub struct Node {
    #[serde(rename = "Rloc16")]
    pub rloc16: u16,
    /// Its routing locator, which starts with the network's mesh-local
    /// prefix.
    #[serde(rename = "RlocAddress")]
    pub rloc_address: Ipv6Addr,
}

/// The network diagnostics of one device, from `/diagnostics`. The border
/// router lists itself and every device that answered its query.
#[derive(Debug, Deserialize)]
pub struct Diagnostics {
    #[serde(rename = "Rloc16")]
    pub rloc16: u16,
    #[serde(rename = "IP6AddressList", default)]
    pub addresses: Vec<Ipv6Addr>,
    #[serde(rename = "Route", default)]
    pub route: Route,
}

/// A device's links to the routers of its network.
#[derive(Debug, Default, Deserialize)]
pub struct Route {
    #[serde(rename = "RouteData", default)]
    pub links: Vec<RouterLink>,
}

#[derive(Debug, Deserialize)]
pub struct RouterLink {
    #[serde(rename = "RouteId")]
    pub router: u8,
    /// How well the device receives the router, from 1 to 3. 0 if they have
    /// no direct radio link, and for the device's own entry.
    #[serde(rename = "LinkQualityIn")]
    pub quality_in: u8,
}

/// The addresses that a machine outside the Thread network can send to, of
/// every device but the border router itself: those that are neither
/// link-local nor mesh-local.
pub fn routable_addresses(node: &Node, devices: &[Diagnostics]) -> Vec<Ipv6Addr> {
    let mesh_local_prefix = prefix(node.rloc_address);
    devices
        .iter()
        .filter(|device| device.rloc16 != node.rloc16)
        .flat_map(|device| &device.addresses)
        .copied()
        .filter(|address| !address.is_unicast_link_local() && prefix(*address) != mesh_local_prefix)
        .collect()
}

/// The addresses to send to from this machine, to reach every board on the
/// border router's network: those of the devices that the border router
/// lists, and those of the boards that are advertised over mDNS. A board on
/// battery is only among the second, because the border router lists
/// routers.
pub async fn targets(node: &Node, devices: &[Diagnostics]) -> Vec<Ipv6Addr> {
    let mut targets = routable_addresses(node, devices);
    match mdns::boards().await {
        Ok(boards) => {
            for board in boards {
                if !targets.contains(&board.address) {
                    targets.push(board.address);
                }
            }
        }
        // The boards that are routers can still be reached.
        Err(e) => eprintln!("Could not look for boards over mDNS: {e:#}."),
    }
    targets
}

/// The first 64 bits of `address`.
fn prefix(address: Ipv6Addr) -> [u8; 8] {
    *address
        .octets()
        .first_chunk()
        .expect("an address is 16 bytes")
}

pub struct BorderRouter {
    address: String,
}

impl BorderRouter {
    /// `address` is the host and port of the REST API.
    pub fn at(address: &str) -> BorderRouter {
        BorderRouter {
            address: address.to_owned(),
        }
    }

    pub fn node(&self) -> Result<Node> {
        self.get("node")
    }

    pub fn diagnostics(&self) -> Result<Vec<Diagnostics>> {
        self.get("diagnostics")
    }

    /// The border router's own node, and the diagnostics of every device on
    /// its network. Does not block the caller's thread.
    pub async fn network(self) -> Result<(Node, Vec<Diagnostics>)> {
        let requests =
            tokio::task::spawn_blocking(move || anyhow::Ok((self.node()?, self.diagnostics()?)));
        requests.await?
    }

    /// Blocks until the border router has responded.
    fn get<T: DeserializeOwned>(&self, path: &str) -> Result<T> {
        let url = format!("http://{}/{path}", self.address);
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .timeout_global(Some(TIMEOUT))
            .build()
            .into();
        let request = agent.get(&url).header("Accept", "application/json");
        let body = request
            .call()
            .and_then(|mut response| response.body_mut().read_to_string())
            .with_context(|| format!("could not get {url}"))?;
        serde_json::from_str(&body).with_context(|| format!("{url} returned unexpected JSON"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two entries of a response from OTBR, shortened: the border router
    /// itself and a router.
    const DIAGNOSTICS: &str = r#"[{
        "ExtAddress": "56B34E0DD73DDE8E",
        "Rloc16": 23552,
        "Mode": { "RxOnWhenIdle": 1, "DeviceType": 1, "NetworkData": 1 },
        "Route": {
            "IdSequence": 200,
            "RouteData": [
                { "RouteId": 23, "LinkQualityOut": 0, "LinkQualityIn": 0, "RouteCost": 1 },
                { "RouteId": 59, "LinkQualityOut": 3, "LinkQualityIn": 2, "RouteCost": 1 }
            ]
        },
        "IP6AddressList": [
            "fd61:67e:fbdc:f78e:0:ff:fe00:5c00",
            "fdf7:2e7f:cbf2:1:e85b:9895:8a5f:5470",
            "fe80::54b3:4e0d:d73d:de8e"
        ],
        "ChildTable": [],
        "ChannelPages": "00"
    }, {
        "ExtAddress": "0A506EB99AF74369",
        "Rloc16": 60416,
        "IP6AddressList": ["fdf7:2e7f:cbf2:1:605b:76:31d1:4464"],
        "ChildTable": []
    }]"#;

    #[test]
    fn diagnostics_are_parsed() {
        let devices: Vec<Diagnostics> = serde_json::from_str(DIAGNOSTICS).unwrap();
        let [border_router, board] = devices.as_slice() else {
            panic!("expected two devices");
        };

        assert_eq!(border_router.rloc16, 0x5c00);
        assert_eq!(border_router.addresses.len(), 3);
        let link = &border_router.route.links[1];
        assert_eq!((link.router, link.quality_in), (59, 2));

        assert_eq!(board.rloc16, 0xec00);
        assert_eq!(
            board.addresses,
            ["fdf7:2e7f:cbf2:1:605b:76:31d1:4464"
                .parse::<Ipv6Addr>()
                .unwrap()]
        );
        // A device that sent no route data has no links.
        assert!(board.route.links.is_empty());
    }

    #[test]
    fn routable_addresses_leave_out_the_border_router_and_local_addresses() {
        let mut devices: Vec<Diagnostics> = serde_json::from_str(DIAGNOSTICS).unwrap();
        // The board's mesh-local and link-local addresses.
        devices[1].addresses.extend([
            "fd61:67e:fbdc:f78e:c51c:81a7:4754:989"
                .parse::<Ipv6Addr>()
                .unwrap(),
            "fe80::a4ab:e08f:a44b:ab".parse().unwrap(),
        ]);
        let node = Node {
            rloc16: 0x5c00,
            rloc_address: "fd61:67e:fbdc:f78e:0:ff:fe00:5c00".parse().unwrap(),
        };

        assert_eq!(
            routable_addresses(&node, &devices),
            ["fdf7:2e7f:cbf2:1:605b:76:31d1:4464"
                .parse::<Ipv6Addr>()
                .unwrap()]
        );
    }

    #[test]
    fn prefix_is_the_first_64_bits() {
        let address: Ipv6Addr = "fd61:67e:fbdc:f78e:0:ff:fe00:5c00".parse().unwrap();
        assert_eq!(
            prefix(address),
            [0xfd, 0x61, 0x06, 0x7e, 0xfb, 0xdc, 0xf7, 0x8e]
        );
    }

    #[test]
    fn node_is_parsed() {
        let node: Node = serde_json::from_str(
            r#"{
                "State": "leader",
                "RlocAddress": "fd61:67e:fbdc:f78e:0:ff:fe00:5c00",
                "ExtAddress": "56B34E0DD73DDE8E",
                "Rloc16": 23552
            }"#,
        )
        .unwrap();
        assert_eq!(node.rloc16, 0x5c00);
        assert_eq!(
            node.rloc_address.segments()[..4],
            [0xfd61, 0x67e, 0xfbdc, 0xf78e]
        );
    }
}
