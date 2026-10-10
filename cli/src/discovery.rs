//! The `network show` command: list the sensor boards on a Thread network.
//!
//! Boards are found with [`DiscoveryMessage`]. An attached board multicasts
//! the request and forwards the replies. Without one, this machine sends the
//! request to every device that a border router lists, through that border
//! router.

use std::{collections::BTreeMap, net::Ipv6Addr, time::Duration};

use anyhow::{Context, Result, bail};
use postcard_rpc::host_client::MultiSubRxError;
use protocol::{AddressKind, DISCOVERY_PORT, DiscoveryMessage, Member, NetworkStatus, RouterId};
use tokio::{
    net::UdpSocket,
    time::{Instant, timeout_at},
};

use crate::{
    board::{self, Board},
    border_router::{BorderRouter, RouterLink},
    network,
};

/// The border router to ask when none is named.
pub const DEFAULT_BORDER_ROUTER: &str = "cerberus:8081";

/// How often the request is sent. Neither the request nor a reply is
/// retransmitted when it is lost.
const ROUNDS: usize = 2;

/// How long replies are collected after each request.
const ROUND: Duration = Duration::from_millis(1500);

/// The link between a board and the device that was asked about the network:
/// an attached board, or a border router.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Link {
    /// Seconds since the device last heard from the board. A border router
    /// does not report it.
    age_secs: Option<u32>,
    /// How well the device receives the board, from 1 to 3.
    quality: u8,
    /// In dBm. A border router does not report it.
    average_rssi: Option<i8>,
}

/// A board that replied. `link` is `None` if the board has no direct radio
/// link with the device that was asked.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Found {
    member: Member,
    link: Option<Link>,
}

impl Found {
    /// The address to show for the board: one that is reachable from outside
    /// the Thread network if it has one, and else its mesh-local address.
    fn address(&self) -> Option<Ipv6Addr> {
        let of_kind = |kind| {
            let mut addresses = self.member.addresses.addresses.iter();
            addresses.find(|a| a.kind == kind).map(|a| a.address)
        };
        of_kind(AddressKind::Routable).or_else(|| of_kind(AddressKind::MeshLocal))
    }
}

/// Print the boards on the network. `serial` selects the attached board to
/// ask through, and `border_router` the border router. With neither, ask
/// through any attached board that is on a network, and without one through
/// the default border router.
pub async fn show(serial: Option<&str>, border_router: Option<&str>) -> Result<()> {
    let found = match (border_router, serial) {
        (Some(address), _) => through_border_router(address).await?,
        (None, Some(serial)) => through_board(&Board::select(Some(serial)).await?).await?,
        (None, None) => match attached_board().await? {
            Some(board) => through_board(&board).await?,
            None => through_border_router(DEFAULT_BORDER_ROUTER).await?,
        },
    };
    print!("{}", describe(&found));
    Ok(())
}

/// The first attached board that is on a network.
async fn attached_board() -> Result<Option<Board>> {
    for device in board::devices().await? {
        let Ok(board) = Board::open(&device).await else {
            continue;
        };
        if let Ok(NetworkStatus::Configured(link)) = board.network_status().await
            && link.role.is_attached()
        {
            return Ok(Some(board));
        }
    }
    Ok(None)
}

/// Find the boards on the network of `board`. Each link is as `board`
/// measures it.
async fn through_board(board: &Board) -> Result<Vec<Found>> {
    match board.network_status().await? {
        NetworkStatus::Configured(link) if link.role.is_attached() => {}
        status => bail!(
            "board {} is not on a network: {}",
            board.serial(),
            network::describe(&status)
        ),
    }
    eprintln!("Asking through board {}.", board.serial());

    // The board receives its own request and replies to itself, so it is
    // among the members without being added here.
    let mut members = BTreeMap::new();
    // Subscribe before asking, so that no reply is missed.
    let mut replies = board.discovered().await?;
    for _ in 0..ROUNDS {
        board.discover().await?;
        let deadline = Instant::now() + ROUND;
        loop {
            match timeout_at(deadline, replies.recv()).await {
                Ok(Ok(member)) => {
                    members.insert(member.board, member);
                }
                Ok(Err(MultiSubRxError::Lagged(_))) => {}
                Ok(Err(MultiSubRxError::IoClosed)) => bail!("board {} is gone", board.serial()),
                Err(_) => break,
            }
        }
    }

    let neighbors = board.neighbors().await?.neighbors;
    let found = members.into_values().map(|member| {
        let neighbor = neighbors.iter().find(|n| n.rloc16 == member.rloc16);
        let link = neighbor.map(|neighbor| Link {
            age_secs: Some(neighbor.age_secs),
            quality: neighbor.link_quality_in,
            average_rssi: Some(neighbor.average_rssi),
        });
        Found { member, link }
    });
    Ok(found.collect())
}

/// Find the boards on the network of the border router whose REST API is at
/// `address`. Each link is as the border router measures it.
async fn through_border_router(address: &str) -> Result<Vec<Found>> {
    eprintln!("Asking through the border router at {address}.");
    let border_router = BorderRouter::at(address);
    let requests = tokio::task::spawn_blocking(move || {
        anyhow::Ok((border_router.node()?, border_router.diagnostics()?))
    });
    let (node, devices) = requests.await??;

    let mesh_local_prefix = prefix(node.rloc_address);
    // The addresses of every other device that this machine can send to.
    let targets: Vec<Ipv6Addr> = devices
        .iter()
        .filter(|device| device.rloc16 != node.rloc16)
        .flat_map(|device| &device.addresses)
        .copied()
        .filter(|address| !address.is_unicast_link_local() && prefix(*address) != mesh_local_prefix)
        .collect();
    if targets.is_empty() {
        bail!("the border router lists no other device with a routable address");
    }

    // Boards reply to the discovery port, whatever port the request came
    // from.
    let socket = UdpSocket::bind((Ipv6Addr::UNSPECIFIED, DISCOVERY_PORT))
        .await
        .with_context(|| format!("could not bind UDP port {DISCOVERY_PORT}"))?;
    let mut request = [0; DiscoveryMessage::MAX_LEN];
    let request = DiscoveryMessage::Request
        .encode(&mut request)
        .expect("the buffer has room for the longest message");

    let mut members = BTreeMap::new();
    let mut datagram = [0; 2 * DiscoveryMessage::MAX_LEN];
    for _ in 0..ROUNDS {
        let mut sent = Ok(());
        let mut any_sent = false;
        for target in &targets {
            match socket.send_to(request, (*target, DISCOVERY_PORT)).await {
                Ok(_) => any_sent = true,
                Err(e) => sent = Err(e),
            }
        }
        if !any_sent {
            sent.context("could not send to any device on the Thread network")?;
        }

        let deadline = Instant::now() + ROUND;
        while let Ok(received) = timeout_at(deadline, socket.recv_from(&mut datagram)).await {
            let (len, _) = received.context("could not receive replies")?;
            if let Some(DiscoveryMessage::Reply(member)) =
                DiscoveryMessage::decode(&datagram[..len])
            {
                members.insert(member.board, member);
            }
        }
    }
    if members.is_empty() {
        bail!(
            "none of the {} devices that the border router lists replied. A board replies if its \
             firmware has discovery and this machine has a route to the Thread network",
            devices.len() - 1
        );
    }

    let links = devices
        .iter()
        .find(|device| device.rloc16 == node.rloc16)
        .map_or(&[][..], |device| &device.route.links);
    let found = members.into_values().map(|member| {
        let link = router_link(links, member.rloc16);
        Found { member, link }
    });
    Ok(found.collect())
}

/// The first 64 bits of `address`.
fn prefix(address: Ipv6Addr) -> [u8; 8] {
    *address
        .octets()
        .first_chunk()
        .expect("an address is 16 bytes")
}

/// The border router's link to the board with `rloc16`, from the border
/// router's links to routers. `None` if the board is a child, or a router
/// without a direct radio link to the border router.
fn router_link(links: &[RouterLink], rloc16: u16) -> Option<Link> {
    let router = RouterId::of_rloc16(rloc16);
    if router.rloc16() != rloc16 {
        return None;
    }
    let link = links.iter().find(|link| link.router == router.0)?;
    (link.quality_in > 0).then_some(Link {
        age_secs: None,
        quality: link.quality_in,
        average_rssi: None,
    })
}

/// Format the boards as text: a heading, then one line per board. A value
/// that is not known shows as `-`.
fn describe(boards: &[Found]) -> String {
    let or_dash = |value: Option<String>| value.unwrap_or_else(|| "-".to_owned());
    let addresses: Vec<String> = boards
        .iter()
        .map(|board| or_dash(board.address().map(|a| a.to_string())))
        .collect();
    let heading = "IPv6 address";
    let width = addresses.iter().map(String::len).max().unwrap_or(0);
    let width = width.max(heading.len());

    let mut text = format!(
        "{:<24}  {:<6}  {:<width$}  {:>7}  {:>7}  {:>8}\n",
        "Serial", "RLOC16", heading, "Age", "Quality", "Avg RSSI"
    );
    for (board, address) in boards.iter().zip(addresses) {
        let Member {
            board: serial,
            rloc16,
            ..
        } = &board.member;
        let link = board.link;
        let age = or_dash(link.and_then(|l| l.age_secs).map(|age| format!("{age} s")));
        let quality = or_dash(link.map(|l| l.quality.to_string()));
        let rssi = or_dash(
            link.and_then(|l| l.average_rssi)
                .map(|rssi| format!("{rssi} dBm")),
        );
        text += &format!(
            "{serial}  {rloc16:#06x}  {address:<width$}  {age:>7}  {quality:>7}  {rssi:>8}\n"
        );
    }
    text
}

#[cfg(test)]
mod tests {
    use protocol::{Address, Addresses, BoardId};

    use super::*;

    fn member(serial: &str, rloc16: u16, addresses: &[(&str, AddressKind)]) -> Member {
        let mut list = Addresses::default();
        for (address, kind) in addresses {
            let address = Address {
                address: address.parse().unwrap(),
                kind: *kind,
            };
            list.addresses.push(address).unwrap();
        }
        Member {
            board: serial.parse::<BoardId>().unwrap(),
            rloc16,
            addresses: list,
        }
    }

    #[test]
    fn boards_line_up_under_their_headings() {
        let boards = [
            Found {
                member: member(
                    "4B002E000350475532303120",
                    0x1400,
                    &[
                        (
                            "fd61:67e:fbdc:f78e:c51c:81a7:4754:989",
                            AddressKind::MeshLocal,
                        ),
                        (
                            "fdf7:2e7f:cbf2:1:9237:bd87:c4f4:c874",
                            AddressKind::Routable,
                        ),
                    ],
                ),
                link: Some(Link {
                    age_secs: Some(3),
                    quality: 3,
                    average_rssi: Some(-48),
                }),
            },
            // The attached board itself: it has no link to itself.
            Found {
                member: member(
                    "4C0032000350475532303120",
                    0xec00,
                    &[(
                        "fd61:67e:fbdc:f78e:4c51:2d3d:c69c:4af6",
                        AddressKind::MeshLocal,
                    )],
                ),
                link: None,
            },
            // As a border router reports a link.
            Found {
                member: member("4D0021000350475532303120", 0xb800, &[]),
                link: Some(Link {
                    age_secs: None,
                    quality: 2,
                    average_rssi: None,
                }),
            },
        ];

        assert_eq!(
            describe(&boards),
            "Serial                    RLOC16  IPv6 address                                Age  Quality  Avg RSSI\n\
             4B002E000350475532303120  0x1400  fdf7:2e7f:cbf2:1:9237:bd87:c4f4:c874        3 s        3   -48 dBm\n\
             4C0032000350475532303120  0xec00  fd61:67e:fbdc:f78e:4c51:2d3d:c69c:4af6        -        -         -\n\
             4D0021000350475532303120  0xb800  -                                             -        2         -\n"
        );
    }

    #[test]
    fn border_router_link_is_for_routers_it_hears() {
        let links = [
            RouterLink {
                router: 5,
                quality_in: 3,
            },
            RouterLink {
                router: 46,
                quality_in: 0,
            },
        ];
        let heard = Link {
            age_secs: None,
            quality: 3,
            average_rssi: None,
        };
        assert_eq!(router_link(&links, 0x1400), Some(heard));
        // A child of that router.
        assert_eq!(router_link(&links, 0x1401), None);
        // A router with no direct link, and one that is not listed.
        assert_eq!(router_link(&links, 0xb800), None);
        assert_eq!(router_link(&links, 0xec00), None);
    }

    #[test]
    fn prefix_is_the_first_64_bits() {
        let address: Ipv6Addr = "fd61:67e:fbdc:f78e:0:ff:fe00:5c00".parse().unwrap();
        assert_eq!(
            prefix(address),
            [0xfd, 0x61, 0x06, 0x7e, 0xfb, 0xdc, 0xf7, 0x8e]
        );
    }
}
