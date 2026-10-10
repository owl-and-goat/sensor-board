//! Putting boards on a Thread network.

use std::{
    env,
    ffi::OsString,
    fs::{self, OpenOptions, Permissions},
    io::{ErrorKind, Write},
    os::unix::fs::{OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{Context, Result, bail};
use protocol::{
    Address, Addresses, Dataset, Link, Neighbor, NeighborTable, NetworkStatus, Route, Router,
    RouterTable,
};
use rand::RngExt;

use crate::board::Board;

/// How long a board gets to attach. Forming a network is the slow case: the
/// first board looks for an existing one before it makes itself leader.
const ATTACH_TIMEOUT: Duration = Duration::from_secs(120);
const POLL_INTERVAL: Duration = Duration::from_millis(500);

/// The parameters of a Thread network that does not exist yet.
pub struct NewNetwork {
    /// At most 16 bytes.
    pub name: String,
    /// 11 to 26, the 2.4 GHz channels.
    pub channel: u8,
    pub pan_id: u16,
    pub extended_pan_id: [u8; 8],
    /// The /64 that every address inside the mesh shares.
    pub mesh_local_prefix: [u8; 8],
    pub network_key: [u8; 16],
    /// Key for commissioning sessions, normally derived from a passphrase.
    /// Nothing here commissions that way, so it only has to be unguessable.
    pub pskc: [u8; 16],
}

/// The MeshCoP TLVs an operational dataset is made of.
#[derive(Clone, Copy)]
#[repr(u8)]
enum Tlv {
    Channel = 0,
    PanId = 1,
    ExtendedPanId = 2,
    NetworkName = 3,
    Pskc = 4,
    NetworkKey = 5,
    MeshLocalPrefix = 7,
    SecurityPolicy = 12,
    ActiveTimestamp = 14,
    ChannelMask = 53,
}

impl NewNetwork {
    /// Random identifiers and keys, the way OpenThread's `dataset init new`
    /// picks them.
    pub fn random() -> NewNetwork {
        let mut rng = rand::rng();
        // 0xffff is the broadcast PAN ID.
        let pan_id = rng.random_range(0..0xffff);
        // A unique local address prefix, fd00::/8.
        let mut mesh_local_prefix: [u8; 8] = rng.random();
        mesh_local_prefix[0] = 0xfd;

        NewNetwork {
            name: format!("sensors-{pan_id:04x}"),
            channel: rng.random_range(11..=26),
            pan_id,
            extended_pan_id: rng.random(),
            mesh_local_prefix,
            network_key: rng.random(),
            pskc: rng.random(),
        }
    }

    /// The active operational dataset that makes a device a member.
    pub fn dataset(&self) -> Dataset {
        let mut tlvs = Vec::new();
        let mut push = |tlv: Tlv, value: &[u8]| {
            tlvs.push(tlv as u8);
            tlvs.push(value.len() as u8);
            tlvs.extend_from_slice(value);
        };

        // One second, no ticks, not authoritative: the first version of this
        // network's dataset.
        push(Tlv::ActiveTimestamp, &[0, 0, 0, 0, 0, 1, 0, 0]);
        // Channel page 0, then the channel as 16 bits.
        push(Tlv::Channel, &[0, 0, self.channel]);
        // Channel page 0, a 4-byte mask: channels 11 to 26 may be used.
        push(Tlv::ChannelMask, &[0, 4, 0x00, 0x1f, 0xff, 0xe0]);
        push(Tlv::ExtendedPanId, &self.extended_pan_id);
        push(Tlv::MeshLocalPrefix, &self.mesh_local_prefix);
        push(Tlv::NetworkKey, &self.network_key);
        push(Tlv::NetworkName, self.name.as_bytes());
        push(Tlv::PanId, &self.pan_id.to_be_bytes());
        push(Tlv::Pskc, &self.pskc);
        // Key rotation every 672 hours, and OpenThread's default flags.
        push(Tlv::SecurityPolicy, &[0x02, 0xa0, 0xf7, 0xf8]);

        Dataset::from_tlvs(&tlvs).expect("a dataset with a 16-byte name is about 110 bytes")
    }
}

/// The dataset of the network that [`init`] puts boards on, kept in a file
/// from one run to the next: a board that is plugged in later then joins the
/// network the others are on, whether or not one of them is plugged in too.
pub struct SavedDataset {
    path: PathBuf,
}

impl SavedDataset {
    pub fn at(path: PathBuf) -> SavedDataset {
        SavedDataset { path }
    }

    /// The file used unless another is named: `sensor-board/dataset` under
    /// `$XDG_STATE_HOME`, which is `~/.local/state` unless set.
    pub fn default_path() -> Result<PathBuf> {
        state_dir(env::var_os("XDG_STATE_HOME"), env::var_os("HOME"))
            .map(|dir| dir.join("sensor-board").join("dataset"))
            .context("neither XDG_STATE_HOME nor HOME says where to keep the dataset; use --dataset-path")
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// `None` if nothing has been saved yet.
    pub fn load(&self) -> Result<Option<Dataset>> {
        let hex = match fs::read_to_string(&self.path) {
            Ok(hex) => hex,
            Err(e) if e.kind() == ErrorKind::NotFound => return Ok(None),
            Err(e) => {
                return Err(e).with_context(|| format!("could not read {}", self.path.display()));
            }
        };
        let dataset = hex
            .trim()
            .parse()
            .with_context(|| format!("{} does not hold a dataset", self.path.display()))?;
        Ok(Some(dataset))
    }

    /// The file holds the network key, so only its owner gets to read it.
    pub fn save(&self, dataset: &Dataset) -> Result<()> {
        let write = || -> std::io::Result<()> {
            if let Some(dir) = self.path.parent() {
                fs::create_dir_all(dir)?;
            }
            let mut file = OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .mode(0o600)
                .open(&self.path)?;
            // A file that was already there keeps the mode it had.
            file.set_permissions(Permissions::from_mode(0o600))?;
            writeln!(file, "{dataset}")
        };
        write().with_context(|| format!("could not write {}", self.path.display()))
    }
}

/// `$XDG_STATE_HOME`, or its default when that is unset.
fn state_dir(xdg_state_home: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
    match xdg_state_home.filter(|dir| !dir.is_empty()) {
        Some(dir) => Some(PathBuf::from(dir)),
        None => Some(PathBuf::from(home?).join(".local/state")),
    }
}

/// Where [`init`] gets the dataset of the network to put boards on.
#[derive(Debug, PartialEq, Eq)]
enum Source {
    /// The file from an earlier run.
    Saved(Dataset),
    /// The board at this index, which is on a network already.
    Board(usize, Dataset),
    /// Nowhere: a new network is to be made.
    New,
}

impl Source {
    /// `on_boards` is the dataset each attached board has, if it has one.
    fn choose(saved: Option<Dataset>, on_boards: &[Option<Dataset>], force_reinit: bool) -> Self {
        if force_reinit {
            return Source::New;
        }
        if let Some(dataset) = saved {
            return Source::Saved(dataset);
        }
        let configured = on_boards
            .iter()
            .enumerate()
            .find_map(|(index, dataset)| Some((index, dataset.clone()?)));
        match configured {
            Some((index, dataset)) => Source::Board(index, dataset),
            None => Source::New,
        }
    }
}

/// Put every one of `boards` on the same network. That is the saved one; or,
/// with nothing saved, the one the first configured board is on; or else a
/// new one. `force_reinit` makes it a new one regardless. Whichever it is, it
/// is the saved one from here on.
pub async fn init(boards: &[Board], saved: &SavedDataset, force_reinit: bool) -> Result<()> {
    let mut on_boards = Vec::new();
    for board in boards {
        on_boards.push(board.dataset().await?);
    }

    let source = Source::choose(saved.load()?, &on_boards, force_reinit);
    let from_file = matches!(source, Source::Saved(_));
    let dataset = match source {
        Source::Saved(dataset) => {
            println!("Using the network saved in {}.", saved.path().display());
            dataset
        }
        Source::Board(index, dataset) => {
            println!("Using the network of board {}.", boards[index].serial());
            dataset
        }
        Source::New => {
            let network = NewNetwork::random();
            println!(
                "Creating network {} on channel {}, PAN ID {:#06x}.",
                network.name, network.channel, network.pan_id
            );
            network.dataset()
        }
    };
    // Before any board is told to join: a network whose dataset is only on
    // the boards is one a later board cannot be added to without them.
    if !from_file {
        saved.save(&dataset)?;
        println!("Saved its dataset in {}.", saved.path().display());
    }

    // Boards that are on the network already go first, and each board is
    // attached before the next is told to join. That way one board forms the
    // network and the others find it, rather than each starting a partition
    // of its own that then has to merge.
    let mut members: Vec<_> = boards.iter().zip(on_boards).collect();
    members.sort_by_key(|(_, current)| current.as_ref() != Some(&dataset));
    for (board, current) in members {
        if current.as_ref() != Some(&dataset) {
            if current.is_some() {
                println!("Moving board {} over from another network.", board.serial());
            }
            board.join(&dataset).await?;
        }
        let link = attached(board).await?;
        println!("{}  {}", board.serial(), describe_link(&link));
    }
    Ok(())
}

/// Wait until `board` is attached to its network.
pub async fn attached(board: &Board) -> Result<Link> {
    let deadline = Instant::now() + ATTACH_TIMEOUT;
    loop {
        match board.network_status().await? {
            NetworkStatus::Configured(link) if link.role.is_attached() => return Ok(link),
            NetworkStatus::Unavailable(e) => bail!("board {}: {e}", board.serial()),
            NetworkStatus::Unconfigured => bail!("board {} has no network", board.serial()),
            // Still starting up, or still looking for the network.
            NetworkStatus::Starting | NetworkStatus::Configured(_) => {}
        }
        if Instant::now() > deadline {
            bail!(
                "board {} did not attach within {} s",
                board.serial(),
                ATTACH_TIMEOUT.as_secs()
            );
        }
        tokio::time::sleep(POLL_INTERVAL).await;
    }
}

pub fn describe(status: &NetworkStatus) -> String {
    match status {
        NetworkStatus::Starting => "starting".into(),
        NetworkStatus::Unavailable(e) => format!("no Thread: {e}"),
        NetworkStatus::Unconfigured => "no network".into(),
        NetworkStatus::Configured(link) => describe_link(link),
    }
}

pub fn describe_link(link: &Link) -> String {
    let Link {
        role,
        rloc16,
        channel,
        pan_id,
    } = link;
    if role.is_attached() {
        format!("{role}, RLOC16 {rloc16:#06x}, channel {channel}, PAN ID {pan_id:#06x}")
    } else {
        format!("{role}, looking for PAN ID {pan_id:#06x} on channel {channel}")
    }
}

/// The table as text: a heading, then a line for each neighbor.
pub fn describe_neighbors(table: &NeighborTable) -> String {
    let mut text = format!(
        "{:<6}  {:<6}  {:<16}  {:>7}  {:>7}  {:>8}  {:>9}  {:>6}\n",
        "Kind", "RLOC16", "Extended address", "Age", "Quality", "Avg RSSI", "Last RSSI", "Margin"
    );
    for neighbor in &table.neighbors {
        let Neighbor {
            kind,
            rloc16,
            ext_address,
            age_secs,
            link_quality_in,
            average_rssi,
            last_rssi,
            link_margin,
        } = neighbor;
        let (age, average_rssi, last_rssi, link_margin) = (
            format!("{age_secs} s"),
            format!("{average_rssi} dBm"),
            format!("{last_rssi} dBm"),
            format!("{link_margin} dB"),
        );
        text += &format!(
            "{kind:<6}  {rloc16:#06x}  {ext_address:<16}  {age:>7}  {link_quality_in:>7}  \
             {average_rssi:>8}  {last_rssi:>9}  {link_margin:>6}\n",
        );
    }
    if table.truncated {
        text += "The board has more neighbors than these.\n";
    }
    text
}

/// The table as text: a heading, then a line for each router.
pub fn describe_routers(table: &RouterTable) -> String {
    let mut text = format!("{:<6}  {:<10}  {:>9}\n", "RLOC16", "Reached", "Path cost");
    for Router { id, route } in &table.routers {
        let (reached, cost) = match route {
            Route::ThisBoard => ("this board".to_owned(), None),
            Route::Direct { cost } => ("directly".to_owned(), Some(cost)),
            Route::Relayed { next_hop, cost } => {
                (format!("via {:#06x}", next_hop.rloc16()), Some(cost))
            }
            Route::Unreachable => ("no route".to_owned(), None),
        };
        let rloc16 = id.rloc16();
        text += &match cost {
            Some(cost) => format!("{rloc16:#06x}  {reached:<10}  {cost:>9}\n"),
            None => format!("{rloc16:#06x}  {reached}\n"),
        };
    }
    text
}

/// The addresses as text: a line for each, with where it reaches the board
/// from.
pub fn describe_addresses(addresses: &Addresses) -> String {
    let mut text = String::new();
    for Address { address, kind } in &addresses.addresses {
        text += &format!("{kind:<10}  {address}\n");
    }
    if addresses.truncated {
        text += "The board has more addresses than these.\n";
    }
    text
}

#[cfg(test)]
mod tests {
    use protocol::{ExtAddress, NeighborKind, RouterId};

    use super::*;

    #[test]
    fn routers_line_up_under_their_headings() {
        let mut table = RouterTable::default();
        let routes = [
            (24, Route::Direct { cost: 1 }),
            (27, Route::ThisBoard),
            (
                47,
                Route::Relayed {
                    next_hop: RouterId(24),
                    cost: 3,
                },
            ),
            (58, Route::Unreachable),
        ];
        for (id, route) in routes {
            let id = RouterId(id);
            table.routers.push(Router { id, route }).unwrap();
        }

        assert_eq!(
            describe_routers(&table),
            "RLOC16  Reached     Path cost\n\
             0x6000  directly            1\n\
             0x6c00  this board\n\
             0xbc00  via 0x6000          3\n\
             0xe800  no route\n"
        );
    }

    #[test]
    fn neighbors_line_up_under_their_headings() {
        let neighbor = Neighbor {
            kind: NeighborKind::Child,
            rloc16: 0xd401,
            ext_address: ExtAddress([0x02, 0xa1, 0, 0, 0, 0, 0x0f, 0xff]),
            age_secs: 3,
            link_quality_in: 3,
            average_rssi: -18,
            last_rssi: -20,
            link_margin: 82,
        };
        let mut table = NeighborTable::default();
        table.neighbors.push(neighbor).unwrap();
        let router = Neighbor {
            kind: NeighborKind::Router,
            rloc16: 0x0c00,
            age_secs: 120,
            average_rssi: -101,
            ..neighbor
        };
        table.neighbors.push(router).unwrap();

        assert_eq!(
            describe_neighbors(&table),
            "Kind    RLOC16  Extended address      Age  Quality  Avg RSSI  Last RSSI  Margin\n\
             child   0xd401  02a1000000000fff      3 s        3   -18 dBm    -20 dBm   82 dB\n\
             router  0x0c00  02a1000000000fff    120 s        3  -101 dBm    -20 dBm   82 dB\n"
        );
    }

    fn dataset(byte: u8) -> Dataset {
        Dataset::from_tlvs(&[0, 1, byte]).unwrap()
    }

    #[test]
    fn init_prefers_the_saved_network() {
        let on_boards = [None, Some(dataset(2))];
        assert_eq!(
            Source::choose(Some(dataset(1)), &on_boards, false),
            Source::Saved(dataset(1))
        );
    }

    #[test]
    fn init_adopts_a_boards_network_when_nothing_is_saved() {
        let on_boards = [None, Some(dataset(2)), Some(dataset(3))];
        assert_eq!(
            Source::choose(None, &on_boards, false),
            Source::Board(1, dataset(2))
        );
    }

    #[test]
    fn init_makes_a_network_when_there_is_none_or_when_forced() {
        assert_eq!(Source::choose(None, &[None, None], false), Source::New);
        assert_eq!(
            Source::choose(Some(dataset(1)), &[Some(dataset(1))], true),
            Source::New
        );
    }

    #[test]
    fn saved_dataset_round_trips_and_is_private() {
        let dir = env::temp_dir().join(format!("sensor-board-cli-test-{}", std::process::id()));
        let saved = SavedDataset::at(dir.join("state/dataset"));
        assert_eq!(saved.load().unwrap(), None);

        saved.save(&dataset(1)).unwrap();
        saved.save(&dataset(2)).unwrap();
        assert_eq!(saved.load().unwrap(), Some(dataset(2)));
        let mode = fs::metadata(saved.path()).unwrap().permissions().mode();
        assert_eq!(mode & 0o777, 0o600);

        fs::write(saved.path(), "not a dataset").unwrap();
        assert!(saved.load().is_err());
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn state_is_kept_where_xdg_says() {
        let os = |s: &str| Some(OsString::from(s));
        assert_eq!(
            state_dir(os("/state"), os("/home/me")),
            Some("/state".into())
        );
        assert_eq!(
            state_dir(None, os("/home/me")),
            Some("/home/me/.local/state".into())
        );
        assert_eq!(
            state_dir(os(""), os("/home/me")),
            Some("/home/me/.local/state".into())
        );
        assert_eq!(state_dir(None, None), None);
    }

    /// The TLVs, their order and their fixed values are those of a dataset
    /// that the boards' own OpenThread stack generated, less its wake-up
    /// channel TLV (type 74).
    #[test]
    fn dataset_is_laid_out_like_openthreads() {
        let network = NewNetwork {
            name: "OpenThread-5938".into(),
            channel: 15,
            pan_id: 0x5938,
            extended_pan_id: [0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0xca, 0xfe],
            mesh_local_prefix: [0xfd, 0xde, 0xad, 0x00, 0xbe, 0xef, 0x00, 0x00],
            network_key: [
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
                0xee, 0xff,
            ],
            pskc: [
                0x3a, 0xa5, 0x5f, 0x91, 0xca, 0x47, 0xd1, 0xe4, 0xe7, 0x1a, 0x08, 0xcb, 0x35, 0xe9,
                0x15, 0x91,
            ],
        };
        assert_eq!(
            network.dataset().to_string(),
            "0e08000000000001000000030000\
             0f35060004001fffe00208dead00beef00cafe0708fddead00beef0000\
             051000112233445566778899aabbccddeeff030f4f70656e5468726561642d35393338\
             010259380410\
             3aa55f91ca47d1e4e71a08cb35e915910c0402a0f7f8"
        );
    }

    #[test]
    fn random_networks_are_valid_and_distinct() {
        let (a, b) = (NewNetwork::random(), NewNetwork::random());
        assert_ne!(a.network_key, b.network_key);
        for network in [a, b] {
            assert!((11..=26).contains(&network.channel));
            assert_ne!(network.pan_id, 0xffff);
            assert!(network.name.len() <= 16);
            assert_eq!(network.mesh_local_prefix[0], 0xfd);
        }
    }
}
