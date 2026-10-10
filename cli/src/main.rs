//! Command-line client for sensor boards attached over USB.

mod board;
mod config;
mod coprocessor;
mod firmware;
mod network;
mod reports;

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand};

use board::Board;
use network::SavedDataset;
use protocol::{BoardId, Link, NetworkStatus, Role, RouterId, Sensor, SensorValue};

#[derive(Parser)]
#[command(version, about = "Talk to sensor boards attached over USB")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Show every attached board with its firmware and network status
    List,
    /// Manage the boards' Thread network
    #[command(subcommand)]
    Network(NetworkCommand),
    /// Manage the firmware of a board's radio coprocessor
    #[command(subcommand)]
    Coprocessor(CoprocessorCommand),
    /// Reboot a board into its ROM bootloader, to flash it over USB DFU
    Bootloader(Target),
    #[command(subcommand)]
    Sensor(SensorCommand),
    /// Take in the sensor reports that boards send over their network, through a board that stays
    /// attached
    #[command(subcommand)]
    Reports(ReportsCommand),
    /// Inspect a board's Prometheus metrics
    #[command(subcommand)]
    Metrics(MetricsCommand),
    /// Update a board's firmware without its ROM bootloader
    #[command(subcommand)]
    Firmware(FirmwareCommand),
    /// Show or change a board's configuration: which sensors it reads and how often, and how it is
    /// powered
    #[command(subcommand)]
    Config(ConfigCommand),
}

#[derive(Subcommand)]
enum ConfigCommand {
    /// Print a board's configuration
    Show(ConfigTarget),
    /// Change a board's configuration. Settings that are not given keep their values
    Set {
        #[command(flatten)]
        changes: config::Changes,

        #[command(flatten)]
        target: ConfigTarget,
    },
}

#[derive(Subcommand)]
enum FirmwareCommand {
    /// Show which build of the firmware a board runs, and where it stands with updates
    Status(Target),
    /// Send a board a firmware image and have it restart into it. The firmware it replaces comes
    /// back if the new one does not get the board back on its network
    Update {
        /// The image, as `just firmware-image` makes it
        image: PathBuf,

        #[command(flatten)]
        target: Target,
    },
    /// Update every board on the network: the attached board takes the image and offers it to
    /// the others, which fetch it over the network. The attached board is updated last
    Push {
        /// The image, as `just firmware-image` makes it
        image: PathBuf,

        #[command(flatten)]
        target: Target,
    },
}

#[derive(Subcommand)]
enum ReportsCommand {
    /// Print the reports as they arrive
    Watch(Target),
}

#[derive(Subcommand)]
enum MetricsCommand {
    /// Print a board's current metrics, in the text format it serves to Prometheus or pushes to
    /// a Pushgateway
    Show(Target),
}

#[derive(Subcommand)]
enum NetworkCommand {
    /// Put every attached board on one network
    ///
    /// The first time this is run, this creates a new network. Every subsequent time, it puts
    /// boards on the same network. To override this behavior and create a new network, pass
    /// `--force-reinit`
    Init {
        /// Make a new network, and move every attached board to it, whatever network was saved or
        /// the boards are on
        #[arg(long)]
        force_reinit: bool,

        /// The file the network's dataset is saved in. [default:
        /// $XDG_STATE_HOME/sensor-board/dataset]
        #[arg(long, value_name = "FILE")]
        dataset_path: Option<PathBuf>,
    },

    /// Leave the network and forget its dataset
    Leave(Target),
    /// Print the dataset of a board's network. It contains the network key
    Dataset(Target),
    /// Print a board's neighbor table: its children, and the routers it has a direct radio link
    /// with, other than its parent
    Neighbors(Target),
    /// Print a board's router table: every router on its network, and whether the board reaches it
    /// directly or through another router
    Routers(Target),
    /// Print a board's IPv6 addresses. Prometheus needs a routable one to scrape the board
    Addresses(Target),
}

#[derive(Subcommand)]
enum CoprocessorCommand {
    /// Show what a board's radio coprocessor has installed and is running
    Status(Target),

    /// Install one of ST's coprocessor images: a FUS update or a wireless stack. The coprocessor
    /// has to be running FUS, as it does on a new board. Without --board, installs on the only
    /// attached board that is in that state
    Install {
        /// The image, from STM32CubeWB's STM32WB_Copro_Wireless_Binaries
        image: PathBuf,

        #[command(flatten)]
        target: Target,
    },

    /// Remove the wireless stack, to install another in its place
    Uninstall(Target),
}

/// Commands to interact with sensors
#[derive(Subcommand)]
enum SensorCommand {
    /// Read the current value of a sensor
    Read {
        /// Which sensor to read
        #[arg(long)]
        sensor: Sensor,

        #[command(flatten)]
        target: Target,
    },
}

/// The board a command should target.
#[derive(Args)]
struct Target {
    /// The board's serial number, or the start of it. Only needed when several boards are attached
    #[arg(long, value_name = "SERIAL")]
    board: Option<String>,
}

impl Target {
    async fn board(&self) -> Result<Board> {
        Board::select(self.board.as_deref()).await
    }
}

/// The board that a `config` command applies to: an attached board, or a board that the attached
/// one reaches over its network.
#[derive(Args)]
struct ConfigTarget {
    /// The full serial number of a board on the attached board's network. The attached board
    /// relays the command to it
    #[arg(long, value_name = "SERIAL")]
    remote: Option<BoardId>,

    #[command(flatten)]
    attached: Target,
}

async fn list() -> Result<()> {
    let devices = board::devices().await?;
    if devices.is_empty() {
        bail!("no sensor board found on USB");
    }
    for device in &devices {
        // A board that cannot be talked to still belongs in the list.
        match summary(device).await {
            Ok(summary) => println!("{}  {summary}", board::serial_of(device)),
            Err(e) => println!("{}  {e}", board::serial_of(device)),
        }
    }
    Ok(())
}

async fn summary(device: &nusb::DeviceInfo) -> Result<String> {
    let board = Board::open(device).await?;
    let info = board.info().await?;
    let status = board.network_status().await?;
    Ok(format!(
        "built {}  {}",
        info.firmware_built,
        network::describe(&status)
    ))
}

#[tokio::main]
async fn main() -> Result<()> {
    match Cli::parse().command {
        Command::List => list().await,
        Command::Network(NetworkCommand::Init {
            force_reinit,
            dataset_path,
        }) => {
            let path = match dataset_path {
                Some(path) => path,
                None => SavedDataset::default_path()?,
            };
            network::init(&Board::all().await?, &SavedDataset::at(path), force_reinit).await
        }
        Command::Network(NetworkCommand::Leave(target)) => target.board().await?.leave().await,
        Command::Network(NetworkCommand::Dataset(target)) => {
            let board = target.board().await?;
            match board.dataset().await? {
                Some(dataset) => println!("{dataset}"),
                None => bail!("board {} has no network", board.serial()),
            }
            Ok(())
        }
        Command::Network(NetworkCommand::Neighbors(target)) => {
            let board = target.board().await?;
            let table = board.neighbors().await?;
            if table.neighbors.is_empty() {
                println!("Board {} has no neighbors.", board.serial());
            } else {
                print!("{}", network::describe_neighbors(&table));
            }
            if let NetworkStatus::Configured(Link {
                role: Role::Child,
                rloc16,
                ..
            }) = board.network_status().await?
            {
                println!(
                    "The board is a child of router {:#06x}, which the table leaves out.",
                    RouterId::of_rloc16(rloc16).rloc16()
                );
            }
            Ok(())
        }
        Command::Network(NetworkCommand::Routers(target)) => {
            let board = target.board().await?;
            let table = board.routers().await?;
            if table.routers.is_empty() {
                println!("Board {} knows of no routers.", board.serial());
            } else {
                print!("{}", network::describe_routers(&table));
            }
            Ok(())
        }
        Command::Coprocessor(CoprocessorCommand::Status(target)) => {
            let board = target.board().await?;
            let status = board.coprocessor_status().await?;
            println!("{}  {}", board.serial(), coprocessor::describe(&status));
            Ok(())
        }
        Command::Coprocessor(CoprocessorCommand::Install { image, target }) => {
            let bytes = std::fs::read(&image)
                .with_context(|| format!("could not read {}", image.display()))?;
            coprocessor::install(target.board.as_deref(), &bytes).await
        }
        Command::Coprocessor(CoprocessorCommand::Uninstall(target)) => {
            coprocessor::uninstall(target.board().await?).await
        }
        Command::Bootloader(target) => {
            let board = target.board().await?;
            board.enter_bootloader().await?;
            println!("Board {} is rebooting into its bootloader.", board.serial());
            Ok(())
        }
        Command::Firmware(FirmwareCommand::Status(target)) => {
            let board = target.board().await?;
            let info = board.info().await?;
            let status = board.firmware_status().await?;
            println!(
                "{}  build {} (built {})  {}",
                board.serial(),
                status.build,
                info.firmware_built,
                firmware::describe(&status.update)
            );
            Ok(())
        }
        Command::Firmware(FirmwareCommand::Update { image, target }) => {
            let image = firmware::Image::read(&image)?;
            firmware::update(target.board().await?, &image).await
        }
        Command::Firmware(FirmwareCommand::Push { image, target }) => {
            let image = firmware::Image::read(&image)?;
            firmware::push(target.board().await?, &image).await
        }
        Command::Network(NetworkCommand::Addresses(target)) => {
            let board = target.board().await?;
            let addresses = board.addresses().await?;
            if addresses.addresses.is_empty() {
                println!("Board {} has no addresses.", board.serial());
            } else {
                print!("{}", network::describe_addresses(&addresses));
            }
            Ok(())
        }
        Command::Reports(ReportsCommand::Watch(target)) => {
            reports::watch(target.board.as_deref()).await
        }
        Command::Metrics(MetricsCommand::Show(target)) => {
            print!("{}", target.board().await?.metrics().await?);
            Ok(())
        }
        Command::Sensor(SensorCommand::Read { sensor, target }) => {
            let board = target.board().await?;
            let SensorValue { value } = board.read_sensor(sensor).await??;
            println!("Sensor value: {value}");
            Ok(())
        }
        Command::Config(ConfigCommand::Show(target)) => {
            let board = target.attached.board().await?;
            config::show(&board, target.remote).await
        }
        Command::Config(ConfigCommand::Set { changes, target }) => {
            let board = target.attached.board().await?;
            config::set(&board, target.remote, &changes).await
        }
    }
}
