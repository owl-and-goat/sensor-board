//! Command-line client for sensor boards attached over USB.

mod board;
mod border_router;
mod config;
mod coprocessor;
mod discovery;
mod firmware;
mod mdns;
mod network;

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand};

use board::Board;
use network::SavedDataset;
use protocol::{BoardId, BuildCommit};

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
    Show {
        /// Print the configuration as the board stores it in flash, as a Rust array. A test can
        /// use it to check that a later layout of the configuration still reads it
        #[arg(long)]
        stored: bool,

        #[command(flatten)]
        target: ConfigTarget,
    },
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
    /// Show which firmware build a board runs, and its update status
    Status(Target),
    /// Send a board a firmware image and restart it into the image. The board rolls back to
    /// its previous firmware if the new one does not get it back on its network
    Update {
        /// The image, as built by `just firmware-image`
        image: PathBuf,

        #[command(flatten)]
        target: Target,
    },
    /// Update every board on the network
    ///
    /// This machine offers the image to every device that the border router lists, and to every
    /// board it finds over mDNS. Those that run another build fetch it over the network and
    /// restart into it. It shows the build each board runs, and reports which boards kept the
    /// image and which rolled it back
    Push {
        /// The image, as built by `just firmware-image`
        image: PathBuf,

        /// The border router whose REST API is at this address
        #[arg(long, value_name = "HOST:PORT", default_value = border_router::DEFAULT_ADDRESS)]
        border_router: String,
    },
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
        /// Create a new network and move every attached board to it, regardless of the saved
        /// network and the networks the boards are on
        #[arg(long)]
        force_reinit: bool,

        /// The file the network's dataset is saved in. [default:
        /// $XDG_STATE_HOME/sensor-board/dataset]
        #[arg(long, value_name = "FILE")]
        dataset_path: Option<PathBuf>,
    },

    /// Leave the network and erase its dataset from the board
    Leave(Target),
    /// Print the dataset of a board's network. It contains the network key
    Dataset(Target),
    /// Print a board's IPv6 addresses. Prometheus needs a routable one to scrape the board
    Addresses(Target),
    /// Show whether a board has registered its scrape endpoint with the border router, which
    /// then advertises it over mDNS for Prometheus to find
    Registration(Target),
    /// Show how a board takes part in its network: as a full Thread device, or, on battery, as
    /// a sleepy end device that keeps its receiver off
    Mode(Target),
    /// List the sensor boards on the network: serial number, RLOC16, IPv6 address and link
    ///
    /// Asks through an attached board if there is one, and otherwise through the border router.
    /// Age, quality and RSSI describe the link between each board and the board or border
    /// router that was asked. They show as `-` for a board it has no direct radio link with, and
    /// a border router only reports the quality. A board whose firmware predates discovery is
    /// not listed
    Show {
        /// Ask through the border router whose REST API is at this address, or at cerberus:8081
        /// if no address is given
        #[arg(
            long,
            value_name = "HOST:PORT",
            num_args = 0..=1,
            default_missing_value = border_router::DEFAULT_ADDRESS,
            conflicts_with = "board"
        )]
        border_router: Option<String>,

        #[command(flatten)]
        target: Target,
    },
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

    /// Remove the wireless stack, so that another can be installed
    Uninstall(Target),
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

/// The board that a `config` command applies to, attached or not.
#[derive(Args)]
struct ConfigTarget {
    /// The board's serial number. The start of it is enough for an attached board, and it can
    /// be left out if only one board is attached. A board that is not attached is reached
    /// through an attached board that is on a network, or else through the border router
    #[arg(long, value_name = "SERIAL")]
    board: Option<String>,

    /// Reach the board through the border router whose REST API is at this address, or at
    /// cerberus:8081 if no address is given, whatever is attached
    #[arg(
        long,
        value_name = "HOST:PORT",
        num_args = 0..=1,
        default_missing_value = border_router::DEFAULT_ADDRESS
    )]
    border_router: Option<String>,
}

impl ConfigTarget {
    /// The way to reach the board, and the board.
    async fn route(&self) -> Result<(config::Route, BoardId)> {
        config::route(self.board.as_deref(), self.border_router.as_deref()).await
    }
}

async fn list() -> Result<()> {
    let devices = board::devices().await?;
    if devices.is_empty() {
        bail!("no sensor board found on USB");
    }
    for device in &devices {
        // List a board even if it does not respond.
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
    let commit = board.commit().await;
    let status = board.network_status().await?;
    Ok(format!(
        "built {}{}  {}",
        info.firmware_built,
        describe_commit(commit),
        network::describe(&status)
    ))
}

/// " from <commit>", or nothing for a firmware that does not report its
/// commit.
fn describe_commit(commit: Option<BuildCommit>) -> String {
    commit
        .map(|commit| format!(" from {commit}"))
        .unwrap_or_default()
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
            let commit = board.commit().await;
            let status = board.firmware_status().await?;
            println!(
                "{}  build {} (built {}{})  {}",
                board.serial(),
                status.build,
                info.firmware_built,
                describe_commit(commit),
                firmware::describe(&status.update)
            );
            Ok(())
        }
        Command::Firmware(FirmwareCommand::Update { image, target }) => {
            let image = firmware::Image::read(&image)?;
            firmware::update(target.board().await?, &image).await
        }
        Command::Firmware(FirmwareCommand::Push {
            image,
            border_router,
        }) => {
            let image = firmware::Image::read(&image)?;
            firmware::push_through_border_router(&border_router, &image).await
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
        Command::Network(NetworkCommand::Registration(target)) => {
            let board = target.board().await?;
            let registration = board.registration().await?;
            println!(
                "{}  {}",
                board.serial(),
                network::describe_registration(board.serial(), &registration)
            );
            Ok(())
        }
        Command::Network(NetworkCommand::Mode(target)) => {
            let board = target.board().await?;
            let mode = board.device_mode().await?;
            println!(
                "{}  {}",
                board.serial(),
                network::describe_device_mode(mode)
            );
            Ok(())
        }
        Command::Network(NetworkCommand::Show {
            border_router,
            target,
        }) => discovery::show(target.board.as_deref(), border_router.as_deref()).await,
        Command::Metrics(MetricsCommand::Show(target)) => {
            print!("{}", target.board().await?.metrics().await?);
            Ok(())
        }
        Command::Config(ConfigCommand::Show { stored, target }) => {
            let (route, board) = target.route().await?;
            config::show(route, board, stored).await
        }
        Command::Config(ConfigCommand::Set { changes, target }) => {
            let (route, board) = target.route().await?;
            config::set(route, board, &changes).await
        }
    }
}
