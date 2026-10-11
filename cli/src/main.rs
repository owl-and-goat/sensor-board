//! Command-line client for sensor boards attached over USB.

mod board;
mod border_router;
mod config;
mod coprocessor;
mod discovery;
mod firmware;
mod network;
mod reports;

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Args, Parser, Subcommand};

use board::Board;
use network::SavedDataset;
use protocol::{
    BoardId, HeaterDuration, HeaterPower, Link, NetworkStatus, Precision, Role, RouterId, Sensor,
    SensorValue, TempRh, TempRhReq,
};

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
    /// Receive the sensor reports that boards send over their network, through a board that
    /// stays attached
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
    /// An attached board receives the image and offers it to the others, which fetch it over
    /// the network. That board is updated last. With no board attached, this machine offers
    /// the image itself, through the border router. It then cannot tell which build each
    /// board runs, and reports the boards that fetched the image
    Push {
        /// The image, as built by `just firmware-image`
        image: PathBuf,

        /// The attached board to push through: its serial number, or the start of it. Without
        /// it, the first attached board that is on a network
        #[arg(long, value_name = "SERIAL")]
        board: Option<String>,

        /// Push through the border router whose REST API is at this address, or at
        /// cerberus:8081 if no address is given, even if a board is attached
        #[arg(
            long,
            value_name = "HOST:PORT",
            num_args = 0..=1,
            default_missing_value = border_router::DEFAULT_ADDRESS,
            conflicts_with = "board"
        )]
        border_router: Option<String>,
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
    /// Print a board's neighbor table: its children, and the routers it has a direct radio link
    /// with, other than its parent
    Neighbors(Target),
    /// Print a board's router table: every router on its network, and whether the board reaches it
    /// directly or through another router
    Routers(Target),
    /// Print a board's IPv6 addresses. Prometheus needs a routable one to scrape the board
    Addresses(Target),
    /// Show whether a board has registered its scrape endpoint with the border router, which
    /// then advertises it over mDNS for Prometheus to find
    Registration(Target),
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
    /// Measure temperature and relative humidity
    TempRh {
        /// How precisely to measure
        #[arg(long, default_value = "high", conflicts_with = "heat")]
        precision: Precision,

        /// Heat the sensor first, at this power, to drive off condensation. What is then measured,
        /// at high precision, is the heated sensor rather than the air
        #[arg(long, value_name = "POWER")]
        heat: Option<HeaterPower>,

        /// How long to heat the sensor for
        #[arg(
            long,
            value_name = "DURATION",
            default_value = "100ms",
            requires = "heat"
        )]
        heat_for: HeaterDuration,

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

/// The border router for `firmware push` to go through instead of an attached board, if any: the
/// one that the command names, or the default one if it names no board either and none is
/// attached.
async fn border_router_for(
    border_router: &Option<String>,
    board: &Option<String>,
) -> Result<Option<String>> {
    Ok(match border_router {
        Some(address) => Some(address.clone()),
        // With nothing attached, the border router is the only way.
        None if board.is_none() && board::devices().await?.is_empty() => {
            Some(border_router::DEFAULT_ADDRESS.to_owned())
        }
        None => None,
    })
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
        Command::Firmware(FirmwareCommand::Push {
            image,
            board,
            border_router,
        }) => {
            let image = firmware::Image::read(&image)?;
            match border_router_for(&border_router, &board).await? {
                Some(address) => firmware::push_through_border_router(&address, &image).await,
                None => firmware::push(firmware::gateway(board.as_deref()).await?, &image).await,
            }
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
        Command::Network(NetworkCommand::Show {
            border_router,
            target,
        }) => discovery::show(target.board.as_deref(), border_router.as_deref()).await,
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
            let (route, board) = target.route().await?;
            config::show(route, board).await
        }
        Command::Config(ConfigCommand::Set { changes, target }) => {
            let (route, board) = target.route().await?;
            config::set(route, board, &changes).await
        }
        Command::Sensor(SensorCommand::TempRh {
            precision,
            heat,
            heat_for,
            target,
        }) => {
            let req = match heat {
                None => TempRhReq::Plain(precision),
                Some(power) => TempRhReq::Heated {
                    power,
                    duration: heat_for,
                },
            };
            let board = target.board().await?;
            let TempRh {
                temperature,
                humidity,
            } = board.measure_temp_rh(req).await??;
            println!("Temperature: {temperature:.2} °C");
            println!("Humidity: {humidity:.2} %RH");
            Ok(())
        }
    }
}
