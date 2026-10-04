//! The USB device: one vendor interface with a pair of bulk endpoints, which
//! postcard-rpc frames its messages over.

use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{PA11, PA12, USB},
    usb,
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use postcard_rpc::server::impls::embassy_usb_v0_6::{
    PacketBuffers, USB_FS_MAX_PACKET_SIZE,
    dispatch_impl::{WireRxBuf, WireRxImpl, WireStorage, WireTxImpl},
};
use static_cell::ConstStaticCell;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<USB>;
});

pub type Driver = usb::Driver<'static, USB>;
pub type Device = embassy_usb::UsbDevice<'static, Driver>;
pub type Tx = WireTxImpl<ThreadModeRawMutex, Driver>;
pub type Rx = WireRxImpl<Driver>;

/// The message side of the device, for the RPC server to run on.
pub struct Link {
    pub tx: Tx,
    pub rx: Rx,
    pub rx_buf: WireRxBuf,
}

pub struct Builder {
    pub usb: Peri<'static, USB>,
    pub dp: Peri<'static, PA12>,
    pub dm: Peri<'static, PA11>,
}

impl Builder {
    /// Panics if called a second time: there is one USB peripheral, and one
    /// set of buffers for it.
    pub fn init(self) -> (Device, Link) {
        static STORAGE: WireStorage<ThreadModeRawMutex, Driver> = WireStorage::new();
        static PACKETS: ConstStaticCell<PacketBuffers> = ConstStaticCell::new(PacketBuffers::new());

        let mut config = embassy_usb::Config::new(protocol::USB_VID, protocol::USB_PID);
        config.manufacturer = Some("aspen");
        config.product = Some("sensor-board");
        // Chip UID as the USB serial, so boards can be told apart.
        config.serial_number = Some(embassy_stm32::uid::uid_hex());
        config.max_power = 100;

        let driver = usb::Driver::new(self.usb, Irqs, self.dp, self.dm);
        let packets = PACKETS.take();
        let (device, tx, rx) =
            STORAGE.init(driver, config, &mut packets.tx_buf, USB_FS_MAX_PACKET_SIZE);
        let link = Link {
            tx,
            rx,
            rx_buf: &mut packets.rx_buf,
        };
        (device, link)
    }
}
