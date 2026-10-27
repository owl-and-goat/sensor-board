//! The USB device: one vendor interface with a pair of bulk endpoints, which
//! carry postcard-rpc messages.

use core::fmt::Arguments;
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{PA11, PA12, USB},
    usb,
};

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use postcard_rpc::postcard;
use postcard_rpc::{
    header::{VarHeader, VarKeyKind},
    server::{
        WireTx, WireTxErrorKind,
        impls::embassy_usb_v0_6::{
            PacketBuffers, USB_FS_MAX_PACKET_SIZE,
            dispatch_impl::{WireRxBuf, WireRxImpl, WireStorage, WireTxImpl},
        },
    },
};
use serde::Serialize;
use static_cell::ConstStaticCell;

bind_interrupts!(struct Irqs {
    USB_LP => usb::InterruptHandler<USB>;
});

pub type Driver = usb::Driver<'static, USB>;
pub type Device = embassy_usb::UsbDevice<'static, Driver>;

/// The longest message sent to the host, with its header.
const MAX_MESSAGE: usize = 1024;

/// Sends postcard-rpc messages to the host. A message is encoded into one
/// buffer and sent from there, so the code that sends it over USB exists
/// once, and not once for every type of message, as in postcard-rpc's own
/// transmitter.
#[derive(Clone)]
pub struct Tx {
    usb: WireTxImpl<ThreadModeRawMutex, Driver>,
    message: &'static Mutex<ThreadModeRawMutex, [u8; MAX_MESSAGE]>,
}

impl WireTx for Tx {
    type Error = WireTxErrorKind;

    async fn wait_connection(&self) {
        self.usb.wait_connection().await
    }

    async fn send<T: Serialize + ?Sized>(
        &self,
        hdr: VarHeader,
        msg: &T,
    ) -> Result<(), Self::Error> {
        let mut message = self.message.lock().await;
        let len = encode(&mut message[..], hdr, msg).ok_or(WireTxErrorKind::Other)?;
        self.usb.send_raw(&message[..len]).await
    }

    async fn send_raw(&self, buf: &[u8]) -> Result<(), Self::Error> {
        self.usb.send_raw(buf).await
    }

    async fn send_log_str(&self, kkind: VarKeyKind, s: &str) -> Result<(), Self::Error> {
        self.usb.send_log_str(kkind, s).await
    }

    async fn send_log_fmt<'a>(
        &self,
        kkind: VarKeyKind,
        a: Arguments<'a>,
    ) -> Result<(), Self::Error> {
        self.usb.send_log_fmt(kkind, a).await
    }
}

/// Write `hdr` and `msg` into `buf`. Returns the length written, or `None`
/// if they don't fit.
fn encode<T: Serialize + ?Sized>(buf: &mut [u8], hdr: VarHeader, msg: &T) -> Option<usize> {
    let (head, rest) = hdr.write_to_slice(buf)?;
    let head = head.len();
    let body = postcard::to_slice(msg, rest).ok()?.len();
    Some(head + body)
}
pub type Rx = WireRxImpl<Driver>;

/// The postcard-rpc transport, for the RPC server.
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
        // Use the chip UID as the USB serial number, to tell boards apart.
        config.serial_number = Some(embassy_stm32::uid::uid_hex());
        config.max_power = 100;

        let driver = usb::Driver::new(self.usb, Irqs, self.dp, self.dm);
        let packets = PACKETS.take();
        let (device, usb, rx) =
            STORAGE.init(driver, config, &mut packets.tx_buf, USB_FS_MAX_PACKET_SIZE);
        static MESSAGE: Mutex<ThreadModeRawMutex, [u8; MAX_MESSAGE]> = Mutex::new([0; MAX_MESSAGE]);
        let link = Link {
            tx: Tx {
                usb,
                message: &MESSAGE,
            },
            rx,
            rx_buf: &mut packets.rx_buf,
        };
        (device, link)
    }
}
