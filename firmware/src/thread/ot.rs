//! OpenThread API calls and error type. Translated (sort of) from the original
//! C API. See
//! [here](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v1.24.0/thread/openthread/core/openthread_api/).

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::net::Ipv6Addr;
use core::ptr::{copy_nonoverlapping, read_volatile, write_volatile};

use embassy_futures::block_on;
use embassy_stm32_wpan::sub::thread::{OtNotification, ThreadNotifRx, ThreadOt};
use protocol::{Dataset, ExtAddress, Neighbor, NeighborKind, OtError, Role, Route, RouterId};

// TODO: move ffi module to a submodule of this one?
use super::ffi;

use ffi::MsgId_M0toM4_Enum_t as ffi_notification;
use ffi::MsgId_M4toM0_Enum_t as ffi_command;

/// `Ok(())` is `OT_ERROR_NONE`.
pub type Result<T> = core::result::Result<T, OtError>;

fn check(raw: u32) -> Result<()> {
    let e = ffi::otError(raw as _);
    match e {
        ffi::otError::OT_ERROR_NONE => Ok(()),
        ffi::otError::OT_ERROR_FAILED => Err(OtError::Failed),
        ffi::otError::OT_ERROR_DROP => Err(OtError::Drop),
        ffi::otError::OT_ERROR_NO_BUFS => Err(OtError::NoBufs),
        ffi::otError::OT_ERROR_NO_ROUTE => Err(OtError::NoRoute),
        ffi::otError::OT_ERROR_BUSY => Err(OtError::Busy),
        ffi::otError::OT_ERROR_PARSE => Err(OtError::Parse),
        ffi::otError::OT_ERROR_INVALID_ARGS => Err(OtError::InvalidArgs),
        ffi::otError::OT_ERROR_SECURITY => Err(OtError::Security),
        ffi::otError::OT_ERROR_ABORT => Err(OtError::Abort),
        ffi::otError::OT_ERROR_NOT_IMPLEMENTED => Err(OtError::NotImplemented),
        ffi::otError::OT_ERROR_INVALID_STATE => Err(OtError::InvalidState),
        ffi::otError::OT_ERROR_NO_ACK => Err(OtError::NoAck),
        ffi::otError::OT_ERROR_DETACHED => Err(OtError::Detached),
        ffi::otError::OT_ERROR_NOT_FOUND => Err(OtError::NotFound),
        ffi::otError::OT_ERROR_ALREADY => Err(OtError::Already),
        other => Err(OtError::Other(other.0)),
    }
}

fn role_from_raw(raw: u32) -> Role {
    match ffi::otDeviceRole(raw as _) {
        ffi::otDeviceRole::OT_DEVICE_ROLE_DISABLED => Role::Disabled,
        ffi::otDeviceRole::OT_DEVICE_ROLE_DETACHED => Role::Detached,
        ffi::otDeviceRole::OT_DEVICE_ROLE_CHILD => Role::Child,
        ffi::otDeviceRole::OT_DEVICE_ROLE_ROUTER => Role::Router,
        ffi::otDeviceRole::OT_DEVICE_ROLE_LEADER => Role::Leader,
        other => Role::Other(other.0),
    }
}

fn neighbor_from_raw(info: &ffi::otNeighborInfo) -> Neighbor {
    Neighbor {
        kind: if info.mIsChild() {
            NeighborKind::Child
        } else {
            NeighborKind::Router
        },
        rloc16: info.mRloc16,
        ext_address: ExtAddress(info.mExtAddress.m8),
        age_secs: info.mAge,
        link_quality_in: info.mLinkQualityIn,
        average_rssi: info.mAverageRssi,
        last_rssi: info.mLastRssi,
        link_margin: info.mLinkMargin,
    }
}

/// What `otThreadGetNextHopAndPathCost` answered about `destination`, an
/// RLOC16.
fn route_from_raw(destination: u16, next_hop: u16, cost: u8) -> Route {
    /// The stack's "no next hop".
    const INVALID_RLOC16: u16 = 0xfffe;
    /// The path cost it gives to what it has no way to. A child still names
    /// its parent as the next hop then, as it does for everything.
    const MAX_ROUTE_COST: u8 = 16;

    if next_hop == INVALID_RLOC16 || cost >= MAX_ROUTE_COST {
        Route::Unreachable
    } else if next_hop != destination {
        Route::Relayed {
            next_hop: RouterId::of_rloc16(next_hop),
            cost,
        }
    } else if cost == 0 {
        // No link costs nothing: this is the stack's answer about the device
        // it runs on.
        Route::ThisBoard
    } else {
        Route::Direct { cost }
    }
}

/// CPU1 memory that CPU2 is given a pointer to, and only reads: the dataset
/// argument of `otDatasetSetActiveTlvs`. It has to be handed over as
/// `&'static`, because CPU2 keeps reading until it has answered the call,
/// whatever has become of the future that made it.
pub struct DatasetBuffer(UnsafeCell<MaybeUninit<ffi::otOperationalDatasetTlvs>>);

impl DatasetBuffer {
    pub const fn new() -> Self {
        DatasetBuffer(UnsafeCell::new(MaybeUninit::zeroed()))
    }
}

/// CPU1 memory that CPU2 is given pointers to, and writes: the two out
/// arguments of `otThreadGetNextNeighborInfo`. `&'static` for the same reason
/// as [`DatasetBuffer`], and more so: CPU2 writes here until it has answered.
pub struct NeighborBuffer {
    iterator: UnsafeCell<ffi::otNeighborInfoIterator>,
    info: UnsafeCell<MaybeUninit<ffi::otNeighborInfo>>,
}

impl NeighborBuffer {
    pub const fn new() -> Self {
        NeighborBuffer {
            iterator: UnsafeCell::new(NeighborIterator::INIT.0),
            info: UnsafeCell::new(MaybeUninit::zeroed()),
        }
    }
}

/// CPU1 memory that CPU2 is given pointers to, and writes: the two out
/// arguments of `otThreadGetNextHopAndPathCost`. `&'static` for the same
/// reason as [`NeighborBuffer`].
pub struct NextHopBuffer {
    next_hop_rloc16: UnsafeCell<u16>,
    path_cost: UnsafeCell<u8>,
}

impl NextHopBuffer {
    pub const fn new() -> Self {
        NextHopBuffer {
            next_hop_rloc16: UnsafeCell::new(0),
            path_cost: UnsafeCell::new(0),
        }
    }
}

/// CPU1 memory for the UDP socket: what the `otUdp` and `otMessage` calls
/// point CPU2 at. The socket itself CPU2 links into its list of sockets and
/// goes on using, from `otUdpOpen` until `otUdpClose`, so this buffer more
/// than any has to be `&'static`. The rest CPU2 reads or writes within one
/// call.
pub struct UdpBuffer {
    socket: UnsafeCell<MaybeUninit<ffi::otUdpSocket>>,
    name: UnsafeCell<MaybeUninit<ffi::otSockAddr>>,
    peer: UnsafeCell<MaybeUninit<ffi::otMessageInfo>>,
    payload: UnsafeCell<[u8; MAX_DATAGRAM_LEN]>,
}

impl UdpBuffer {
    pub const fn new() -> Self {
        UdpBuffer {
            socket: UnsafeCell::new(MaybeUninit::zeroed()),
            name: UnsafeCell::new(MaybeUninit::zeroed()),
            peer: UnsafeCell::new(MaybeUninit::zeroed()),
            payload: UnsafeCell::new([0; MAX_DATAGRAM_LEN]),
        }
    }
}

/// The longest payload the UDP socket sends or takes in.
pub const MAX_DATAGRAM_LEN: usize = 256;

/// The payload of a UDP datagram.
pub type Datagram = heapless::Vec<u8, MAX_DATAGRAM_LEN>;

fn ip6_address(address: Ipv6Addr) -> ffi::otIp6Address {
    ffi::otIp6Address {
        mFields: ffi::otIp6Address__bindgen_ty_1 {
            m8: address.octets(),
        },
    }
}

/// A callback from the stack, as far as the firmware has a use for it.
pub enum Notification {
    /// The device's role, one of its addresses or the like has changed.
    StateChanged,
    /// A datagram has arrived on the UDP socket: its payload.
    UdpReceived(Datagram),
    /// Anything else, by its ID, and a datagram that was too long to take.
    Other(u32),
}

/// Wait for the stack's next callback.
///
/// CPU2 is inside that callback until its notification is acknowledged, and
/// the message a datagram arrives in is only there that long. So the payload
/// is read out before the acknowledgement, which is why this takes `ot`, and
/// with calls that are blocked on, as nothing can be awaited at that point.
/// CPU2 answers them in well under a millisecond.
pub async fn notification(
    notif_rx: &mut ThreadNotifRx<'_>,
    ot: &mut ThreadOt<'_>,
    buffer: &'static UdpBuffer,
) -> Notification {
    const STATE_CHANGE: u32 = ffi_notification::MSG_M0TOM4_NOTIFY_STATE_CHANGE as u32;
    const UDP_RECEIVE: u32 = ffi_notification::MSG_M0TOM4_UDP_RECEIVE as u32;

    let acknowledged_after = |raw: OtNotification| match raw.id {
        STATE_CHANGE => Notification::StateChanged,
        UDP_RECEIVE => {
            // The arguments of the socket's receive callback: its context,
            // the message, and the message's otMessageInfo.
            let message = raw.data[1];
            let datagram = unsafe {
                // SAFETY: the notification that brought the message is not
                // acknowledged until this closure returns.
                block_on(udp_read(ot, buffer, message))
            };
            match datagram {
                Some(datagram) => Notification::UdpReceived(datagram),
                None => Notification::Other(raw.id),
            }
        }
        other => Notification::Other(other),
    };
    notif_rx.receive_with(acknowledged_after).await
}

/// The payload of the datagram in `message`, a `const otMessage *`. `None`
/// if it is longer than a [`Datagram`].
///
/// SAFETY: `message` has to be one that CPU2 still holds: the argument of a
/// receive callback whose notification has not been acknowledged yet.
async unsafe fn udp_read(
    ot: &mut impl OpenThread,
    buffer: &'static UdpBuffer,
    message: u32,
) -> Option<Datagram> {
    let payload = buffer.payload.get().cast::<u8>();
    unsafe {
        // SAFETY: both expect a pointer to an otMessage. The message is the
        // whole packet, and its offset is where the UDP payload starts.
        let length = ot
            .ffi_call(ffi_command::MSG_M4TOM0_OT_MESSAGE_GET_LENGTH, &[message])
            .await as u16;
        let offset = ot
            .ffi_call(ffi_command::MSG_M4TOM0_OT_MESSAGE_GET_OFFSET, &[message])
            .await as u16;
        let len = usize::from(length.checked_sub(offset)?);
        if len > MAX_DATAGRAM_LEN {
            return None;
        }

        // SAFETY: expects the message, an offset into it, a pointer to write
        // to and the number of bytes to write there at most, and answers
        // with the number it wrote. The payload buffer has room for `len`,
        // and CPU1 touches it only here and in `udp_send`, never through a
        // reference and never while CPU2 has a call to answer.
        let read = ot
            .ffi_call(
                ffi_command::MSG_M4TOM0_OT_MESSAGE_READ,
                &[message, offset as u32, payload as u32, len as u32],
            )
            .await as usize;

        let mut datagram = Datagram::new();
        datagram.resize_default(read.min(len)).ok()?;
        copy_nonoverlapping(payload, datagram.as_mut_ptr(), datagram.len());
        Some(datagram)
    }
}

/// A place in the stack's neighbor table (`otNeighborInfoIterator`).
#[derive(Clone, Copy)]
pub struct NeighborIterator(ffi::otNeighborInfoIterator);

impl NeighborIterator {
    /// Before the first entry (`OT_NEIGHBOR_INFO_ITERATOR_INIT`).
    pub const INIT: Self = NeighborIterator(0);
}

/// Trait for stuff that supports OpenThread's API, based around its
/// message-based FFI interface. I don't know if this is the *right* way to do
/// this, but it makes using the types relatively easy.
pub unsafe trait OpenThread {
    /// SAFETY: It strikes me as very possible to pass horrible arguments to all of
    /// these functions, or the wrong number of arguments in `args`.
    ///
    /// If you want to know what the actual arguments are, I *think* the way to
    /// do it is to look at the equivalent OpenThread call in ST's checkout of
    /// its API. For example, `MSG_M4TOM0_OT_RADIO_SET_TRANSMIT_POWER` is used
    /// [here](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v1.24.0/thread/openthread/core/openthread_api/radio.c#L85),
    /// which initializes one `uint32_t` argument (the power) in its body.
    async unsafe fn ffi_call(&mut self, cmd: ffi_command::Type, args: &[u32]) -> u32;

    /// The same as `ffi_call()` but with return-value conversion to `Error`,
    /// since we do this a lot.
    async unsafe fn ffi_try(&mut self, cmd: ffi_command::Type, args: &[u32]) -> Result<()> {
        check(unsafe { self.ffi_call(cmd, args).await })
    }

    /// Initialize the Thread stack.
    async fn instance_init_single(&mut self) {
        unsafe {
            // SAFETY: expects no arguments. What comes back is the
            // `otInstance` pointer, not an error; every other call leaves the
            // instance implicit, so there is no use for it.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_INSTANCE_INIT_SINGLE, &[])
                .await;
        }
    }

    /// Unset the STATE_CHANGED callback, so that state-change events
    /// come through the mailbox instead.
    ///
    /// FIXME: should probably take a callback in case we do actually want to set
    /// one.
    /// TODO: who receives those events, anyway?
    async fn set_state_changed_callback(&mut self) -> Result<()> {
        unsafe {
            // SAFETY: expects a single (nullable) pointer argument.
            self.ffi_try(ffi_command::MSG_M4TOM0_OT_SET_STATE_CHANGED_CALLBACK, &[0])
                .await
        }
    }

    async fn ip6_set_enabled(&mut self, enabled: bool) -> Result<()> {
        unsafe {
            // SAFETY: expects one numeric boolean, which this conversion will produce
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_IP6_SET_ENABLED,
                &[enabled as u32],
            )
            .await
        }
    }

    async fn thread_set_enabled(&mut self, enabled: bool) -> Result<()> {
        unsafe {
            // SAFETY: expects one numeric boolean, which this conversion will produce
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_THREAD_SET_ENABLED,
                &[enabled as u32],
            )
            .await
        }
    }

    async fn thread_get_device_role(&mut self) -> Role {
        let role = unsafe {
            // SAFETY: expects no arguments.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE, &[])
                .await
        };

        role_from_raw(role)
    }

    async fn thread_get_rloc16(&mut self) -> u16 {
        unsafe {
            // SAFETY: expects no arguments.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_THREAD_GET_RLOC_16, &[])
                .await as u16
        }
    }

    async fn link_get_channel(&mut self) -> u8 {
        unsafe {
            // SAFETY: expects no arguments.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_LINK_GET_CHANNEL, &[])
                .await as u8
        }
    }

    async fn link_get_panid(&mut self) -> u16 {
        unsafe {
            // SAFETY: expects no arguments.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_LINK_GET_PANID, &[])
                .await as u16
        }
    }

    /// Make `dataset` the active operational dataset: the network the stack
    /// attaches to when it is enabled.
    async fn dataset_set_active_tlvs(
        &mut self,
        buffer: &'static DatasetBuffer,
        dataset: &Dataset,
    ) -> Result<()> {
        let tlvs = dataset.as_tlvs();
        let mut arg = ffi::otOperationalDatasetTlvs {
            mTlvs: [0; Dataset::MAX_LEN],
            mLength: tlvs.len() as u8,
        };
        arg.mTlvs[..tlvs.len()].copy_from_slice(tlvs);

        let buffer = buffer.0.get().cast::<ffi::otOperationalDatasetTlvs>();
        unsafe {
            // SAFETY: this is the only place CPU1 touches what is in the
            // buffer, never through a reference, and CPU2 only reads it. If an
            // earlier call was dropped while CPU2 was still reading, that
            // abandoned call may see a mix of the two datasets (each with a
            // length that is in bounds); the call below queues up behind it
            // and installs the dataset that was asked for.
            write_volatile(buffer, arg);
            // SAFETY: expects one pointer to an otOperationalDatasetTlvs,
            // which stays valid for as long as CPU2 could read it.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_DATASET_SET_ACTIVE_TLVS,
                &[buffer as u32],
            )
            .await
        }
    }

    /// The entry of the neighbor table that comes after `iterator`, which is
    /// moved on past it. `None` once the table has been gone through.
    async fn thread_get_next_neighbor_info(
        &mut self,
        buffer: &'static NeighborBuffer,
        iterator: &mut NeighborIterator,
    ) -> Result<Option<Neighbor>> {
        let iterator_out = buffer.iterator.get();
        let info_out = buffer.info.get().cast::<ffi::otNeighborInfo>();

        let found = unsafe {
            // SAFETY: this function is the only place CPU1 touches what is in
            // the buffer, never through a reference. If an earlier call was
            // dropped while CPU2 was still working on it, that abandoned call
            // may overwrite the iterator written here before the call below,
            // queued up behind it, is looked at: an entry of the table is
            // then skipped or repeated, and nothing worse.
            write_volatile(iterator_out, iterator.0);
            // SAFETY: expects a pointer to an otNeighborInfoIterator, which
            // it reads and writes, and one to an otNeighborInfo, which it
            // writes. Both stay valid for as long as CPU2 could use them.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_THREAD_GET_NEXT_NEIGHBOR_INFO,
                &[iterator_out as u32, info_out as u32],
            )
            .await
        };
        match found {
            Ok(()) => {}
            Err(OtError::NotFound) => return Ok(None),
            Err(e) => return Err(e),
        }

        let info = unsafe {
            // SAFETY: CPU2 has answered, so it has done its writing. Both are
            // plain numbers, valid whatever they hold, and the buffer started
            // out zeroed.
            iterator.0 = read_volatile(iterator_out);
            read_volatile(info_out)
        };
        Ok(Some(neighbor_from_raw(&info)))
    }

    /// Whether a router on the network has this ID.
    async fn thread_is_router_id_allocated(&mut self, id: RouterId) -> bool {
        let allocated = unsafe {
            // SAFETY: expects one argument, the router ID.
            self.ffi_call(
                ffi_command::MSG_M4TOM0_OT_THREAD_IS_ROUTER_ID_ALLOCATED,
                &[id.0 as u32],
            )
            .await
        };
        allocated != 0
    }

    /// What the stack does with a message for the router that has this ID.
    async fn thread_get_next_hop_and_path_cost(
        &mut self,
        buffer: &'static NextHopBuffer,
        destination: RouterId,
    ) -> Route {
        let destination = destination.rloc16();
        let next_hop_out = buffer.next_hop_rloc16.get();
        let path_cost_out = buffer.path_cost.get();

        let (next_hop, cost) = unsafe {
            // SAFETY: expects the destination's RLOC16 and two pointers that
            // it writes through, to a uint16_t and to a uint8_t. Both stay
            // valid for as long as CPU2 could use them. (The name of the
            // message is ST's spelling.)
            self.ffi_call(
                ffi_command::MSG_M4TOM0_OT_THREAD_GET_NEXT_HOP_AND_PAST_COST,
                &[
                    destination as u32,
                    next_hop_out as u32,
                    path_cost_out as u32,
                ],
            )
            .await;
            // SAFETY: CPU2 has answered, so it has done its writing, and this
            // function is the only place CPU1 touches what is in the buffer,
            // never through a reference. An earlier call that was dropped
            // has been answered before this one, so what is here now is this
            // one's.
            (read_volatile(next_hop_out), read_volatile(path_cost_out))
        };
        route_from_raw(destination, next_hop, cost)
    }

    /// Open the UDP socket. It can send from then on.
    async fn udp_open(&mut self, buffer: &'static UdpBuffer) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to an otUdpSocket, which CPU2 fills
            // in and keeps until the socket is closed, and the context of
            // its receive callback, which nothing here has a use for. The
            // callback is no argument: its calls arrive as notifications.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_UDP_OPEN,
                &[buffer.socket.get() as u32, 0],
            )
            .await
        }
    }

    async fn udp_close(&mut self, buffer: &'static UdpBuffer) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to an open otUdpSocket.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_UDP_CLOSE,
                &[buffer.socket.get() as u32],
            )
            .await
        }
    }

    /// Have the open socket receive what is sent to `port`, on any address
    /// of the device.
    async fn udp_bind(&mut self, buffer: &'static UdpBuffer, port: u16) -> Result<()> {
        let name = ffi::otSockAddr {
            mAddress: ip6_address(Ipv6Addr::UNSPECIFIED),
            mPort: port,
        };
        let name_in = buffer.name.get().cast::<ffi::otSockAddr>();
        unsafe {
            // SAFETY: this is the only place CPU1 touches the name in the
            // buffer, never through a reference, and CPU2 only reads it.
            write_volatile(name_in, name);
            // SAFETY: expects pointers to an open otUdpSocket and to an
            // otSockAddr, which it copies, and an otNetifIdentifier.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_UDP_BIND,
                &[
                    buffer.socket.get() as u32,
                    name_in as u32,
                    ffi::otNetifIdentifier::OT_NETIF_THREAD_HOST.0 as u32,
                ],
            )
            .await
        }
    }

    /// Send `payload` from the open socket to `port` at `address`. Of what
    /// goes to a multicast address, a copy comes back to the device itself.
    async fn udp_send(
        &mut self,
        buffer: &'static UdpBuffer,
        address: Ipv6Addr,
        port: u16,
        payload: &Datagram,
    ) -> Result<()> {
        // SAFETY: the fields of an otMessageInfo are numbers, all of them
        // valid as zero: the default source address, port and hop limit.
        let mut peer: ffi::otMessageInfo = unsafe { MaybeUninit::zeroed().assume_init() };
        peer.mPeerAddr = ip6_address(address);
        peer.mPeerPort = port;
        peer.set_mMulticastLoop(true);
        let peer_in = buffer.peer.get().cast::<ffi::otMessageInfo>();
        let payload_in = buffer.payload.get().cast::<u8>();

        let message = unsafe {
            // SAFETY: expects a pointer to an otMessageSettings, or null for
            // the default ones. It answers with a pointer to a new
            // otMessage, null if the stack has no buffer left for one.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_UDP_NEW_MESSAGE, &[0])
                .await
        };
        if message == 0 {
            return Err(OtError::NoBufs);
        }

        let sent = unsafe {
            // SAFETY: CPU1 touches the payload and the peer in the buffer
            // only here and in `udp_read`, never through a reference and
            // never while CPU2 has a call to answer. A `Datagram` is no
            // longer than the payload buffer.
            copy_nonoverlapping(payload.as_ptr(), payload_in, payload.len());
            write_volatile(peer_in, peer);
            // SAFETY: expects the message, a pointer to bytes that it copies
            // onto the end of it, and how many of them there are.
            let appended = self
                .ffi_try(
                    ffi_command::MSG_M4TOM0_OT_MESSAGE_APPEND,
                    &[message, payload_in as u32, payload.len() as u32],
                )
                .await;
            match appended {
                // SAFETY: expects pointers to an open otUdpSocket, to the
                // message, and to an otMessageInfo, which it copies.
                Ok(()) => {
                    self.ffi_try(
                        ffi_command::MSG_M4TOM0_OT_UDP_SEND,
                        &[buffer.socket.get() as u32, message, peer_in as u32],
                    )
                    .await
                }
                Err(e) => Err(e),
            }
        };
        if sent.is_err() {
            // The stack takes over a message that it accepts for sending.
            // This one it did not.
            unsafe {
                // SAFETY: expects a pointer to an otMessage that is still
                // the caller's.
                self.ffi_call(ffi_command::MSG_M4TOM0_OT_MESSAGE_FREE, &[message])
                    .await;
            }
        }
        sent
    }

    /// Set the radio's transmit power, in dBm. The STM32WB55's radio covers
    /// -40 to +6.
    #[expect(dead_code, reason = "nothing sets the transmit power yet")]
    async fn plat_radio_set_transmit_power(&mut self, dbm: i8) -> Result<()> {
        unsafe {
            // SAFETY: expects one argument, the power as a signed byte widened
            // to a word, which is what the cast of a negative `i8` gives.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_RADIO_SET_TRANSMIT_POWER,
                &[dbm as u32],
            )
            .await
        }
    }

    // TODO: the rest of the OpenThread API
}

unsafe impl OpenThread for ThreadOt<'_> {
    /// TODO: propagate (un)safety down into `ThreadOt`.
    async unsafe fn ffi_call(&mut self, cmd: ffi_command::Type, args: &[u32]) -> u32 {
        self.call(cmd as u32, args).await
    }
}
