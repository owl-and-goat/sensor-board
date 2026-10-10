//! OpenThread API calls and error type. Translated (sort of) from the original
//! C API. See
//! [here](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v1.24.0/thread/openthread/core/openthread_api/).

use core::cell::{Cell, UnsafeCell};
use core::mem::{MaybeUninit, transmute_copy};
use core::net::{Ipv6Addr, SocketAddrV6};
use core::ptr::{copy_nonoverlapping, null, null_mut, read_volatile, write_volatile};

use embassy_futures::{
    block_on,
    select::{Either, select},
};
use embassy_stm32_wpan::sub::thread::{OtNotification, ThreadNotifRx, ThreadOt};
use protocol::{
    Address, AddressKind, Addresses, Dataset, ExtAddress, Neighbor, NeighborKind, OtError, Role,
    Route, RouterId,
};

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

/// How much of what arrives over TCP CPU2 keeps until it is taken: what
/// OpenThread recommends for a connection that goes a few hops.
const TCP_RECEIVE_LEN: usize = ffi::OT_TCP_RECEIVE_BUFFER_SIZE_FEW_HOPS as usize;

/// The most bytes of a TCP connection that are sent, or taken, at a time.
pub const TCP_CHUNK_LEN: usize = 512;

/// A piece of what a TCP connection carries.
pub type TcpChunk = heapless::Vec<u8, TCP_CHUNK_LEN>;

/// CPU1 memory for TCP: an endpoint, which is one connection at a time, and
/// the listener that takes connections into it. CPU2 links both into its
/// lists when they are set up and goes on using them, along with the room
/// for what arrives, so this has to be `&'static` for the same reason as
/// [`UdpBuffer`]. What is being sent CPU2 reads until the peer has
/// acknowledged it. The rest it reads or writes within one call.
pub struct TcpBuffer {
    endpoint: UnsafeCell<MaybeUninit<ffi::otTcpEndpoint>>,
    endpoint_args: UnsafeCell<MaybeUninit<ffi::otTcpEndpointInitializeArgs>>,
    listener: UnsafeCell<MaybeUninit<ffi::otTcpListener>>,
    listener_args: UnsafeCell<MaybeUninit<ffi::otTcpListenerInitializeArgs>>,
    /// The address that is listened on, or connected to.
    name: UnsafeCell<MaybeUninit<ffi::otSockAddr>>,
    received: UnsafeCell<[u8; TCP_RECEIVE_LEN]>,
    /// Where `otTcpReceiveByReference` answers: with a pointer to the
    /// endpoint's own links to what has arrived.
    received_link: UnsafeCell<*const ffi::otLinkedBuffer>,
    sending: UnsafeCell<[u8; TCP_CHUNK_LEN]>,
    sending_link: UnsafeCell<MaybeUninit<ffi::otLinkedBuffer>>,
}

impl TcpBuffer {
    pub const fn new() -> Self {
        TcpBuffer {
            endpoint: UnsafeCell::new(MaybeUninit::zeroed()),
            endpoint_args: UnsafeCell::new(MaybeUninit::zeroed()),
            listener: UnsafeCell::new(MaybeUninit::zeroed()),
            listener_args: UnsafeCell::new(MaybeUninit::zeroed()),
            name: UnsafeCell::new(MaybeUninit::zeroed()),
            received: UnsafeCell::new([0; TCP_RECEIVE_LEN]),
            received_link: UnsafeCell::new(null()),
            sending: UnsafeCell::new([0; TCP_CHUNK_LEN]),
            sending_link: UnsafeCell::new(MaybeUninit::zeroed()),
        }
    }

    fn endpoint(&self) -> u32 {
        self.endpoint.get() as u32
    }

    fn listener(&self) -> u32 {
        self.listener.get() as u32
    }

    /// Put `address` where the calls that take one read it, and give CPU2's
    /// pointer to it.
    ///
    /// SAFETY: not while CPU2 has a call to answer.
    unsafe fn name(&self, address: SocketAddrV6) -> u32 {
        let name = ffi::otSockAddr {
            mAddress: ip6_address(*address.ip()),
            mPort: address.port(),
        };
        let name_in = self.name.get().cast::<ffi::otSockAddr>();
        unsafe { write_volatile(name_in, name) };
        name_in as u32
    }
}

/// A callback to give CPU2 where it may insist on one. Nothing calls it: what
/// CPU2 calls back about arrives as a notification.
unsafe extern "C" fn never_called() {}

/// [`never_called`] as a callback of the type `F`.
///
/// SAFETY: `F` has to be an optional `extern "C"` function pointer.
unsafe fn placeholder<F>() -> F {
    const { assert!(size_of::<F>() == size_of::<*const ()>()) };
    unsafe { transmute_copy(&(never_called as *const ())) }
}

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
    /// A datagram has arrived on one of the UDP sockets.
    UdpReceived {
        /// Which socket: the context it was opened with.
        socket: usize,
        from: Ipv6Addr,
        payload: Datagram,
    },
    /// Something has come of the TCP endpoint.
    Tcp(TcpEvent),
    /// Anything else, by its ID, and a datagram that could not be taken.
    Other(u32),
}

/// What CPU2 calls back about the TCP endpoint.
#[derive(Clone, Copy)]
pub enum TcpEvent {
    /// A connection has come in to the listener. `taken` is what CPU2 was
    /// told: that it goes into the endpoint, or that it is refused.
    Incoming {
        taken: bool,
    },
    /// The connection is there to be used: the one that was opened, or the
    /// one that came in.
    Established,
    /// The peer has acknowledged what was being sent.
    SendDone,
    /// More has arrived, or the peer has said that it sends no more.
    ReceiveAvailable {
        end_of_stream: bool,
    },
    Disconnected(TcpDisconnected),
}

/// How a TCP connection has ended.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TcpDisconnected {
    /// Both sides are done, or the connection was dropped from here.
    Normal,
    Refused,
    Reset,
    TimedOut,
    /// Both sides are done, and the endpoint waits out the time in which
    /// something of the connection may still turn up. Until then, or until
    /// it is aborted, it takes no other connection.
    Lingering,
}

fn disconnected_from_raw(reason: u32) -> TcpDisconnected {
    use ffi::otTcpDisconnectedReason as Reason;
    match Reason(reason as _) {
        Reason::OT_TCP_DISCONNECTED_REASON_NORMAL => TcpDisconnected::Normal,
        Reason::OT_TCP_DISCONNECTED_REASON_REFUSED => TcpDisconnected::Refused,
        Reason::OT_TCP_DISCONNECTED_REASON_TIME_WAIT => TcpDisconnected::Lingering,
        Reason::OT_TCP_DISCONNECTED_REASON_TIMED_OUT => TcpDisconnected::TimedOut,
        _ => TcpDisconnected::Reset,
    }
}

/// Give CPU2 the return value of the callback it is in. ST's own handlers
/// put it where the callback's first argument arrived, before they
/// acknowledge.
///
/// SAFETY: only inside the handling of a notification, before it is
/// acknowledged: until then its buffer is CPU1's to write.
unsafe fn answer_callback(value: u32) {
    use embassy_stm32_wpan::consts::{TL_EVT_HEADER_SIZE, TL_PACKET_HEADER_SIZE};
    use embassy_stm32_wpan::tables::THREAD_NOTIF_RSP_EVT_BUFFER;

    // The event's payload is a Thread_OT_Cmd_Request_t: an ID, a size, and
    // the arguments.
    const FIRST_ARGUMENT: usize = TL_PACKET_HEADER_SIZE + TL_EVT_HEADER_SIZE + 8;
    unsafe {
        let buffer = (&raw mut THREAD_NOTIF_RSP_EVT_BUFFER).cast::<u8>();
        buffer
            .add(FIRST_ARGUMENT)
            .cast::<u32>()
            .write_unaligned(value);
    }
}

/// Wait for the stack's next callback. `sockets` are the buffers of the UDP
/// sockets, each at the index that is the context it was opened with. `tcp`
/// is that of the TCP endpoint, and `take_tcp` whether a connection that
/// comes in is to go into it: CPU2 wants to know inside the callback.
///
/// CPU2 is inside that callback until its notification is acknowledged, and
/// the message a datagram arrives in is only there that long. So the payload
/// is read out before the acknowledgement, which is why this takes `ot`, and
/// with calls that are blocked on, as nothing can be awaited at that point.
/// CPU2 answers them in well under a millisecond.
pub async fn notification(
    notif_rx: &mut ThreadNotifRx<'_>,
    ot: &mut ThreadOt<'_>,
    sockets: &'static [UdpBuffer],
    tcp: &'static TcpBuffer,
    take_tcp: bool,
) -> Notification {
    const STATE_CHANGE: u32 = ffi_notification::MSG_M0TOM4_NOTIFY_STATE_CHANGE as u32;
    const UDP_RECEIVE: u32 = ffi_notification::MSG_M0TOM4_UDP_RECEIVE as u32;
    const TCP_ACCEPT_READY: u32 = ffi_notification::MSG_M0TOM4_TCP_ACCEPT_READY_CALLBACK as u32;
    const TCP_ACCEPT_DONE: u32 = ffi_notification::MSG_M0TOM4_TCP_ACCEPT_DONE_CALLBACK as u32;
    const TCP_ESTABLISHED: u32 = ffi_notification::MSG_M0TOM4_TCP_ESTABLISHED_CALLBACK as u32;
    const TCP_SEND_DONE: u32 = ffi_notification::MSG_M0TOM4_TCP_SEND_DONE_CALLBACK as u32;
    const TCP_RECEIVE_AVAILABLE: u32 =
        ffi_notification::MSG_M0TOM4_TCP_RECEIVE_AVAILABLE_CALLBACK as u32;
    const TCP_DISCONNECTED: u32 = ffi_notification::MSG_M0TOM4_TCP_DISCONNECTED_CALLBACK as u32;

    let acknowledged_after = |raw: OtNotification| match raw.id {
        STATE_CHANGE => Notification::StateChanged,
        UDP_RECEIVE => {
            // The arguments of a socket's receive callback: its context, the
            // message, and the message's otMessageInfo.
            let [context, message, info, _] = raw.data;
            let socket = context as usize;
            let info = info as *const ffi::otMessageInfo;
            let Some(buffer) = sockets.get(socket) else {
                return Notification::Other(raw.id);
            };
            if raw.size < 3 || info.is_null() {
                return Notification::Other(raw.id);
            }

            let (from, payload) = unsafe {
                // SAFETY: CPU2 keeps the otMessageInfo, as it does the
                // message, until the notification is acknowledged, which is
                // not before this closure returns. It is in memory that CPU1
                // can read: ST's own receive callbacks read it where it is.
                // Its fields are numbers, the address a union of them.
                let from = read_volatile(info).mPeerAddr.mFields.m8;
                (from, block_on(udp_read(ot, buffer, message)))
            };
            match payload {
                Some(payload) => Notification::UdpReceived {
                    socket,
                    from: Ipv6Addr::from(from),
                    payload,
                },
                None => Notification::Other(raw.id),
            }
        }
        TCP_ACCEPT_READY => {
            use ffi::otTcpIncomingConnectionAction as Action;

            // The arguments of the listener's callback: the listener, the
            // peer's address, and where to put a pointer to the endpoint
            // that is to take the connection.
            let [_, _, endpoint_out, _] = raw.data;
            let taken = take_tcp && raw.size >= 3 && endpoint_out != 0;
            let action = if taken {
                Action::OT_TCP_INCOMING_CONNECTION_ACTION_ACCEPT
            } else {
                Action::OT_TCP_INCOMING_CONNECTION_ACTION_REFUSE
            };
            unsafe {
                // SAFETY: CPU2 waits for the acknowledgement with a place
                // for that pointer, which is in memory CPU1 can write: ST's
                // own handler hands it to the application to write to.
                if taken {
                    write_volatile(endpoint_out as *mut u32, tcp.endpoint());
                }
                // SAFETY: the notification is not acknowledged before this
                // closure returns.
                answer_callback(action.0 as u32);
            }
            Notification::Tcp(TcpEvent::Incoming { taken })
        }
        TCP_ACCEPT_DONE | TCP_ESTABLISHED => Notification::Tcp(TcpEvent::Established),
        TCP_SEND_DONE => Notification::Tcp(TcpEvent::SendDone),
        TCP_RECEIVE_AVAILABLE => {
            // The endpoint, how much has arrived, whether the peer is done
            // sending, and how much room is left.
            let [_, _, end_of_stream, _] = raw.data;
            Notification::Tcp(TcpEvent::ReceiveAvailable {
                end_of_stream: end_of_stream != 0,
            })
        }
        TCP_DISCONNECTED => {
            let [_, reason, _, _] = raw.data;
            Notification::Tcp(TcpEvent::Disconnected(disconnected_from_raw(reason)))
        }
        other => Notification::Other(other),
    };
    notif_rx.receive_with(acknowledged_after).await
}

/// Whether `pointer` is to somewhere in RAM that an otNetifAddress fits in.
fn is_in_ram(pointer: *const ffi::otNetifAddress) -> bool {
    /// SRAM1 and SRAM2, which follow each other.
    const RAM: core::ops::Range<usize> = 0x2000_0000..0x2004_0000;
    let (from, to) = (
        pointer as usize,
        pointer as usize + size_of::<ffi::otNetifAddress>(),
    );
    pointer.is_aligned() && RAM.contains(&from) && RAM.contains(&to)
}

fn address_from_raw(raw: &ffi::otNetifAddress) -> Address {
    // SAFETY: every bit pattern is an address.
    let address = Ipv6Addr::from(unsafe { raw.mAddress.mFields.m8 });
    // The interface identifier of a locator ends in what locates: the
    // device's RLOC16, or the like for a role it has, such as leader.
    let is_locator = address.segments()[4..7] == [0, 0xff, 0xfe00];
    let kind = if raw.mRloc() || (raw.mMeshLocal() && is_locator) {
        AddressKind::Locator
    } else if raw.mMeshLocal() {
        AddressKind::MeshLocal
    } else if address.is_unicast_link_local() {
        AddressKind::LinkLocal
    } else {
        AddressKind::Routable
    };
    Address { address, kind }
}

/// Drop the connection of the TCP endpoint, whatever state it is in. The
/// endpoint is free for another one afterwards. Answers whether the stack's
/// state changed meanwhile.
///
/// CPU2 calls back about the end of the connection from inside this call,
/// and waits for that to be acknowledged before it answers. So the
/// notifications are taken while the call is out, which is what ST's own
/// wrapper for this does. Nothing can be asked of CPU2 then: a datagram that
/// arrives just now is lost.
pub async fn tcp_abort(
    notif_rx: &mut ThreadNotifRx<'_>,
    ot: &mut ThreadOt<'_>,
    buffer: &'static TcpBuffer,
) -> (Result<()>, bool) {
    const STATE_CHANGE: u32 = ffi_notification::MSG_M0TOM4_NOTIFY_STATE_CHANGE as u32;

    let state_changed = Cell::new(false);
    let endpoint = [buffer.endpoint()];
    let abort = unsafe {
        // SAFETY: expects a pointer to the endpoint.
        ot.ffi_try(ffi_command::MSG_M4TOM0_OT_TCP_ABORT, &endpoint)
    };
    let acknowledge = async {
        loop {
            if notif_rx.receive().await.id == STATE_CHANGE {
                state_changed.set(true);
            }
        }
    };
    match select(abort, acknowledge).await {
        Either::First(aborted) => (aborted, state_changed.get()),
        Either::Second(never) => never,
    }
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

    /// Open a UDP socket. It can send from then on. What it receives comes
    /// with `context`, which tells it from the other sockets (see
    /// [`notification`]).
    async fn udp_open(&mut self, buffer: &'static UdpBuffer, context: usize) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to an otUdpSocket, which CPU2 fills
            // in and keeps until the socket is closed, and the context of
            // its receive callback, which is any word. The callback is no
            // argument: its calls arrive as notifications.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_UDP_OPEN,
                &[buffer.socket.get() as u32, context as u32],
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

    /// Set up the TCP endpoint and its listener. Once: CPU2 keeps both.
    async fn tcp_init(&mut self, buffer: &'static TcpBuffer) -> Result<()> {
        let endpoint_args = buffer
            .endpoint_args
            .get()
            .cast::<ffi::otTcpEndpointInitializeArgs>();
        let listener_args = buffer
            .listener_args
            .get()
            .cast::<ffi::otTcpListenerInitializeArgs>();
        unsafe {
            // SAFETY: this is the only place CPU1 touches either set of
            // arguments, never through a reference, and CPU2 only reads
            // them. Every callback is an optional function pointer.
            write_volatile(
                endpoint_args,
                ffi::otTcpEndpointInitializeArgs {
                    mContext: null_mut(),
                    mEstablishedCallback: placeholder(),
                    mSendDoneCallback: placeholder(),
                    // How much of what is being sent has got how far: more
                    // than there is a use for.
                    mForwardProgressCallback: None,
                    mReceiveAvailableCallback: placeholder(),
                    mDisconnectedCallback: placeholder(),
                    mReceiveBuffer: buffer.received.get().cast(),
                    mReceiveBufferSize: TCP_RECEIVE_LEN,
                },
            );
            write_volatile(
                listener_args,
                ffi::otTcpListenerInitializeArgs {
                    mContext: null_mut(),
                    mAcceptReadyCallback: placeholder(),
                    mAcceptDoneCallback: placeholder(),
                },
            );
            // SAFETY: each expects a pointer to memory for the endpoint, or
            // the listener, which CPU2 fills in and keeps, and a pointer to
            // its arguments, which it copies. The room for what arrives is
            // CPU2's from here on: CPU1 only reads it where CPU2 says.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_ENDPOINT_INITIALIZE,
                &[buffer.endpoint(), endpoint_args as u32],
            )
            .await?;
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_LISTENER_INITIALIZE,
                &[buffer.listener(), listener_args as u32],
            )
            .await
        }
    }

    /// Have the listener take connections to `port`, on any address of the
    /// device.
    async fn tcp_listen(&mut self, buffer: &'static TcpBuffer, port: u16) -> Result<()> {
        unsafe {
            // SAFETY: no call is waiting for its answer. Expects pointers to
            // the listener and to an otSockAddr, which it copies.
            let name = buffer.name(SocketAddrV6::new(Ipv6Addr::UNSPECIFIED, port, 0, 0));
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_LISTEN,
                &[buffer.listener(), name],
            )
            .await
        }
    }

    async fn tcp_stop_listening(&mut self, buffer: &'static TcpBuffer) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to the listener.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_STOP_LISTENING,
                &[buffer.listener()],
            )
            .await
        }
    }

    /// Start opening a connection from the endpoint to `peer`. A callback
    /// tells what comes of it.
    async fn tcp_connect(&mut self, buffer: &'static TcpBuffer, peer: SocketAddrV6) -> Result<()> {
        unsafe {
            // SAFETY: no call is waiting for its answer. Expects pointers to
            // the endpoint and to an otSockAddr, which it copies, and flags.
            let name = buffer.name(peer);
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_CONNECT,
                &[
                    buffer.endpoint(),
                    name,
                    ffi::OT_TCP_CONNECT_NO_FAST_OPEN.0 as u32,
                ],
            )
            .await
        }
    }

    /// Send `bytes` over the endpoint's connection. Nothing more may be sent
    /// until a callback has told that the peer has these, or that the
    /// connection is over: CPU2 reads them where they are until then.
    async fn tcp_send(&mut self, buffer: &'static TcpBuffer, bytes: &TcpChunk) -> Result<()> {
        let data = buffer.sending.get().cast::<u8>();
        let link = buffer.sending_link.get().cast::<ffi::otLinkedBuffer>();
        unsafe {
            // SAFETY: CPU1 touches what is being sent, and the link to it,
            // only here and never through a reference. That CPU2 is done
            // with the last of it is the caller's to see to.
            copy_nonoverlapping(bytes.as_ptr(), data, bytes.len());
            write_volatile(
                link,
                ffi::otLinkedBuffer {
                    mNext: null_mut(),
                    mData: data,
                    mLength: bytes.len(),
                },
            );
            // SAFETY: expects pointers to the endpoint and to an
            // otLinkedBuffer, which it keeps until the peer has what the
            // buffer links to, and flags.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_SEND_BY_REFERENCE,
                &[buffer.endpoint(), link as u32, 0],
            )
            .await
        }
    }

    /// Take what has arrived on the endpoint's connection, as far as a chunk
    /// has room. An empty one if nothing has.
    async fn tcp_receive(&mut self, buffer: &'static TcpBuffer) -> Result<TcpChunk> {
        let link_out = buffer.received_link.get();
        unsafe {
            // SAFETY: expects pointers to the endpoint and to where it puts
            // a pointer to the first of the links to what has arrived.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_RECEIVE_BY_REFERENCE,
                &[buffer.endpoint(), link_out as u32],
            )
            .await?;
        }

        // The links are the endpoint's own, and what they link to is in the
        // room it was given: anything else is not followed.
        let endpoint = buffer.endpoint.get() as usize;
        let links = endpoint..endpoint + size_of::<ffi::otTcpEndpoint>();
        let received = buffer.received.get() as usize;
        let room = received..=received + TCP_RECEIVE_LEN;

        let mut chunk = TcpChunk::new();
        // SAFETY: CPU2 wrote the pointer within the call, and has no call to
        // answer now.
        let mut link = unsafe { read_volatile(link_out) };
        while links.contains(&(link as usize)) && !chunk.is_full() {
            // SAFETY: `link` is inside the endpoint, which is in the buffer.
            // CPU2 writes the links when it is asked for them, which it is
            // not now.
            let ffi::otLinkedBuffer {
                mNext,
                mData,
                mLength,
            } = unsafe { read_volatile(link) };
            let (from, to) = (mData as usize, (mData as usize).saturating_add(mLength));
            if !room.contains(&from) || !room.contains(&to) {
                return Err(OtError::Failed);
            }
            let old = chunk.len();
            let taken = mLength.min(chunk.capacity() - old);
            // Cannot fail: no more than the chunk has room for.
            let _ = chunk.resize_default(old + taken);
            // SAFETY: `taken` bytes from `mData` are in the room for what
            // arrives, which CPU2 leaves as it is until they are committed,
            // and the chunk has that much room after `old`.
            unsafe { copy_nonoverlapping(mData, chunk.as_mut_ptr().add(old), taken) };
            link = mNext;
        }

        if !chunk.is_empty() {
            unsafe {
                // SAFETY: expects a pointer to the endpoint, how many bytes
                // at the front of what has arrived it may let go of, and
                // flags.
                self.ffi_try(
                    ffi_command::MSG_M4TOM0_OT_TCP_COMMIT_RECEIVE,
                    &[buffer.endpoint(), chunk.len() as u32, 0],
                )
                .await?;
            }
        }
        Ok(chunk)
    }

    /// Tell the peer that nothing more is sent on the endpoint's connection.
    async fn tcp_send_end_of_stream(&mut self, buffer: &'static TcpBuffer) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to the endpoint.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_SEND_END_OF_STREAM,
                &[buffer.endpoint()],
            )
            .await
        }
    }

    /// The device's unicast addresses, as far as [`Addresses`] has room.
    async fn ip6_unicast_addresses(&mut self) -> Addresses {
        let mut next = unsafe {
            // SAFETY: expects no arguments, and answers with a pointer to
            // the first of a linked list of otNetifAddress, or null.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_IP6_GET_UNICAST_ADDRESSES, &[])
                .await
        } as *const ffi::otNetifAddress;

        let mut addresses = Addresses::default();
        // The list is CPU2's, in its memory, and nothing keeps CPU2 from
        // changing it while it is read. So only what is in RAM is followed,
        // and no further than there is room for.
        while is_in_ram(next) {
            // SAFETY: `next` is the address of readable memory, aligned for
            // an otNetifAddress, whose fields are numbers and a pointer that
            // is not followed unchecked.
            let raw = unsafe { read_volatile(next) };
            if addresses.addresses.push(address_from_raw(&raw)).is_err() {
                addresses.truncated = true;
                break;
            }
            next = raw.mNext;
        }
        addresses
    }

    // TODO: the rest of the OpenThread API
}

unsafe impl OpenThread for ThreadOt<'_> {
    /// TODO: propagate (un)safety down into `ThreadOt`.
    async unsafe fn ffi_call(&mut self, cmd: ffi_command::Type, args: &[u32]) -> u32 {
        self.call(cmd as u32, args).await
    }
}
