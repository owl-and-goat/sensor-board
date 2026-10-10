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

/// Convert what `otThreadGetNextHopAndPathCost` returned for `destination`,
/// an RLOC16, to a [`Route`].
fn route_from_raw(destination: u16, next_hop: u16, cost: u8) -> Route {
    /// The stack's value for "no next hop".
    const INVALID_RLOC16: u16 = 0xfffe;
    /// The path cost that the stack reports for an unreachable destination.
    /// A child still reports its parent as the next hop in that case, as it
    /// does for every destination.
    const MAX_ROUTE_COST: u8 = 16;

    if next_hop == INVALID_RLOC16 || cost >= MAX_ROUTE_COST {
        Route::Unreachable
    } else if next_hop != destination {
        Route::Relayed {
            next_hop: RouterId::of_rloc16(next_hop),
            cost,
        }
    } else if cost == 0 {
        // No link has a cost of zero, so this is the stack's answer for the
        // device it runs on.
        Route::ThisBoard
    } else {
        Route::Direct { cost }
    }
}

/// CPU1 memory that CPU2 reads through a pointer: the dataset argument of
/// `otDatasetSetActiveTlvs`. It has to be `&'static`, because CPU2 keeps
/// reading it until it has answered the call, even if the future that made
/// the call has been dropped.
pub struct DatasetBuffer(UnsafeCell<MaybeUninit<ffi::otOperationalDatasetTlvs>>);

impl DatasetBuffer {
    pub const fn new() -> Self {
        DatasetBuffer(UnsafeCell::new(MaybeUninit::zeroed()))
    }
}

/// CPU1 memory that CPU2 writes through pointers: the two out arguments of
/// `otThreadGetNextNeighborInfo`. It has to be `&'static` for the same
/// reason as [`DatasetBuffer`], and here CPU2 writes until it has answered.
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

/// CPU1 memory that CPU2 writes through pointers: the two out arguments of
/// `otThreadGetNextHopAndPathCost`. `&'static` for the same reason as
/// [`NeighborBuffer`].
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

/// CPU1 memory for a UDP socket: the arguments that the `otUdp` and
/// `otMessage` calls pass by pointer. CPU2 links the socket itself into its
/// list of sockets and keeps using it from `otUdpOpen` until `otUdpClose`,
/// so this buffer in particular has to be `&'static`. CPU2 accesses the
/// other fields only during a call.
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

/// The longest payload a UDP socket sends or receives.
pub const MAX_DATAGRAM_LEN: usize = 256;

/// The payload of a UDP datagram.
pub type Datagram = heapless::Vec<u8, MAX_DATAGRAM_LEN>;

/// Size of the TCP receive buffer: the size OpenThread recommends for a
/// connection that crosses a few hops.
const TCP_RECEIVE_LEN: usize = ffi::OT_TCP_RECEIVE_BUFFER_SIZE_FEW_HOPS as usize;

/// The most bytes sent or received over TCP in one call.
pub const TCP_CHUNK_LEN: usize = 512;

/// A chunk of TCP data.
pub type TcpChunk = heapless::Vec<u8, TCP_CHUNK_LEN>;

/// CPU1 memory for TCP: one endpoint, which holds one connection at a time,
/// and the listener that accepts connections into it. CPU2 links both into
/// its own lists at initialization and keeps using them, and it keeps
/// writing to the receive buffer, so this has to be `&'static` for the same
/// reason as [`UdpBuffer`]. CPU2 reads the send buffer until the peer has
/// acknowledged the data. It accesses the other fields only during a call.
pub struct TcpBuffer {
    endpoint: UnsafeCell<MaybeUninit<ffi::otTcpEndpoint>>,
    endpoint_args: UnsafeCell<MaybeUninit<ffi::otTcpEndpointInitializeArgs>>,
    listener: UnsafeCell<MaybeUninit<ffi::otTcpListener>>,
    listener_args: UnsafeCell<MaybeUninit<ffi::otTcpListenerInitializeArgs>>,
    /// The address argument of `otTcpListen` and `otTcpConnect`.
    name: UnsafeCell<MaybeUninit<ffi::otSockAddr>>,
    received: UnsafeCell<[u8; TCP_RECEIVE_LEN]>,
    /// The out argument of `otTcpReceiveByReference`: a pointer to the
    /// endpoint's chain of `otLinkedBuffer`s that hold the received data.
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

    /// Write `address` to the `name` field, and return a pointer to it to
    /// pass to CPU2.
    ///
    /// SAFETY: no call to CPU2 may be in progress.
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

/// A placeholder for callback pointers that CPU2 may require to be non-null.
/// It is never called: CPU2 delivers its callbacks as notifications.
unsafe extern "C" fn never_called() {}

/// [`never_called`], cast to the callback type `F`.
///
/// SAFETY: `F` must be an `Option` of an `extern "C"` function pointer.
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

/// A callback from the stack, reduced to what the firmware uses.
pub enum Notification {
    /// Some state of the stack has changed, such as the device's role or one
    /// of its addresses.
    StateChanged,
    /// A datagram has arrived on one of the UDP sockets.
    UdpReceived {
        /// The socket that received it: the context it was opened with.
        socket: usize,
        from: Ipv6Addr,
        payload: Datagram,
    },
    /// A TCP callback.
    Tcp(TcpEvent),
    /// Any other notification, by its ID. Also a received datagram that
    /// could not be read.
    Other(u32),
}

/// A TCP callback from CPU2.
#[derive(Clone, Copy)]
pub enum TcpEvent {
    /// The listener has an incoming connection. `taken` is the answer CPU2
    /// was given: accept it into the endpoint, or refuse it.
    Incoming {
        taken: bool,
    },
    /// The connection is established. This covers both a connection opened
    /// from here and an accepted one.
    Established,
    /// The peer has acknowledged the data being sent.
    SendDone,
    /// More data has arrived, or the peer has closed its sending side.
    ReceiveAvailable {
        end_of_stream: bool,
    },
    Disconnected(TcpDisconnected),
}

/// Why a TCP connection ended.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TcpDisconnected {
    /// Both sides closed the connection, or it was aborted from here.
    Normal,
    Refused,
    Reset,
    TimedOut,
    /// Both sides closed the connection, and the endpoint is now in
    /// TIME-WAIT. It cannot accept or open another connection until that
    /// state expires or the endpoint is aborted.
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

/// Set the return value of the callback that CPU2 is waiting in. As ST's
/// handlers do, write it over the callback's first argument in the
/// notification buffer, before the notification is acknowledged.
///
/// SAFETY: call this only while handling a notification, before it is
/// acknowledged. Until then CPU1 may write to the notification buffer.
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
/// sockets, indexed by the context each was opened with. `tcp` is the buffer
/// of the TCP endpoint. `take_tcp` says whether to accept an incoming TCP
/// connection, which has to be decided inside the callback.
///
/// CPU2 stays inside the callback until its notification is acknowledged,
/// and the message that holds a received datagram is only valid until then.
/// So the payload is read before the acknowledgement, which is why this
/// takes `ot`. Those calls block, because nothing can be awaited at that
/// point. CPU2 answers them in well under a millisecond.
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
                // SAFETY: CPU2 keeps the otMessageInfo and the message valid
                // until the notification is acknowledged, which happens
                // after this closure returns. CPU1 can read that memory:
                // ST's receive callbacks read it in place. Every field is an
                // integer, and the address is a union of integers.
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

            // The accept-ready callback's arguments: the listener, the
            // peer's address, and an out pointer for the endpoint that
            // accepts the connection.
            let [_, _, endpoint_out, _] = raw.data;
            let taken = take_tcp && raw.size >= 3 && endpoint_out != 0;
            let action = if taken {
                Action::OT_TCP_INCOMING_CONNECTION_ACTION_ACCEPT
            } else {
                Action::OT_TCP_INCOMING_CONNECTION_ACTION_REFUSE
            };
            unsafe {
                // SAFETY: `endpoint_out` stays valid until the notification
                // is acknowledged, and CPU1 can write to it: ST's handler
                // passes it to the application for exactly that.
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
            // The arguments: the endpoint, the bytes available, whether the
            // peer has closed its sending side, and the bytes of buffer
            // remaining.
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

/// Whether an otNetifAddress at `pointer` would be aligned and entirely in
/// RAM.
fn is_in_ram(pointer: *const ffi::otNetifAddress) -> bool {
    /// SRAM1 and SRAM2, which are contiguous.
    const RAM: core::ops::Range<usize> = 0x2000_0000..0x2004_0000;
    let (from, to) = (
        pointer as usize,
        pointer as usize + size_of::<ffi::otNetifAddress>(),
    );
    pointer.is_aligned() && RAM.contains(&from) && RAM.contains(&to)
}

fn address_from_raw(raw: &ffi::otNetifAddress) -> Address {
    // SAFETY: every bit pattern is valid for this field of the union.
    let address = Ipv6Addr::from(unsafe { raw.mAddress.mFields.m8 });
    // A locator's interface identifier is 0000:00ff:fe00:xxxx, where xxxx is
    // the RLOC16, or an anycast locator such as the leader's.
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

/// Abort the TCP endpoint's connection, whatever state it is in. The
/// endpoint can then be used for another connection. Returns the result,
/// and whether a state-change notification arrived during the call.
///
/// CPU2 sends the disconnected callback from inside `otTcpAbort`, and does
/// not answer the call until that notification is acknowledged. So this
/// acknowledges notifications while the call is in progress, as ST's wrapper
/// does. No other call can be made during that time, so a UDP datagram that
/// arrives then cannot be read and is dropped.
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

/// Read the payload of the datagram in `message`, a `const otMessage *`.
/// `None` if it is longer than a [`Datagram`].
///
/// SAFETY: `message` must still be valid on CPU2: it has to be the argument
/// of a receive callback whose notification has not been acknowledged yet.
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
        // to and the maximum number of bytes to write. Returns the number
        // written. The payload buffer has room for `len` bytes. CPU1
        // accesses it only here and in `udp_send`, never through a
        // reference, and never while a call to CPU2 is in progress.
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

/// A position in the stack's neighbor table (`otNeighborInfoIterator`).
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
            // SAFETY: expects no arguments. The return value is the
            // `otInstance` pointer, not an error. It is not needed, because
            // every other call uses the instance implicitly.
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
            // SAFETY: CPU1 accesses the buffer only here, and never through
            // a reference. CPU2 only reads it. If an earlier call was
            // dropped while CPU2 was still reading, that call may see a mix
            // of the two datasets (each with a length that is in bounds).
            // The call below queues behind it and installs the requested
            // dataset.
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

    /// The entry of the neighbor table that follows `iterator`. Advances
    /// `iterator` past it. `None` at the end of the table.
    async fn thread_get_next_neighbor_info(
        &mut self,
        buffer: &'static NeighborBuffer,
        iterator: &mut NeighborIterator,
    ) -> Result<Option<Neighbor>> {
        let iterator_out = buffer.iterator.get();
        let info_out = buffer.info.get().cast::<ffi::otNeighborInfo>();

        let found = unsafe {
            // SAFETY: CPU1 accesses the buffer only in this function, and
            // never through a reference. If an earlier call was dropped
            // while CPU2 was still working on it, that call may overwrite
            // the iterator written here before CPU2 handles the call below,
            // which queues behind it. The worst result is that a table
            // entry is skipped or repeated.
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
            // SAFETY: CPU2 has answered, so it has finished writing. Both
            // are plain data, valid for any bit pattern, and the buffer
            // started out zeroed.
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

    /// The stack's route to the router with this ID.
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
            // SAFETY: CPU2 has answered, so it has finished writing. CPU1
            // accesses the buffer only in this function, and never through
            // a reference. CPU2 answers an earlier, dropped call before
            // this one, so the buffer now holds this call's results.
            (read_volatile(next_hop_out), read_volatile(path_cost_out))
        };
        route_from_raw(destination, next_hop, cost)
    }

    /// Open a UDP socket. It can send from then on. Datagrams it receives
    /// are reported with `context`, which identifies the socket (see
    /// [`notification`]).
    async fn udp_open(&mut self, buffer: &'static UdpBuffer, context: usize) -> Result<()> {
        unsafe {
            // SAFETY: expects a pointer to an otUdpSocket, which CPU2
            // initializes and keeps until the socket is closed, and the
            // context for its receive callback, which can be any word. The
            // callback itself is not an argument: its calls arrive as
            // notifications.
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

    /// Bind the open socket to `port`, on every address of the device, so
    /// that it receives the datagrams sent there.
    async fn udp_bind(&mut self, buffer: &'static UdpBuffer, port: u16) -> Result<()> {
        let name = ffi::otSockAddr {
            mAddress: ip6_address(Ipv6Addr::UNSPECIFIED),
            mPort: port,
        };
        let name_in = buffer.name.get().cast::<ffi::otSockAddr>();
        unsafe {
            // SAFETY: CPU1 accesses the name in the buffer only here, and
            // never through a reference. CPU2 only reads it.
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

    /// Send `payload` from the open socket to `port` at `address`. A
    /// datagram sent to a multicast address is also delivered to this
    /// device.
    async fn udp_send(
        &mut self,
        buffer: &'static UdpBuffer,
        address: Ipv6Addr,
        port: u16,
        payload: &Datagram,
    ) -> Result<()> {
        // SAFETY: every field of an otMessageInfo is an integer for which
        // zero is valid. Zero selects the default source address, port and
        // hop limit.
        let mut peer: ffi::otMessageInfo = unsafe { MaybeUninit::zeroed().assume_init() };
        peer.mPeerAddr = ip6_address(address);
        peer.mPeerPort = port;
        peer.set_mMulticastLoop(true);
        let peer_in = buffer.peer.get().cast::<ffi::otMessageInfo>();
        let payload_in = buffer.payload.get().cast::<u8>();

        let message = unsafe {
            // SAFETY: expects a pointer to an otMessageSettings, or null for
            // the defaults. Returns a pointer to a new otMessage, or null if
            // the stack has no buffer left.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_UDP_NEW_MESSAGE, &[0])
                .await
        };
        if message == 0 {
            return Err(OtError::NoBufs);
        }

        let sent = unsafe {
            // SAFETY: CPU1 accesses the payload and the peer in the buffer
            // only here and in `udp_read`, never through a reference, and
            // never while a call to CPU2 is in progress. A `Datagram` is no
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
            // The stack takes ownership of a message only when it accepts
            // it for sending. It did not accept this one, so free it.
            unsafe {
                // SAFETY: expects a pointer to an otMessage that the caller
                // still owns.
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

    /// Initialize the TCP endpoint and its listener. Call this once: CPU2
    /// keeps both from then on.
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
            // SAFETY: CPU1 accesses the two argument structs only here, and
            // never through a reference. CPU2 only reads them. Every
            // callback field is an optional function pointer.
            write_volatile(
                endpoint_args,
                ffi::otTcpEndpointInitializeArgs {
                    mContext: null_mut(),
                    mEstablishedCallback: placeholder(),
                    mSendDoneCallback: placeholder(),
                    // Not used: it reports send progress in more detail than
                    // the firmware needs.
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
            // SAFETY: each call expects a pointer to the endpoint or the
            // listener, which CPU2 initializes and keeps, and a pointer to
            // its arguments, which CPU2 copies. From here on CPU2 owns the
            // receive buffer, and CPU1 reads only the parts CPU2 points it
            // at.
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

    /// Listen for connections on `port`, on every address of the device.
    async fn tcp_listen(&mut self, buffer: &'static TcpBuffer, port: u16) -> Result<()> {
        unsafe {
            // SAFETY: no call to CPU2 is in progress. The call expects
            // pointers to the listener and to an otSockAddr, which CPU2
            // copies.
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

    /// Start connecting the endpoint to `peer`. A callback reports the
    /// outcome.
    async fn tcp_connect(&mut self, buffer: &'static TcpBuffer, peer: SocketAddrV6) -> Result<()> {
        unsafe {
            // SAFETY: no call to CPU2 is in progress. The call expects
            // pointers to the endpoint and to an otSockAddr, which CPU2
            // copies, and flags.
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

    /// Send `bytes` over the endpoint's connection. CPU2 reads them from the
    /// send buffer until the peer acknowledges them, so do not call this
    /// again before the send-done or disconnected callback.
    async fn tcp_send(&mut self, buffer: &'static TcpBuffer, bytes: &TcpChunk) -> Result<()> {
        let data = buffer.sending.get().cast::<u8>();
        let link = buffer.sending_link.get().cast::<ffi::otLinkedBuffer>();
        unsafe {
            // SAFETY: CPU1 accesses the send buffer and its link only here,
            // and never through a reference. The caller ensures that CPU2
            // has finished with the previous chunk.
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
            // otLinkedBuffer, and flags. CPU2 keeps the otLinkedBuffer until
            // the peer has acknowledged its data.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_SEND_BY_REFERENCE,
                &[buffer.endpoint(), link as u32, 0],
            )
            .await
        }
    }

    /// Read received data from the endpoint's connection, up to one chunk.
    /// Returns an empty chunk if there is none.
    async fn tcp_receive(&mut self, buffer: &'static TcpBuffer) -> Result<TcpChunk> {
        let link_out = buffer.received_link.get();
        unsafe {
            // SAFETY: expects a pointer to the endpoint, and an out pointer
            // for the first otLinkedBuffer of the received data.
            self.ffi_try(
                ffi_command::MSG_M4TOM0_OT_TCP_RECEIVE_BY_REFERENCE,
                &[buffer.endpoint(), link_out as u32],
            )
            .await?;
        }

        // Valid links are inside the endpoint struct, and their data is
        // inside the receive buffer. A link elsewhere ends the list, and
        // data elsewhere is an error.
        let endpoint = buffer.endpoint.get() as usize;
        let links = endpoint..endpoint + size_of::<ffi::otTcpEndpoint>();
        let received = buffer.received.get() as usize;
        let room = received..=received + TCP_RECEIVE_LEN;

        let mut chunk = TcpChunk::new();
        // SAFETY: CPU2 wrote the pointer during the call, and no call is in
        // progress now.
        let mut link = unsafe { read_volatile(link_out) };
        while links.contains(&(link as usize)) && !chunk.is_full() {
            // SAFETY: `link` points into the endpoint, which is in `buffer`.
            // CPU2 writes the links only during a receive-by-reference call,
            // and none is in progress.
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
            // Cannot fail: `old + taken` is at most the chunk's capacity.
            let _ = chunk.resize_default(old + taken);
            // SAFETY: `taken` bytes at `mData` are inside the receive
            // buffer, and CPU2 does not overwrite them before they are
            // committed. The chunk has `taken` bytes of room after `old`.
            unsafe { copy_nonoverlapping(mData, chunk.as_mut_ptr().add(old), taken) };
            link = mNext;
        }

        if !chunk.is_empty() {
            unsafe {
                // SAFETY: expects a pointer to the endpoint, the number of
                // bytes to release from the front of the received data, and
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

    /// Close the sending side of the endpoint's connection.
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

    /// The device's unicast addresses, as many as fit in [`Addresses`].
    async fn ip6_unicast_addresses(&mut self) -> Addresses {
        let mut next = unsafe {
            // SAFETY: expects no arguments. Returns a pointer to the head of
            // a linked list of otNetifAddress, or null.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_IP6_GET_UNICAST_ADDRESSES, &[])
                .await
        } as *const ffi::otNetifAddress;

        let mut addresses = Addresses::default();
        // The list is in CPU2's memory, and CPU2 may change it while it is
        // being read. So follow a pointer only if it is in RAM, and stop
        // when `addresses` is full.
        while is_in_ram(next) {
            // SAFETY: `next` points to readable memory and is aligned for an
            // otNetifAddress. Its fields are integers, plus a pointer that
            // is checked before it is followed.
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
