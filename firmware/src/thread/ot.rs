//! OpenThread API calls and error type. Translated (sort of) from the original
//! C API. See
//! [here](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v1.24.0/thread/openthread/core/openthread_api/).

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::ptr::{read_volatile, write_volatile};

use embassy_stm32_wpan::sub::thread::ThreadOt;
use protocol::{Dataset, ExtAddress, Neighbor, NeighborKind, OtError, Role};

// TODO: move ffi module to a submodule of this one?
use super::ffi;

// use ffi::MsgId_M0toM4_Enum_t as ffi_notification;
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
