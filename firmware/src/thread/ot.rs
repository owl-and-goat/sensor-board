//! OpenThread API calls and error type. Translated (sort of) from the original
//! C API. See
//! [here](https://github.com/STMicroelectronics/stm32-mw-wpan/blob/v1.24.0/thread/openthread/core/openthread_api/).

use embassy_stm32_wpan::sub::thread::ThreadOt;

// TODO: move ffi module to a submodule of this one?
use super::ffi;

// use ffi::MsgId_M0toM4_Enum_t as ffi_notification;
use ffi::MsgId_M4toM0_Enum_t as ffi_command;

/// An `otError` from the stack. `Ok(())` is `OT_ERROR_NONE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Failed,
    Drop,
    NoBufs,
    NoRoute,
    Busy,
    Parse,
    InvalidArgs,
    Security,
    Abort,
    NotImplemented,
    InvalidState,
    NoAck,
    Detached,
    NotFound,
    Already,
    Other(u8),
}

pub type Result<T> = core::result::Result<T, Error>;

impl Error {
    fn check(raw: u32) -> Result<()> {
        let e = ffi::otError(raw as _);
        match e {
            ffi::otError::OT_ERROR_NONE => Ok(()),
            ffi::otError::OT_ERROR_FAILED => Err(Error::Failed),
            ffi::otError::OT_ERROR_DROP => Err(Error::Drop),
            ffi::otError::OT_ERROR_NO_BUFS => Err(Error::NoBufs),
            ffi::otError::OT_ERROR_NO_ROUTE => Err(Error::NoRoute),
            ffi::otError::OT_ERROR_BUSY => Err(Error::Busy),
            ffi::otError::OT_ERROR_PARSE => Err(Error::Parse),
            ffi::otError::OT_ERROR_INVALID_ARGS => Err(Error::InvalidArgs),
            ffi::otError::OT_ERROR_SECURITY => Err(Error::Security),
            ffi::otError::OT_ERROR_ABORT => Err(Error::Abort),
            ffi::otError::OT_ERROR_NOT_IMPLEMENTED => Err(Error::NotImplemented),
            ffi::otError::OT_ERROR_INVALID_STATE => Err(Error::InvalidState),
            ffi::otError::OT_ERROR_NO_ACK => Err(Error::NoAck),
            ffi::otError::OT_ERROR_DETACHED => Err(Error::Detached),
            ffi::otError::OT_ERROR_NOT_FOUND => Err(Error::NotFound),
            ffi::otError::OT_ERROR_ALREADY => Err(Error::Already),
            other => Err(Error::Other(other.0)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Disabled,
    Detached,
    Child,
    Router,
    Leader,
    Other(u8),
}

impl Role {
    fn from_raw(raw: u32) -> Role {
        match ffi::otDeviceRole(raw as _) {
            ffi::otDeviceRole::OT_DEVICE_ROLE_DISABLED => Role::Disabled,
            ffi::otDeviceRole::OT_DEVICE_ROLE_DETACHED => Role::Detached,
            ffi::otDeviceRole::OT_DEVICE_ROLE_CHILD => Role::Child,
            ffi::otDeviceRole::OT_DEVICE_ROLE_ROUTER => Role::Router,
            ffi::otDeviceRole::OT_DEVICE_ROLE_LEADER => Role::Leader,
            other => Role::Other(other.0),
        }
    }

    /// True once the device is part of a network.
    pub fn is_attached(self) -> bool {
        matches!(self, Role::Child | Role::Router | Role::Leader)
    }
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
        Error::check(unsafe { self.ffi_call(cmd, args).await })
    }

    /// Initialize the Thread stack.
    async fn instance_init_single(&mut self) -> Result<()> {
        unsafe {
            // SAFETY: expects no arguments.
            self.ffi_try(ffi_command::MSG_M4TOM0_OT_INSTANCE_INIT_SINGLE, &[])
                .await
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

    async fn thread_get_device_role(&mut self) -> Result<Role> {
        let role = unsafe {
            // SAFETY: expects no arguments.
            self.ffi_call(ffi_command::MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE, &[])
                .await
        };

        Ok(Role::from_raw(role))
    }

    // TODO: the rest of the OpenThread API
}

unsafe impl OpenThread for ThreadOt<'_> {
    /// TODO: propagate (un)safety down into `ThreadOt`.
    async unsafe fn ffi_call(&mut self, cmd: ffi_command::Type, args: &[u32]) -> u32 {
        self.call(cmd as u32, args).await
    }
}
