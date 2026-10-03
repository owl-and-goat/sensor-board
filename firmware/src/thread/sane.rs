//! Sane Thread networking module. At some point this will take over from the
//! existing mod.rs.

use core::{
    net::Ipv6Addr,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_stm32_wpan::sub::thread::ThreadOt;

use crate::thread::ot;
use ot::OpenThread as _;

pub use ot::{Result, Role};

pub struct ThreadBuilder<'d> {}

/// High-level interface to the OpenThread stack on CPU2.
///
/// Example:
///
/// ```
/// # // let stack = [some ThreadOt stack]
/// # let thread = Thread::new(stack);
/// # thread.init()?;
/// ```
pub struct Thread<'d> {
    ot: ThreadOt<'d>,
    initialized: AtomicBool,
}

impl<'d> Thread<'d> {
    /// Create a new instance (well... the *only* instance) of the OpenThread
    /// stack.
    ///
    /// NOTE: You must call `init()` before using it.
    pub async fn new(ot: ThreadOt<'d>) -> Self {
        Self {
            ot,
            initialized: AtomicBool::new(false),
        }
    }

    /// Initialize the OpenThread stack.
    ///
    /// NOTE: Call this right after `new()` before you do anything else with the
    /// stack.
    pub async fn init(&mut self) -> Result<()> {
        if self.initialized.load(Ordering::Acquire) {
            return Ok(());
        }

        // TODO: clock initialization (that whole bullshit with semaphore 5)

        self.ot.instance_init_single().await?;
        self.ot.set_state_changed_callback().await?;

        self.up().await?;

        self.initialized.store(true, Ordering::Release);
        Ok(())
    }

    /// Check if the OpenThread stack is initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized.load(Ordering::Acquire)
    }

    /// Bring up the OpenThread v6 stack.
    pub async fn up(&mut self) -> Result<()> {
        self.ot.thread_set_enabled(true).await?;
        self.ot.ip6_set_enabled(true).await?;
        Ok(())
    }

    /// Shut down the OpenThread v6 stack.
    pub async fn down(&mut self) -> Result<()> {
        // does this really need to be in reverse order?
        self.ot.ip6_set_enabled(false).await?;
        self.ot.thread_set_enabled(false).await?;
        Ok(())
    }

    /// Get this device's current Thread role.
    pub async fn role(&mut self) -> Result<Role> {
        self.ot.thread_get_device_role().await
    }

    /* FIXME: I was about to think about how "ping" was going to work, then
     * stopped dead when I realized that there are several functions in the
     * OpenThread interface that are basically impossible to call safely because
     * they pass pointers to CPU1's memory for CPU2 to *overwrite*! I think this
     * means that we can't possibly be memory-safe if we're allowed to cancel a
     * Future (e.g., back the pointer with a static allocation, cancel, call a
     * second time, go to read the allocation, and have it changed out from
     * under us).
     */
}
