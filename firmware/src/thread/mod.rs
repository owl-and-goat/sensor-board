//! Sane Thread networking module. At some point this will take over from the
//! existing mod.rs.

use embassy_stm32::{Peri, peripherals::IPCC};
use embassy_stm32_wpan::sub::{
    mm::MemoryManager,
    thread::{ThreadCliRx, ThreadNotifRx, ThreadOt},
    traces::Traces,
};

mod ffi;
mod inner;
mod ot;

pub use ot::{Result, Role};

/// A Thread operational dataset in its TLV form, which is what joins a device
/// to an existing network.
#[derive(Clone, Copy)]
pub struct Dataset(ffi::otOperationalDatasetTlvs);

impl Dataset {
    pub fn from_bytes(tlvs: &[u8]) -> Dataset {
        let mut d = ffi::otOperationalDatasetTlvs {
            mTlvs: [0; 254],
            mLength: 0,
        };
        let n = tlvs.len().min(d.mTlvs.len());
        d.mTlvs[..n].copy_from_slice(&tlvs[..n]);
        d.mLength = n as u8;
        Dataset(d)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.0.mTlvs[..(self.0.mLength as usize).min(self.0.mTlvs.len())]
    }

    /// The Mesh-Local Prefix TLV (type 7), which every address in the mesh
    /// shares and which the leader anycast address is built from.
    pub fn mesh_local_prefix(&self) -> Option<[u8; 8]> {
        let t = self.as_bytes();
        let mut i = 0;
        while i + 2 <= t.len() {
            let (ty, len) = (t[i], t[i + 1] as usize);
            if ty == 7 && len == 8 && i + 2 + 8 <= t.len() {
                let mut p = [0u8; 8];
                p.copy_from_slice(&t[i + 2..i + 10]);
                return Some(p);
            }
            i += 2 + len;
        }
        None
    }
}

use crate::thread::ot::OpenThread as _;

mod cpu2 {
    use super::*;
    use embassy_futures::select::{Either, select};
    use embassy_stm32::{
        bind_interrupts,
        ipcc::{self, ReceiveInterruptHandler, TransmitInterruptHandler},
    };
    use embassy_stm32_wpan::{
        TlMbox,
        shci::{SchiCommandStatus, SchiSysEventReady},
        sub::sys::Sys,
    };
    use embassy_time::Timer;

    bind_interrupts!(struct Irqs {
        IPCC_C1_RX => ReceiveInterruptHandler;
        IPCC_C1_TX => TransmitInterruptHandler;
    });

    /// CPU2's ready event, absent when it never sent one.
    pub(super) type Ready = Option<core::result::Result<SchiSysEventReady, ()>>;

    pub(super) fn ready_code(ready: Ready) -> u8 {
        match ready {
            None => 0xFF,
            Some(Err(())) => 0xFE,
            Some(Ok(r)) => r as u8,
        }
    }
    /// Hardware semaphore 5 guards the 48 MHz clock (CLK48/HSI48). When a wireless stack starts,
    /// CPU2 takes it and switches that clock off unless CPU1 already holds it. USB needs the clock,
    /// so take it before C2BOOT and never release it (AN5289; same as ST's USB examples on WB).
    const HSEM_CLK48: usize = 5;

    pub(super) fn lock_clk48_semaphore() -> bool {
        use embassy_stm32::pac::{HSEM, RCC};
        RCC.ahb3enr().modify(|w| w.set_hsemen(true));
        // One-step lock: reading RLR locks the semaphore for this core if free.
        let r = HSEM.rlr(HSEM_CLK48).read();
        r.lock()
    }

    pub(super) fn clk48_semaphore_held() -> bool {
        let r = embassy_stm32::pac::HSEM.r(HSEM_CLK48).read();
        r.lock()
    }

    /// Release semaphore 5 (CPU1 core id is 4 in the HSEM registers).
    pub(super) fn release_clk48_semaphore() {
        embassy_stm32::pac::HSEM.r(HSEM_CLK48).write(|w| {
            w.set_lock(false);
            w.set_coreid(4);
            w.set_procid(0);
        });
    }

    /// FUS writes FUS_DEVICE_INFO_TABLE_VALIDITY_KEYWORD as the first word when
    /// it is the one running; the table then has the MB_FUS_DeviceInfoTable_t
    /// layout instead of the wireless-firmware one.
    pub const FUS_TABLE_KEYWORD: u32 = 0xA946_56B9;

    /// True when CPU2 reports a running Thread stack (not FUS, not another stack).
    pub fn thread_stack_running(sys: &Sys<'_>) -> bool {
        sys.device_info_raw()[0] != FUS_TABLE_KEYWORD
            && sys
                .wireless_fw_info()
                .map(|i| i.thread_info & 0xff == 0x10)
                .unwrap_or(false)
    }

    /// "Fake a C2BOOT when it has already been set" (ST's words): SHCI_C2_REINIT
    /// followed by a SEV instruction makes CPU2 restart its firmware, re-read the
    /// reference table and send its ready event again.
    async fn reinit(sys: &mut Sys<'_>) -> (core::result::Result<SchiCommandStatus, ()>, Ready) {
        let status = sys.shci_c2_reinit().await;
        cortex_m::asm::sev();
        let ready = match select(sys.read_ready(), Timer::after_secs(3)).await {
            Either::First(r) => Some(r),
            Either::Second(()) => None,
        };
        (status, ready)
    }

    pub async fn ensure_booted<'d>(ipcc: Peri<'d, IPCC>) -> TlMbox<'d> {
        lock_clk48_semaphore();
        let mut mbox = TlMbox::init_without_ready(ipcc, Irqs, ipcc::Config::default());

        // CPU2 sends its ready event once after it boots. A CPU1-only reset (our
        // DFU round trips) leaves CPU2 running, so don't wait forever.
        let ok = match select(mbox.sys_subsystem.read_ready(), Timer::after_secs(2)).await {
            Either::First(_) => true,
            // No ready event means CPU2 was already up: the ROM bootloader boots it
            // for its own FUS commands, and a CPU1-only reset does not restart it.
            Either::Second(()) => false,
        };

        if !ok {
            reinit(&mut mbox.sys_subsystem).await;
        }

        mbox
    }
}

pub struct Builder<'d> {
    pub ipcc: Peri<'d, IPCC>,
}

impl<'d> Builder<'d> {
    pub async fn init(self) -> Result<(Task<'d>, Handle<'d>)> {
        let mbox = cpu2::ensure_booted(self.ipcc).await;
        let (ot, cli_rx, notif_rx) = mbox.thread_subsystem.split();
        let task = Task {
            mm_subsystem: mbox.mm_subsystem,
        };
        let mut handle = Handle {
            traces_subsystem: mbox.traces_subsystem,
            ot,
            cli_rx,
            notif_rx,
        };

        handle.ot.instance_init_single().await?;
        handle.ot.set_state_changed_callback().await?;

        handle.up().await?;

        Ok((task, handle))
    }
}

pub struct Task<'d> {
    mm_subsystem: MemoryManager<'d>,
}

impl<'d> Task<'d> {
    pub async fn run(mut self) -> ! {
        self.mm_subsystem.run_queue().await
    }
}

/// Handle to the OpenThread stack on CPU2.
///
/// Example:
///
/// ```
/// # // let stack = [some ThreadOt stack]
/// # let thread = Thread::new(stack);
/// # thread.init()?;
/// ```
pub struct Handle<'d> {
    traces_subsystem: Traces<'d>,
    ot: ThreadOt<'d>,
    cli_rx: ThreadCliRx<'d>,
    notif_rx: ThreadNotifRx<'d>,
}

impl<'d> Handle<'d> {
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
