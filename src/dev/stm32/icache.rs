//! The STM32U5 instruction cache (ICACHE) and data cache (DCACHE) controllers.
//!
//! `st.icache` is the register file firmware sees, not a cache. Nothing here
//! holds a line: every fetch and every load already goes straight to the
//! memory behind the cache, which is what a coherent cache looks like from
//! the core. What a firmware *can* observe is the control block — and ST's
//! U5 HAL waits on it:
//!
//! * `HAL_ICACHE_Invalidate` sets `CR.CACHEINV` and spins until
//!   `SR.BSYENDF` comes up;
//! * `HAL_ICACHE_Disable` clears `CR.EN`, which starts an invalidation by
//!   itself, and spins until `EN` reads back clear;
//! * `HAL_DCACHE_Invalidate` spins on `SR.BUSYF` going clear, and the
//!   by-address clean/invalidate commands set `CR.STARTCMD` and spin on
//!   `SR.CMDENDF`.
//!
//! A RAM window answers none of those, so this block exists.
//!
//! # What it does
//!
//! | | |
//! | --- | --- |
//! | invalidation | `CR.CACHEINV` (and, on the ICACHE, `EN` falling) sets `SR.BUSYF`; `busy-delay` ticks later `BUSYF` clears, `CACHEINV` clears and `BSYENDF` sets |
//! | DCACHE commands | `CR.STARTCMD` with a non-zero `CACHECMD` sets `SR.BUSYCMDF`; `busy-delay` ticks later it clears and `CMDENDF` sets. `STARTCMD` reads zero |
//! | interrupts | the `irq` output is `SR & IER` — `BSYENDIE`/`ERRIE` (and the DCACHE's `CMDENDIE`) |
//! | flags | `FCR` is write-one-to-clear and reads zero |
//! | monitors | the hit/miss counters read zero — there is nothing to count — and their reset bits are accepted and read zero |
//! | configuration | `WAYSEL` and the ICACHE's `CRR0`…`CRR3` remap regions are storage, writable only while `EN` is clear, as RM0456 has it |
//!
//! `busy-delay` is a count of ticks of this device's own clock domain: the
//! device is lazily advanced exactly as [`rcc`](super::rcc) is, so the guest's
//! poll is what catches it up and nothing reads the host clock. With
//! `busy-delay = 0` an operation completes inside the write that started it
//! and `BUSYF`/`BUSYCMDF` are never seen set, which every HAL loop also
//! accepts.
//!
//! # Variants
//!
//! | `variant` | Block | NS base on a U575/U585 | NVIC |
//! | --- | --- | --- | --- |
//! | `icache` (default) | `ICACHE_TypeDef`: `CR`, `SR`, `IER`, `FCR`, `HMONR`, `MMONR`, `CRR0`…`CRR3` | `0x4003_0400` | 107 (`ICACHE_IRQn`) |
//! | `dcache` | `DCACHE_TypeDef`: `CR`, `SR`, `IER`, `FCR`, `RHMONR`, `RMMONR`, `WHMONR`, `WMMONR`, `CMDRSADDRR`, `CMDREADDRR` | `0x4003_1400` | 111 (`DCACHE1_IRQn`) |
//!
//! Both decode `0x30` bytes. The ICACHE has no RCC clock-enable bit (only a
//! sleep-mode one), so its `enable` pin is normally left undrawn, which leaves
//! it clocked; the DCACHE's is `RCC_AHB1ENR.DCACHE1EN`, bit 30 —
//! `wire rcc.ahb1en30 -> dcache1.enable`.
//!
//! # Sources
//!
//! * ST **RM0456** (STM32U5 reference manual), the ICACHE and DCACHE
//!   chapters, for the behaviour.
//! * ST's CMSIS device header `stm32u585xx.h` (`ICACHE_TypeDef`,
//!   `DCACHE_TypeDef`, `ICACHE_*_Pos`, `DCACHE_*_Pos`; Apache-2.0) for the
//!   offsets and bits.
//! * ST's U5 HAL (`stm32u5xx_hal_icache.c`, `stm32u5xx_hal_dcache.c`,
//!   BSD-3-Clause) for which flags firmware waits on, and for the reset value
//!   `HAL_ICACHE_DeInit` writes back to `CRRx` (`RSIZE` = 2 MB). None of its
//!   code is reproduced.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # Known deviations
//!
//! * No caching, so `HMONR`/`MMONR` and the DCACHE's four monitors are always
//!   zero and the remap regions remap nothing — a firmware that relies on a
//!   `CRRx` remap to *see* external memory at a cacheable alias needs the
//!   board to map that alias.
//! * `ERRF` is never raised: there is no AHB error a modelled fetch can take.
//! * The power-on invalidation the hardware runs out of reset is not shown —
//!   `SR` resets to zero rather than to a busy cache — because no firmware
//!   waits for it and a reset value transcribed from memory would be a guess.
//! * Clearing the DCACHE's `EN` starts nothing: whether it launches an
//!   invalidation as the ICACHE's does was not to hand to confirm, and the
//!   HAL's wait for `BUSYF | BUSYCMDF` ends either way.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.icache";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes either block decodes: through `CRR3` / `CMDREADDRR` at
/// `+0x2c`.
pub const REGISTER_BYTES: u64 = 0x30;

/// How many 32-bit words that is.
const WORDS: usize = (REGISTER_BYTES / 4) as usize;

/// "No deadline": nothing is in progress.
const NO_DEADLINE: u64 = u64::MAX;

/// The default length of an invalidation or a DCACHE command, in ticks.
///
/// Short on purpose, as the RCC's ready delay is: firmware polls the flag and
/// cannot tell the difference except in virtual time spent.
const DEFAULT_BUSY_DELAY: u64 = 16;

/// The name of the interrupt output.
pub const IRQ_PIN: &str = "irq";

// -- offsets (ICACHE_TypeDef / DCACHE_TypeDef) -------------------------------

const CR: u64 = 0x00;
const SR: u64 = 0x04;
const IER: u64 = 0x08;
const FCR: u64 = 0x0c;
/// ICACHE `CRR0`; `CRR1`…`CRR3` follow.
const CRR0: u64 = 0x20;
/// DCACHE `CMDRSADDRR`.
const CMDRSADDRR: u64 = 0x28;
/// DCACHE `CMDREADDRR`.
const CMDREADDRR: u64 = 0x2c;

// -- bits ----------------------------------------------------------------------

/// `CR.EN`, both blocks.
const CR_EN: u32 = 1 << 0;
/// `CR.CACHEINV`, both blocks.
const CR_CACHEINV: u32 = 1 << 1;
/// ICACHE `CR.WAYSEL` — 1 (two-way) out of reset.
const IC_CR_WAYSEL: u32 = 1 << 2;
/// ICACHE `CR.HITMEN | MISSMEN`.
const IC_CR_MONEN: u32 = 0b11 << 16;
/// DCACHE `CR.CACHECMD[10:8]`.
const DC_CR_CACHECMD: u32 = 0b111 << 8;
/// DCACHE `CR.STARTCMD`.
const DC_CR_STARTCMD: u32 = 1 << 11;
/// DCACHE `CR.RHITMEN | RMISSMEN | WHITMEN | WMISSMEN`.
const DC_CR_MONEN: u32 = (0b11 << 16) | (0b11 << 20);
/// DCACHE `CR.HBURST`.
const DC_CR_HBURST: u32 = 1 << 31;

/// `SR.BUSYF`.
const SR_BUSYF: u32 = 1 << 0;
/// `SR.BSYENDF`.
const SR_BSYENDF: u32 = 1 << 1;
/// `SR.ERRF`.
const SR_ERRF: u32 = 1 << 2;
/// DCACHE `SR.BUSYCMDF`.
const SR_BUSYCMDF: u32 = 1 << 3;
/// DCACHE `SR.CMDENDF`.
const SR_CMDENDF: u32 = 1 << 4;

/// `CRRx`: `BASEADDR[7:0]`, `RSIZE[11:9]`, `REN[15]`, `REMAPADDR[26:16]`,
/// `MSTSEL[28]`, `HBURST[31]`.
const CRR_MASK: u32 = 0x97ff_8eff;
/// `CRRx` out of reset: `RSIZE` = 1, a 2 MB region — what `HAL_ICACHE_DeInit`
/// writes back.
const CRR_RESET: u32 = 0x0000_0200;
/// `CMDRSADDRR`/`CMDREADDRR`: 16-byte line addresses.
const CMDADDR_MASK: u32 = 0xffff_fff0;

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which of the two cache controllers this instance is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The instruction cache, `ICACHE_TypeDef`.
    Icache,
    /// A data cache, `DCACHE_TypeDef`.
    Dcache,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::Icache => "icache",
            Variant::Dcache => "dcache",
        }
    }

    /// Which `SR` bits exist, and so which `IER`/`FCR` bits mean anything.
    fn flags(self) -> u32 {
        match self {
            Variant::Icache => SR_BSYENDF | SR_ERRF,
            Variant::Dcache => SR_BSYENDF | SR_ERRF | SR_CMDENDF,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// When the running invalidation finishes.
    inval_at: u64,
    /// When the running DCACHE command finishes.
    cmd_at: u64,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(variant: Variant) -> State {
        let mut words = [0u32; WORDS];
        if variant == Variant::Icache {
            words[(CR / 4) as usize] = IC_CR_WAYSEL;
            for i in 0..4 {
                words[(CRR0 / 4) as usize + i] = CRR_RESET;
            }
        }
        State {
            words,
            inval_at: NO_DEADLINE,
            cmd_at: NO_DEADLINE,
            tick: 0,
        }
    }

    #[inline]
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    #[inline]
    fn word_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }

    /// Finish the invalidation: `BUSYF` and `CACHEINV` drop, `BSYENDF` rises.
    fn end_invalidate(&mut self) {
        self.inval_at = NO_DEADLINE;
        *self.word_mut(SR) = (self.word(SR) & !SR_BUSYF) | SR_BSYENDF;
        *self.word_mut(CR) &= !CR_CACHEINV;
    }

    /// Finish the DCACHE command: `BUSYCMDF` drops, `CMDENDF` rises.
    fn end_command(&mut self) {
        self.cmd_at = NO_DEADLINE;
        *self.word_mut(SR) = (self.word(SR) & !SR_BUSYCMDF) | SR_CMDENDF;
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    busy_delay: u64,
    /// The `irq` output, once a board connects it.
    irq: Mutex<Option<WireSource>>,
    /// The lock-free half of the lazy contract.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
    /// The RCC enable/reset inputs. Wiring, not chip state.
    gate: ClockGate,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("busy-delay", &self.busy_delay);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

impl Gated for Registers {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }

    fn gate_reset(&self) {
        {
            let mut state = self.state.lock();
            let tick = state.tick;
            *state = State::reset(self.variant);
            state.tick = tick;
            self.republish(&state);
        }
        self.refresh_irq();
    }
}

impl Registers {
    /// Catch the device up before answering an access. No lock held.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else { return };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }

    /// Republish what the lock-free lazy surface reads.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let next = state.inval_at.min(state.cmd_at);
        let next = if next <= state.tick { u64::MAX } else { next };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Whether a flag and its enable are both standing.
    fn pending(&self, state: &State) -> bool {
        state.word(SR) & state.word(IER) & self.variant.flags() != 0
    }

    /// Drive `irq` to what `SR & IER` says. Called with **no lock held**.
    fn refresh_irq(&self) {
        let level = {
            let state = self.state.lock();
            Level::from_bool(self.pending(&state))
        };
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Simulate forward: an invalidation or a command finishes.
    fn advance_to(&self, tick: u64) {
        {
            let mut state = self.state.lock();
            if tick <= state.tick {
                return;
            }
            if !self.gate.live() {
                // No clock, nothing progresses: slide the deadlines so the
                // operation resumes with the ticks it had left.
                let elapsed = tick - state.tick;
                let state = &mut *state;
                state.tick = tick;
                for at in [&mut state.inval_at, &mut state.cmd_at] {
                    if *at != NO_DEADLINE {
                        *at = at.saturating_add(elapsed);
                    }
                }
                self.republish(state);
                return;
            }
            state.tick = tick;
            if state.inval_at <= tick {
                state.end_invalidate();
            }
            if state.cmd_at <= tick {
                state.end_command();
            }
            self.republish(&state);
        }
        self.refresh_irq();
    }

    /// Start an invalidation, unless one is already running.
    fn start_invalidate(&self, state: &mut State) {
        if state.word(SR) & SR_BUSYF != 0 {
            return;
        }
        *state.word_mut(SR) |= SR_BUSYF;
        *state.word_mut(CR) |= CR_CACHEINV;
        if self.busy_delay == 0 {
            state.end_invalidate();
        } else {
            state.inval_at = state.tick.saturating_add(self.busy_delay);
        }
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            // The clear register holds nothing; the monitors count nothing.
            FCR | 0x10 | 0x14 | 0x18 | 0x1c => 0,
            0x20 | 0x24 if self.variant == Variant::Dcache => 0,
            _ => state.word(offset),
        }
    }

    /// Write one register, then whatever it implies.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            match offset {
                CR => self.write_cr(&mut state, value),
                SR => {
                    // Read-only.
                }
                IER => *state.word_mut(IER) = value & self.variant.flags(),
                FCR => *state.word_mut(SR) &= !(value & self.variant.flags()),
                CRR0..=0x2c if self.variant == Variant::Icache => {
                    // "Can be written only when the cache is disabled."
                    if state.word(CR) & CR_EN == 0 {
                        *state.word_mut(offset) = value & CRR_MASK;
                    }
                }
                CMDRSADDRR | CMDREADDRR => *state.word_mut(offset) = value & CMDADDR_MASK,
                _ => {
                    // The monitors and the reserved words ignore a write.
                }
            }
            self.republish(&state);
        }
        self.refresh_irq();
    }

    fn write_cr(&self, state: &mut State, value: u32) {
        let before = state.word(CR);
        let enabled = before & CR_EN != 0;
        let next = match self.variant {
            Variant::Icache => {
                // `WAYSEL` is frozen while the cache is enabled.
                let waysel = if enabled {
                    before & IC_CR_WAYSEL
                } else {
                    value & IC_CR_WAYSEL
                };
                (value & (CR_EN | IC_CR_MONEN)) | waysel
            }
            Variant::Dcache => value & (CR_EN | DC_CR_CACHECMD | DC_CR_MONEN | DC_CR_HBURST),
        };
        // `CACHEINV` is hardware's to clear, so it stays up while busy.
        *state.word_mut(CR) = next | (before & CR_CACHEINV);
        let disabled_now = self.variant == Variant::Icache && enabled && next & CR_EN == 0;
        if value & CR_CACHEINV != 0 || disabled_now {
            // RM0456: clearing the ICACHE's `EN` "automatically starts a
            // cache invalidation", as `HAL_ICACHE_Disable` notes too.
            self.start_invalidate(state);
        }
        if self.variant == Variant::Dcache
            && value & DC_CR_STARTCMD != 0
            && next & DC_CR_CACHECMD != 0
            && state.word(SR) & SR_BUSYCMDF == 0
        {
            *state.word_mut(SR) |= SR_BUSYCMDF;
            if self.busy_delay == 0 {
                state.end_command();
            } else {
                state.cmd_at = state.tick.saturating_add(self.busy_delay);
            }
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        if !self.gate.live() {
            // A gated block answers zero ([`gate`]).
            (*a, *b, *c, *d) = (0, 0, 0, 0);
            return Ok(());
        }
        // A debug sync advances nothing, and no register read has a side
        // effect, so a debugger read changes nothing.
        self.sync(attrs);
        let value = self.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would start an invalidation or a command
            // and to `FCR` would clear a flag; refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            return Ok(());
        }
        self.sync(attrs);
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32U5 ICACHE or DCACHE control block.
#[derive(Debug)]
pub struct Icache {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Icache {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Icache> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "icache", &["icache", "dcache"])? {
            "dcache" => Variant::Dcache,
            _ => Variant::Icache,
        };
        let busy_delay = r.or("busy-delay", DEFAULT_BUSY_DELAY)?;
        r.finish()?;
        Ok(Icache::with_config(variant, busy_delay))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_config(variant: Variant, busy_delay: u64) -> Icache {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(variant)),
            variant,
            busy_delay,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            gate: ClockGate::new(),
        });
        let region = Arc::new(Region::io(
            variant.as_str(),
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Icache { regs, region }
    }

    /// Which block this instance is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }
}

impl Device for Icache {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            *state = State::reset(self.regs.variant);
            state.tick = tick;
            self.regs.republish(&state);
        }
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        w.write_u64(state.inval_at)?;
        w.write_u64(state.cmd_at)?;
        w.write_u64(state.tick)
        // The `irq` wire and the gate are wiring, not chip state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(self.regs.variant);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        state.inval_at = r.read_u64()?;
        state.cmd_at = r.read_u64()?;
        state.tick = r.read_u64()?;
        {
            let mut held = self.regs.state.lock();
            *held = state;
            self.regs.republish(&held);
        }
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: format!("a cache controller drives one pin, `{IRQ_PIN}`"),
            });
        }
        *self.regs.irq.lock() = Some(source);
        self.regs.refresh_irq();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            self.regs.refresh_irq();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        gate::sink(&self.regs, port, sources)
    }

    fn is_lazy(&self) -> bool {
        // An invalidation takes time and the guest's poll of `SR` is what
        // has to see it end.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }
}

impl Instance for Icache {}

/// The `st.icache` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32U5 ICACHE/DCACHE control block: invalidation and clean commands with their \
              busy and end flags, the interrupt, and no actual caching",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which block: \"icache\" (default) or \"dcache\"",
        },
        PropertySpec {
            name: "busy-delay",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many ticks of this device's clock domain an invalidation or a DCACHE \
                      command takes; 0 finishes it inside the write (default 16)",
        },
    ],
    construct: |props| Ok(Box::new(Icache::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Icache::new(props)?)))
}

/// What the validator should know about `st.icache`.
#[must_use]
pub fn schema() -> ClassSchema {
    let schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["icache", "dcache"]))
        .prop(PropSchema::new("busy-delay", ValueKind::Uint))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out);
    gate::ports(schema)
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicU32;
    use crate::core::wire::{Wire, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    const DELAY: u64 = 4;

    fn cache(variant: Variant) -> Icache {
        Icache::with_config(variant, DELAY)
    }

    fn peek(dev: &Icache, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        dev.regs
            .read(offset, &mut buf, MemAttrs::DEFAULT)
            .expect("a word read");
        u32::from_le_bytes(buf)
    }

    fn peek_debug(dev: &Icache, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        dev.regs
            .read(offset, &mut buf, MemAttrs::DEBUG)
            .expect("a debug word read");
        u32::from_le_bytes(buf)
    }

    fn poke(dev: &Icache, offset: u64, value: u32) {
        dev.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write");
    }

    fn tick(dev: &Icache, ticks: u64) {
        let now = Device::current_tick(dev);
        Device::advance_to(dev, now + ticks);
    }

    #[derive(Debug, Default)]
    struct Probe {
        high: AtomicU32,
    }

    impl Probe {
        fn is_high(&self) -> bool {
            self.high.load(Ordering::Relaxed) != 0
        }
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.high
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    fn watch_irq(dev: &Icache) -> (Arc<Wire>, Arc<Probe>) {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let sink = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(dev, IRQ_PIN, WireSource::new(Arc::clone(&wire), id)).unwrap();
        (wire, sink)
    }

    #[test]
    fn the_icache_resets_two_way_and_disabled() {
        let ic = cache(Variant::Icache);
        assert_eq!(peek(&ic, CR), IC_CR_WAYSEL);
        assert_eq!(peek(&ic, SR), 0);
        for i in 0..4 {
            assert_eq!(peek(&ic, CRR0 + 4 * i), CRR_RESET);
        }
    }

    #[test]
    fn hal_icache_invalidate_terminates() {
        let ic = cache(Variant::Icache);
        assert_eq!(peek(&ic, SR) & SR_BUSYF, 0);
        poke(&ic, CR, peek(&ic, CR) | CR_CACHEINV);
        assert_eq!(peek(&ic, SR), SR_BUSYF, "busy");
        assert_ne!(peek(&ic, CR) & CR_CACHEINV, 0, "CACHEINV stands while busy");
        let mut spins = 0;
        while peek(&ic, SR) & SR_BSYENDF == 0 {
            tick(&ic, 1);
            spins += 1;
            assert!(spins < 100);
        }
        assert_eq!(spins, DELAY);
        assert_eq!(peek(&ic, SR), SR_BSYENDF);
        assert_eq!(peek(&ic, CR) & CR_CACHEINV, 0);
        poke(&ic, FCR, SR_BSYENDF);
        assert_eq!(peek(&ic, SR), 0);
        assert_eq!(peek(&ic, FCR), 0, "FCR reads zero");
    }

    #[test]
    fn disabling_the_icache_starts_an_invalidation() {
        let ic = cache(Variant::Icache);
        poke(&ic, CR, CR_EN | IC_CR_WAYSEL);
        assert_eq!(peek(&ic, SR), 0, "enabling starts nothing");
        poke(&ic, FCR, SR_BSYENDF);
        poke(&ic, CR, IC_CR_WAYSEL);
        assert_eq!(peek(&ic, CR) & CR_EN, 0, "HAL_ICACHE_Disable's spin ends");
        assert_eq!(peek(&ic, SR), SR_BUSYF);
        tick(&ic, DELAY);
        assert_eq!(peek(&ic, SR), SR_BSYENDF);
    }

    #[test]
    fn waysel_and_the_regions_are_frozen_while_enabled() {
        let ic = cache(Variant::Icache);
        poke(&ic, CR, CR_EN | IC_CR_WAYSEL);
        poke(&ic, CR, CR_EN);
        assert_eq!(peek(&ic, CR), CR_EN | IC_CR_WAYSEL, "WAYSEL kept");
        poke(&ic, CRR0, 0xffff_ffff);
        assert_eq!(peek(&ic, CRR0), CRR_RESET);
        poke(&ic, CR, 0);
        tick(&ic, DELAY);
        poke(&ic, CR, 0);
        assert_eq!(peek(&ic, CR), 0, "one-way now");
        poke(&ic, CRR0, 0xffff_ffff);
        assert_eq!(peek(&ic, CRR0), CRR_MASK);
    }

    #[test]
    fn bsyendie_drives_the_irq() {
        let ic = cache(Variant::Icache);
        let (_w, probe) = watch_irq(&ic);
        poke(&ic, IER, 0xffff_ffff);
        assert_eq!(peek(&ic, IER), SR_BSYENDF | SR_ERRF);
        poke(&ic, CR, CR_CACHEINV);
        assert!(!probe.is_high());
        assert_eq!(Device::next_event_tick(&ic), Some(DELAY));
        tick(&ic, DELAY);
        assert!(probe.is_high());
        poke(&ic, FCR, SR_BSYENDF);
        assert!(!probe.is_high());
    }

    #[test]
    fn the_monitors_read_zero() {
        let ic = cache(Variant::Icache);
        poke(&ic, CR, IC_CR_MONEN | (0b11 << 18));
        assert_eq!(peek(&ic, CR), IC_CR_MONEN, "the reset bits read zero");
        assert_eq!(peek(&ic, 0x10), 0);
        assert_eq!(peek(&ic, 0x14), 0);
    }

    #[test]
    fn a_dcache_command_runs_to_cmdendf() {
        let dc = cache(Variant::Dcache);
        let (_w, probe) = watch_irq(&dc);
        poke(&dc, IER, SR_CMDENDF);
        poke(&dc, CMDRSADDRR, 0x2000_0013);
        poke(&dc, CMDREADDRR, 0x2000_00ff);
        assert_eq!(peek(&dc, CMDRSADDRR), 0x2000_0010);
        // Clean and invalidate (CACHECMD = 011) by address.
        poke(&dc, CR, CR_EN | (0b011 << 8));
        poke(&dc, CR, CR_EN | (0b011 << 8) | DC_CR_STARTCMD);
        assert_eq!(peek(&dc, SR), SR_BUSYCMDF);
        assert_eq!(peek(&dc, CR), CR_EN | (0b011 << 8), "STARTCMD reads zero");
        tick(&dc, DELAY);
        assert_eq!(peek(&dc, SR), SR_CMDENDF);
        assert!(probe.is_high());
        poke(&dc, FCR, SR_CMDENDF);
        assert_eq!(peek(&dc, SR), 0);
        assert!(!probe.is_high());

        // No command selected: STARTCMD does nothing.
        poke(&dc, CR, CR_EN | DC_CR_STARTCMD);
        assert_eq!(peek(&dc, SR), 0);
    }

    #[test]
    fn a_dcache_full_invalidate_clears_busyf() {
        let dc = cache(Variant::Dcache);
        poke(&dc, CR, CR_EN | CR_CACHEINV);
        assert_eq!(peek(&dc, SR) & SR_BUSYF, SR_BUSYF);
        tick(&dc, DELAY);
        assert_eq!(peek(&dc, SR), SR_BSYENDF);
        // Disabling a DCACHE starts nothing (see the module notes).
        poke(&dc, FCR, SR_BSYENDF);
        poke(&dc, CR, 0);
        assert_eq!(peek(&dc, SR), 0);
    }

    #[test]
    fn a_zero_delay_finishes_inside_the_write() {
        let ic = Icache::with_config(Variant::Icache, 0);
        poke(&ic, CR, CR_CACHEINV);
        assert_eq!(peek(&ic, SR), SR_BSYENDF);
        assert_eq!(Device::next_event_tick(&ic), None);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        let ic = cache(Variant::Icache);
        poke(&ic, CR, CR_CACHEINV);
        assert_eq!(peek_debug(&ic, SR), SR_BUSYF);
        assert_eq!(Device::current_tick(&ic), 0);
        assert!(
            ic.regs
                .write(FCR, &u32::MAX.to_le_bytes(), MemAttrs::DEBUG)
                .is_err()
        );
        let mut half = [0u8; 2];
        assert!(ic.regs.read(SR, &mut half, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_gated_dcache_reads_zero_and_waits() {
        let dc = cache(Variant::Dcache);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let pin = Device::sink(&dc, gate::ENABLE_PIN, &[id]).expect("enable");
        pin.sink.set_level(id, pin.line, Level::High);
        poke(&dc, CR, CR_CACHEINV);
        pin.sink.set_level(id, pin.line, Level::Low);
        assert_eq!(peek(&dc, SR), 0, "a gated block answers nothing");
        tick(&dc, DELAY * 2);
        pin.sink.set_level(id, pin.line, Level::High);
        assert_eq!(peek(&dc, SR), SR_BUSYF, "and counted nothing");
        tick(&dc, DELAY);
        assert_eq!(peek(&dc, SR), SR_BSYENDF);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        for variant in [Variant::Icache, Variant::Dcache] {
            let saved = cache(variant);
            poke(&saved, IER, SR_BSYENDF);
            poke(&saved, CR, CR_CACHEINV | (0b001 << 8));
            if variant == Variant::Dcache {
                poke(&saved, CR, (0b001 << 8) | DC_CR_STARTCMD);
            }
            tick(&saved, 1);

            let mut shape = MachineShape::new();
            shape.add_device("cache", CLASS_NAME).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("cache", CLASS_NAME, STATE_VERSION).unwrap();
                Device::save(&saved, &mut chunk).unwrap();
            }
            let bytes = w.to_vec().unwrap();
            let restored = cache(variant);
            let reader = StateReader::new(&bytes).unwrap();
            let chunk = reader
                .load("cache", CLASS_NAME, STATE_VERSION, &Migrations::new())
                .unwrap();
            Device::load(&restored, &mut chunk.reader()).unwrap();

            let saved_state = *saved.regs.state.lock();
            assert_eq!(saved_state, *restored.regs.state.lock());
            let words = REGISTER_BYTES / 4;
            let before: Vec<u32> = (0..words).map(|i| peek_debug(&saved, i * 4)).collect();
            let after: Vec<u32> = (0..words).map(|i| peek_debug(&restored, i * 4)).collect();
            assert_eq!(before, after, "{variant:?}");
            assert_eq!(
                Device::next_event_tick(&saved),
                Device::next_event_tick(&restored)
            );
            tick(&restored, DELAY);
            assert_ne!(peek(&restored, SR) & SR_BSYENDF, 0);
        }
    }

    #[test]
    fn the_class_is_registrable_and_agrees_with_its_schema() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err());
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);

        let props = Props::new().with("variant", Value::from("dcache"));
        assert_eq!(Icache::new(&props).unwrap().variant(), Variant::Dcache);
        assert!(Icache::new(&Props::new().with("variant", Value::from("l2"))).is_err());
        assert!(Icache::new(&Props::new().with("busy_delay", Value::from(1u64))).is_err());

        let schema = schema();
        assert_eq!(
            schema.port_named(IRQ_PIN).map(|p| p.dir),
            Some(PortDir::Out)
        );
        assert!(schema.port_named(gate::ENABLE_PIN).is_some());
        let ic = cache(Variant::Icache);
        assert!(Device::region(&ic, "").is_some());
        assert!(Device::region(&ic, "fifo").is_none());
        assert!(Device::is_lazy(&ic));
        let ids = WireIdAllocator::new();
        let wire = Wire::builder().source(ids.alloc()).build_shared();
        assert!(Device::connect(&ic, "busy", WireSource::new(wire, ids.alloc())).is_err());
    }
}
