//! The STM32 external interrupt/event controller.
//!
//! One class, `st.exti`. It is the piece that turns *a pin moved* into *the
//! core took an interrupt*: every button, every card-detect switch, every
//! "data ready" line an external chip raises, and the RTC alarm and USB wakeup
//! lines, reach the NVIC through here and nowhere else.
//!
//! Two register maps, chosen by `variant`: `"f4"` (the default, and `"l4"`,
//! which is the same map) is the RM0090/RM0351 block described first below;
//! `"u5"` is the STM32U5's, which ST redesigned — see "The U5", further down.
//!
//! # The registers
//!
//! Six of them per bank of 32 lines (RM0090 §12.3, RM0351 §13.5):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `IMR1` | one bit per line: let the pending bit reach the interrupt output |
//! | `0x04` | `EMR1` | the same for the *event* output, which is what wakes a `WFE` |
//! | `0x08` | `RTSR1` | one bit per line: latch on a rising edge |
//! | `0x0c` | `FTSR1` | one bit per line: latch on a falling edge |
//! | `0x10` | `SWIER1` | write a one to raise the line from software |
//! | `0x14` | `PR1` | the pending bits. **Write one to clear** |
//!
//! An L4 has more than 32 lines, so it repeats the six at `0x20`…`0x34` for
//! lines 32 upwards. `lines` says how many the part has and the second bank
//! decodes only when that is more than 32; an F4 (23 lines) therefore answers
//! `0x00`…`0x17` and nothing above it, which is what the part does.
//!
//! # What a line is
//!
//! A line is a *level* in and an interrupt request out. Lines 0–15 are the
//! GPIO lines and the level arrives from `st.syscfg`, which is where
//! `EXTICR1`…`EXTICR4` choose which port's pin *n* drives line *n*. Lines 16
//! upwards are peripheral wakeups — PVD, the RTC alarm, the OTG FS wakeup —
//! and a board wires those straight from the peripheral.
//!
//! So the wiring of a GPIO interrupt is three statements and the middle one is
//! the mux:
//!
//! ```text
//! wire gpioa.p0    -> syscfg.pa0      # the pin, as the port drives it
//! wire syscfg.exti0 -> exti.line0     # whichever port EXTICR1 selected
//! wire exti.irq0   -> cpu.irq6        # RM0090 Table 62
//! ```
//!
//! **The interrupt numbers are not here.** Each line has its own `irq{n}`
//! output and the board decides where it lands: on both an F4 and an L4 that
//! is IRQ 6–10 for lines 0–4, IRQ 23 for lines 5–9 and IRQ 40 for lines 10–15,
//! so five `irq` pins are wired to one core pin and the wire's fan-in resolves
//! them. A model that knew those numbers would be a model of one part.
//!
//! # Edges
//!
//! `RTSR` and `FTSR` are independent and neither is implied: a line with
//! neither set latches nothing however hard it is driven, which is the state
//! every line comes out of reset in. `PR` is set by the selected edge whether
//! or not `IMR` or `EMR` lets it out — masking hides the request, it does not
//! stop the detector — and `SWIER` sets it with no edge at all. Clearing `PR`
//! is a write of one to it, and it also clears the matching `SWIER` bit, which
//! is the only way a software request goes away (RM0090 §12.3.5).
//!
//! # The event output
//!
//! `EMR` gates a *pulse* rather than a level: an event does not go to the
//! NVIC, it wakes a core sitting in `WFE`. One `event` pin carries all of
//! them, because a wakeup has no number — a core that has woken reads the
//! pending bits like everyone else. No core honours it yet; the pin is the
//! seam for the one that will.
//!
//! # The U5
//!
//! RM0456 §22 (EXTI) rebuilt the block, and `variant = "u5"` models it as a
//! `TZEN = 0` STM32U575/U585. The offsets are `EXTI_TypeDef` in ST CMSIS
//! `stm32u585xx.h` (Apache-2.0):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `RTSR1` | latch on a rising edge |
//! | `0x04` | `FTSR1` | latch on a falling edge |
//! | `0x08` | `SWIER1` | write one: a rising-edge event from software; reads zero |
//! | `0x0c` | `RPR1` | rising-edge pending bits, **write one to clear** |
//! | `0x10` | `FPR1` | falling-edge pending bits, **write one to clear** |
//! | `0x14` | `SECCFGR1` | security per line — RAZ/WI on a `TZEN = 0` part |
//! | `0x18` | `PRIVCFGR1` | privilege per line: stored, not enforced |
//! | `0x60`–`0x6c` | `EXTICR1`–`EXTICR4` | **the GPIO port mux**: one four-bit field per line, at bits `8k..8k+3` |
//! | `0x70` | `LOCKR` | `LOCK`: set once, freezes `PRIVCFGR1` until reset |
//! | `0x80` | `IMR1` | interrupt mask |
//! | `0x84` | `EMR1` | event mask |
//!
//! Three things a board author notices:
//!
//! * **The mux is in here.** On a U5 `EXTICR` is an EXTI register, not a
//!   SYSCFG one, so this device takes the port pins itself, with the names
//!   `st.syscfg` gives them: `pa0`…`pa15`, `pb0`…, up to the `ports`-th
//!   letter (nine by default, A–I). Field value 0 is port A, 8 is port I; a
//!   value naming a port the package lacks selects nothing. Writing `EXTICR`
//!   re-latches the newly selected pin's level at once, so pointing a line at
//!   a pin already high is a rising edge — the same spurious interrupt
//!   `st.syscfg` documents. Lines 16 upwards (16 PVD, 17 COMP1, 18 COMP2, and
//!   the rest of RM0456's table up to 23) are still `line16`…`line23` inputs,
//!   wired from the peripheral; `line0`…`line15` are not inputs on a U5, since
//!   the mux drives them.
//! * **Two pending registers.** A rising edge sets `RPR1`, a falling edge
//!   `FPR1`, and the interrupt is their OR under `IMR1`. `SWIER1` is a strobe
//!   that sets `RPR1` and always reads zero.
//! * **One vector per GPIO line.** A U5 gives `EXTI0`…`EXTI15` their own NVIC
//!   vectors, 11–26 (`EXTI0_IRQn` = 11 in `stm32u585xx.h`), so a board wires
//!   `irq{n}` to `cpu.irq{11 + n}` one for one; there is no fan-in to resolve.
//!   The peripheral lines' interrupts reach the NVIC directly from their
//!   peripherals on a U5, so `irq16`… are usually left unwired.
//!
//! ```text
//! object exti "st.exti" { variant = "u5" }
//! wire gpioc.p13 -> exti.pc13          # the user button
//! wire exti.irq13 -> cpu.irq24         # EXTI13_IRQn
//! ```
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §12 "Interrupts and events", §12.3 for the register
//! map and the write-one-to-clear behaviour of `PR`; ST **RM0351** rev 9 §13
//! for the second bank and for the lines an L4 puts above 32; ST **RM0456**
//! §22 and ST CMSIS `stm32u585xx.h` (Apache-2.0) for the U5. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::BusError;
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.exti";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many lines the widest part this class models has — an L4's 40.
pub const MAX_LINES: u32 = 40;

/// How many lines an F4 has, and the default: 0–22 (RM0090 §12.2.5).
pub const DEFAULT_LINES: u32 = 23;

/// How many lines a U575/U585 has: `IM0`…`IM23` in `stm32u585xx.h`.
pub const U5_LINES: u32 = 24;

/// How many lines a GPIO pin can drive: one per pin number.
pub const GPIO_LINES: u32 = 16;

/// How many GPIO ports a U5's `EXTICR` can name — the same eleven letters
/// `st.syscfg` takes.
pub const MAX_PORTS: u32 = 11;

/// How many ports a U575/U585 bonds by default: A–I.
pub const U5_PORTS: u32 = 9;

/// The port letters, in `EXTICR` order, as `st.syscfg` spells them.
const PORT_LETTERS: &[u8] = b"abcdefghijk";

/// How many bytes one bank of six registers occupies.
const BANK_BYTES: u64 = 0x18;

/// Where the second bank starts (RM0351 §13.5.7).
const BANK2_BASE: u64 = 0x20;

/// How many bytes the two banks occupy together.
const TWO_BANK_BYTES: u64 = BANK2_BASE + BANK_BYTES;

/// How many bytes the U5 block decodes: through `EMR1` at `+0x84`.
const U5_BYTES: u64 = 0x88;

/// U5 `EXTICRx`: the four-bit port field of each of four lines, eight bits
/// apart (`EXTI_EXTICR1_EXTI0_Msk` = `0xF`, `EXTI1_Pos` = 8).
const U5_EXTICR_MASK: u32 = 0x0f0f_0f0f;

/// Marks a U5 snapshot's line count, so a U5 chunk and an F4 chunk can never
/// be loaded into each other. An F4 chunk is exactly the shape it was before
/// the U5 existed.
const U5_SNAPSHOT_TAG: u32 = 0x0100_0000;

/// Which register map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090 §12.3 / RM0351 §13.5: `IMR`/`EMR`/`RTSR`/`FTSR`/`SWIER`/`PR`
    /// per bank, the mux in `st.syscfg`.
    Classic,
    /// RM0456 §22: separate rising and falling pending registers and the
    /// `EXTICR` mux in this block.
    U5,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the edge detector's memory.
///
/// Two words per register because an L4 has two banks; a part with 32 lines or
/// fewer leaves index 1 at zero and never decodes it. On a U5, `pr` is
/// `RPR1` and `fpr` is `FPR1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct State {
    imr: [u32; 2],
    emr: [u32; 2],
    rtsr: [u32; 2],
    ftsr: [u32; 2],
    swier: [u32; 2],
    pr: [u32; 2],
    /// The level each line is latched at.
    ///
    /// **Saved**, unlike `st.gpio`'s pad levels, and deliberately: this is not
    /// the controller reporting what somebody else is driving, it is the edge
    /// detector's own memory of the last level it saw. A snapshot that dropped
    /// it would come back believing every line was low, and the first time a
    /// line that had been high was driven low the detector would see no edge —
    /// or, worse, the first re-drive high would look like an edge that never
    /// happened and post an interrupt out of nowhere.
    input: [u32; 2],
    /// U5 `FPR1`.
    fpr: u32,
    /// U5 `PRIVCFGR1`.
    privcfgr: u32,
    /// U5 `LOCKR.LOCK`.
    lock: bool,
    /// U5 `EXTICR1`…`EXTICR4`.
    exticr: [u32; 4],
    /// U5: one bit per pin, per port — the level each port is driving, which
    /// the mux re-latches when `EXTICR` moves. Saved for the reason
    /// `st.syscfg` saves its copy.
    ports: [u16; MAX_PORTS as usize],
}

/// Which bank and which bit within it line `n` is.
const fn split(n: u32) -> (usize, u32) {
    ((n / 32) as usize, 1u32 << (n % 32))
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    /// One bit per line that exists, per bank.
    valid: [u32; 2],
    /// Bits `IMR` reads back as ones however it is written — an L4 reserves
    /// some of its `IMR1` that way (RM0351 §13.5.1).
    imr_ones: [u32; 2],
    /// How many lines the part has.
    lines: u32,
    /// U5: how many GPIO ports `EXTICR` can select. Zero on the classic map.
    ports: u32,
    /// The per-line interrupt outputs, connected at realize time.
    irq: Mutex<Vec<Option<WireSource>>>,
    /// The one event output. A pulse, not a level — see the module docs.
    event: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("lines", &self.lines);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// The reset state: everything clear except `IMR`'s reserved ones — and,
    /// on a U5, the port levels, which are what the ports are still driving.
    /// The detector's latch for lines 0–15 starts at whatever the reset mux
    /// (every line on port A) now selects, so a pin held high across a reset
    /// is not mistaken for an edge the next time it is selected.
    fn reset_state(&self, ports: [u16; MAX_PORTS as usize]) -> State {
        let mut state = State {
            imr: self.imr_ones,
            ..State::default()
        };
        if self.variant == Variant::U5 {
            state.ports = ports;
            for n in 0..GPIO_LINES {
                if self.mux_level(&state, n) {
                    state.input[0] |= 1 << n;
                }
            }
        }
        state
    }

    /// One bit per line whose pending bit is unmasked — what `irq{n}` carries.
    fn irq_levels(&self, state: &State) -> [u32; 2] {
        match self.variant {
            Variant::Classic => [state.pr[0] & state.imr[0], state.pr[1] & state.imr[1]],
            Variant::U5 => [(state.pr[0] | state.fpr) & state.imr[0], 0],
        }
    }

    /// Whether line `n`'s pending bit is set, masked or not.
    fn is_pending(&self, state: &State, n: u32) -> bool {
        let (bank, bit) = split(n);
        let fpr = if self.variant == Variant::U5 && bank == 0 {
            state.fpr
        } else {
            0
        };
        (state.pr[bank] | fpr) & bit != 0
    }

    /// Drive the interrupt outputs.
    ///
    /// Called with **no lock held**: a sink is free to call back into this
    /// device, so the outward call happens after the critical section
    /// (`CLAUDE.md`, "Concurrency").
    fn drive(&self, levels: [u32; 2]) {
        let sources: Vec<Option<WireSource>> = self.irq.lock().clone();
        for (n, source) in sources.iter().enumerate() {
            let Some(source) = source else { continue };
            let (bank, bit) = split(n as u32);
            source.set(Level::from_bool(levels[bank] & bit != 0));
        }
    }

    /// Pulse the event output. Also outside the lock, for the same reason.
    fn pulse_event(&self) {
        let source = self.event.lock().clone();
        if let Some(source) = source {
            source.pulse(Level::High);
        }
    }

    /// Republish the interrupt outputs from whatever the state now says.
    fn refresh(&self) {
        let levels = self.irq_levels(&self.state.lock());
        self.drive(levels);
    }

    /// Which register `offset` names: the bank, and the register within it.
    ///
    /// `None` for an offset this part does not decode — the second bank on a
    /// part with 32 lines or fewer, and the gap between the banks.
    fn decode(&self, offset: u64) -> Option<(usize, u64)> {
        if offset < BANK_BYTES {
            return Some((0, offset));
        }
        if self.lines > 32 && (BANK2_BASE..BANK2_BASE + BANK_BYTES).contains(&offset) {
            return Some((1, offset - BANK2_BASE));
        }
        None
    }

    /// Read one register.
    fn read_register(&self, offset: u64) -> u32 {
        if self.variant == Variant::U5 {
            return self.read_u5(offset);
        }
        let Some((bank, reg)) = self.decode(offset) else {
            return 0;
        };
        let state = self.state.lock();
        match reg {
            0x00 => state.imr[bank],
            0x04 => state.emr[bank],
            0x08 => state.rtsr[bank],
            0x0c => state.ftsr[bank],
            0x10 => state.swier[bank],
            0x14 => state.pr[bank],
            _ => 0,
        }
    }

    /// Read one U5 register. Nothing here clears on read.
    fn read_u5(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            0x00 => state.rtsr[0],
            0x04 => state.ftsr[0],
            0x0c => state.pr[0],
            0x10 => state.fpr,
            0x18 => state.privcfgr,
            0x60..=0x6c => state.exticr[((offset - 0x60) / 4) as usize],
            0x70 => u32::from(state.lock),
            0x80 => state.imr[0],
            0x84 => state.emr[0],
            // `SWIER1` "always reads 0"; `SECCFGR1` is RAZ on a `TZEN = 0`
            // part; everything else is reserved.
            _ => 0,
        }
    }

    /// Write one register. Returns whether an event pulse is owed.
    ///
    /// The interrupt outputs are republished by the caller; this returns only
    /// the thing that is not a level.
    fn write_register(&self, offset: u64, value: u32) -> bool {
        if self.variant == Variant::U5 {
            return self.write_u5(offset, value);
        }
        let Some((bank, reg)) = self.decode(offset) else {
            return false;
        };
        let valid = self.valid[bank];
        let mut state = self.state.lock();
        match reg {
            // The reserved ones read back as ones whatever is written to them.
            0x00 => state.imr[bank] = (value & valid) | self.imr_ones[bank],
            0x04 => state.emr[bank] = value & valid,
            0x08 => state.rtsr[bank] = value & valid,
            0x0c => state.ftsr[bank] = value & valid,
            0x10 => {
                // "Writing a 1 to this bit when it is at 0 sets the
                // corresponding pending bit" (RM0090 §12.3.5). A bit already
                // set does nothing, which is why this is the rising edge of
                // `SWIER` rather than its level.
                let rising = (value & valid) & !state.swier[bank];
                state.swier[bank] |= value & valid;
                state.pr[bank] |= rising;
                return rising & state.emr[bank] != 0;
            }
            0x14 => {
                // Write one to clear, and it takes the software request with
                // it: "this bit is cleared by clearing the corresponding bit
                // of EXTI_PR".
                let clear = value & valid;
                state.pr[bank] &= !clear;
                state.swier[bank] &= !clear;
            }
            _ => {}
        }
        false
    }

    /// Write one U5 register (RM0456 §22). Returns whether an event is owed.
    fn write_u5(&self, offset: u64, value: u32) -> bool {
        let valid = self.valid[0];
        let mut state = self.state.lock();
        match offset {
            0x00 => state.rtsr[0] = value & valid,
            0x04 => state.ftsr[0] = value & valid,
            0x08 => {
                // A strobe: "a rising edge event on line x, resulting in the
                // corresponding pending bit being set" in `RPR1`, whatever
                // `RTSR1` says. Nothing is stored, so writing the same one
                // twice raises it twice.
                let bits = value & valid;
                state.pr[0] |= bits;
                return bits & state.emr[0] != 0;
            }
            0x0c => state.pr[0] &= !(value & valid),
            0x10 => state.fpr &= !(value & valid),
            0x18 if !state.lock => state.privcfgr = value & valid,
            0x60..=0x6c => {
                state.exticr[((offset - 0x60) / 4) as usize] = value & U5_EXTICR_MASK;
                // The newly selected pins' levels reach the detector at once.
                return self.remux(&mut state);
            }
            0x70 => state.lock |= value & 1 != 0,
            0x80 => state.imr[0] = (value & valid) | self.imr_ones[0],
            0x84 => state.emr[0] = value & valid,
            // `SECCFGR1` is RAZ/WI on a `TZEN = 0` part; `PRIVCFGR1` is
            // frozen once `LOCK` is set.
            _ => {}
        }
        false
    }

    /// The level the U5 mux presents to GPIO line `n`: the pin of the port
    /// `EXTICR` names, or low if it names a port the package does not bond.
    fn mux_level(&self, state: &State, n: u32) -> bool {
        let field = (state.exticr[(n / 4) as usize] >> ((n % 4) * 8)) & 0xf;
        field < self.ports && state.ports[field as usize] & (1 << n) != 0
    }

    /// Re-latch every GPIO line from the mux. Whether an event is owed.
    fn remux(&self, state: &mut State) -> bool {
        let mut event = false;
        for n in 0..GPIO_LINES {
            let high = self.mux_level(state, n);
            event |= self.latch(state, n, high);
        }
        event
    }

    /// Line `n` is now at `high`. Latch a pending bit if the edge is one
    /// `RTSR`/`FTSR` asked for; return whether an event pulse is owed.
    fn latch(&self, state: &mut State, n: u32, high: bool) -> bool {
        let (bank, bit) = split(n);
        let was = state.input[bank] & bit != 0;
        if was == high {
            return false;
        }
        if high {
            state.input[bank] |= bit;
        } else {
            state.input[bank] &= !bit;
        }
        let selected = if high {
            state.rtsr[bank]
        } else {
            state.ftsr[bank]
        };
        let latched = selected & bit;
        if self.variant == Variant::U5 && !high {
            state.fpr |= latched;
        } else {
            state.pr[bank] |= latched;
        }
        latched & state.emr[bank] != 0
    }

    /// A line moved. Latch, then drive the outputs outside the lock.
    fn set_line(&self, n: u32, high: bool) {
        if n >= self.lines {
            return;
        }
        let (levels, event) = {
            let mut state = self.state.lock();
            let event = self.latch(&mut state, n, high);
            (self.irq_levels(&state), event)
        };
        self.drive(levels);
        if event {
            self.pulse_event();
        }
    }

    /// U5: pin `pin` of port `port` moved. If `EXTICR` routes it to its line,
    /// that line moved too.
    fn set_port_pin(&self, port: u32, pin: u32, high: bool) {
        if port >= self.ports || pin >= GPIO_LINES {
            return;
        }
        let (levels, event) = {
            let mut state = self.state.lock();
            let bit = 1u16 << pin;
            if high {
                state.ports[port as usize] |= bit;
            } else {
                state.ports[port as usize] &= !bit;
            }
            let level = self.mux_level(&state, pin);
            let event = self.latch(&mut state, pin, level);
            (self.irq_levels(&state), event)
        };
        self.drive(levels);
        if event {
            self.pulse_event();
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing here clears on read — `PR` needs a write of one — so a debug
        // read is the same read (`ROADMAP.md` §15, invariant 5).
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `PR` would drop an interrupt the guest has not
            // seen and one to `SWIER` would post one it never asked for.
            // Neither has a harmless version.
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        let event = self.write_register(offset & !3, value);
        self.refresh();
        if event {
            self.pulse_event();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers can be accessed by words (32-bit) only"
        // (RM0090 §12.3). A byte write to `PR` would be a guess about which
        // bits the guest meant to clear.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 external interrupt/event controller.
#[derive(Debug)]
pub struct Exti {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins the machine layer has taken. The device keeps the strong
    /// references: a net holds its sinks weakly.
    pins: Mutex<Vec<Arc<LinePin>>>,
    /// The same for a U5's port pins.
    port_pins: Mutex<Vec<Arc<PortPin>>>,
}

impl Exti {
    /// Validate `props` and build the controller.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given; [`Error::Config`] for `ports`
    /// on the classic map, which has no mux.
    pub fn new(props: &Props) -> Result<Exti> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "l4", "u5"])? {
            "u5" => Variant::U5,
            _ => Variant::Classic,
        };
        let default_lines = match variant {
            Variant::Classic => DEFAULT_LINES,
            Variant::U5 => U5_LINES,
        };
        let (min_lines, max_lines) = match variant {
            Variant::Classic => (1, MAX_LINES),
            // Every GPIO line, and one bank: the U5 map has no second set of
            // registers.
            Variant::U5 => (GPIO_LINES, 32),
        };
        let lines = r.or_range(
            "lines",
            u64::from(default_lines),
            u64::from(min_lines)..=u64::from(max_lines),
        )? as u32;
        let imr0 = r.or_range("imr-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        let imr1 = r.or_range("imr2-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        let ports = r.optional::<u64>("ports")?;
        r.finish()?;
        if let Some(p) = ports
            && !(1..=u64::from(MAX_PORTS)).contains(&p)
        {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!("`ports` is 1 to {MAX_PORTS} (A to K), not {p}"),
            });
        }
        match variant {
            Variant::Classic => {
                if ports.is_some() {
                    return Err(Error::Config {
                        at: String::from(CLASS_NAME),
                        message: String::from(
                            "`ports` sizes the U5's EXTICR mux; on an F4 or L4 the mux is \
                             `st.syscfg`'s",
                        ),
                    });
                }
                Ok(Exti::with_lines(lines, [imr0, imr1]))
            }
            Variant::U5 => Ok(Exti::u5(lines, ports.map_or(U5_PORTS, |p| p as u32), imr0)),
        }
    }

    /// Build one with `lines` lines and the given `IMR` reserved-ones — the
    /// route a test takes.
    #[must_use]
    pub fn with_lines(lines: u32, imr_ones: [u32; 2]) -> Exti {
        Exti::build(Variant::Classic, lines.clamp(1, MAX_LINES), 0, imr_ones)
    }

    /// Build a U5 controller with `lines` lines and `ports` GPIO ports behind
    /// its `EXTICR` mux.
    #[must_use]
    pub fn u5(lines: u32, ports: u32, imr_ones: u32) -> Exti {
        Exti::build(
            Variant::U5,
            lines.clamp(GPIO_LINES, 32),
            ports.clamp(1, MAX_PORTS),
            [imr_ones, 0],
        )
    }

    fn build(variant: Variant, lines: u32, ports: u32, imr_ones: [u32; 2]) -> Exti {
        let valid = [bank_mask(lines, 0), bank_mask(lines, 1)];
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            variant,
            valid,
            // A reserved one outside the lines the part has would read back as
            // a line that does not exist.
            imr_ones: [imr_ones[0] & valid[0], imr_ones[1] & valid[1]],
            lines,
            ports,
            irq: Mutex::with_rank(LockRank::WIRE, alloc::vec![None; lines as usize]),
            event: Mutex::with_rank(LockRank::WIRE, None),
        });
        *regs.state.lock() = regs.reset_state([0; MAX_PORTS as usize]);
        let bytes = match variant {
            Variant::U5 => U5_BYTES,
            Variant::Classic if lines > 32 => TWO_BANK_BYTES,
            Variant::Classic => BANK_BYTES,
        };
        let region = Arc::new(Region::io(
            "exti",
            bytes,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Exti {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
            port_pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Which register map this controller has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// How many lines this controller has.
    #[must_use]
    pub fn lines(&self) -> u32 {
        self.regs.lines
    }

    /// How many bytes of registers this controller decodes.
    #[must_use]
    pub fn register_bytes(&self) -> u64 {
        self.region.len()
    }

    /// Drive line `n` directly — the route a test takes when there is no wire.
    pub fn set_line(&self, n: u32, high: bool) {
        self.regs.set_line(n, high);
    }

    /// Drive pin `pin` of GPIO port `port` (A = 0) into a U5's mux.
    pub fn set_port_pin(&self, port: u32, pin: u32, high: bool) {
        self.regs.set_port_pin(port, pin, high);
    }

    /// Whether line `n`'s interrupt request is asserted.
    #[must_use]
    pub fn irq_asserted(&self, n: u32) -> bool {
        if n >= self.regs.lines {
            return false;
        }
        let (bank, bit) = split(n);
        self.regs.irq_levels(&self.regs.state.lock())[bank] & bit != 0
    }

    /// Whether line `n`'s pending bit is set, masked or not — on a U5, either
    /// of its rising and falling pending bits.
    #[must_use]
    pub fn pending(&self, n: u32) -> bool {
        if n >= self.regs.lines {
            return false;
        }
        self.regs.is_pending(&self.regs.state.lock(), n)
    }
}

/// One bit per line of bank `bank` that a part with `lines` lines has.
fn bank_mask(lines: u32, bank: u32) -> u32 {
    let low = bank * 32;
    if lines <= low {
        return 0;
    }
    let count = (lines - low).min(32);
    if count == 32 {
        u32::MAX
    } else {
        (1u32 << count) - 1
    }
}

/// Port index and pin for a U5 port sink named `port` (`pa0`…), if the
/// package has that port.
fn parse_pin(port: &str, ports: u32) -> Option<(u32, u32)> {
    for (index, letter) in PORT_LETTERS.iter().take(ports as usize).enumerate() {
        let prefix = format!("p{}", *letter as char);
        if let Some(pin) = port_index(port, &prefix, GPIO_LINES) {
            return Some((index as u32, pin));
        }
    }
    None
}

impl Device for Exti {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the lines.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. Every register resets to zero bar `IMR`'s reserved ones,
        // and the latched input levels go with them: a reset that kept a
        // pending bit would interrupt a kernel that had not enabled the line.
        //
        // The *input* latch is cleared too, which is the one arguable choice
        // here. It is right because the detector is what reset: whatever is
        // driving a line is still driving it, and the next edge it makes is
        // measured from wherever the line is now — but this model cannot ask
        // the net what level that is, so the honest position is "the detector
        // has seen nothing", and a line held high across a reset raises
        // nothing until it moves. A real EXTI behaves the same way: it has no
        // pending bit for a level, only for an edge.
        //
        // A U5's mux *does* know what its ports drive — those levels are its
        // inputs, not its state, and survive — so its GPIO lines start latched
        // at whatever the reset mux selects (see `reset_state`).
        let ports = self.regs.state.lock().ports;
        *self.regs.state.lock() = self.regs.reset_state(ports);
        self.regs.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        let u5 = self.regs.variant == Variant::U5;
        let tag = if u5 { U5_SNAPSHOT_TAG } else { 0 };
        w.write_u32(self.regs.lines | tag)?;
        for bank in 0..2 {
            for value in [
                state.imr[bank],
                state.emr[bank],
                state.rtsr[bank],
                state.ftsr[bank],
                state.swier[bank],
                state.pr[bank],
                state.input[bank],
            ] {
                w.write_u32(value)?;
            }
        }
        if u5 {
            w.write_u32(self.regs.ports)?;
            w.write_u32(state.fpr)?;
            w.write_u32(state.privcfgr)?;
            w.write_bool(state.lock)?;
            for value in state.exticr {
                w.write_u32(value)?;
            }
            w.write_seq_len(u64::from(MAX_PORTS))?;
            for value in state.ports {
                w.write_u16(value)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let u5 = self.regs.variant == Variant::U5;
        let tag = if u5 { U5_SNAPSHOT_TAG } else { 0 };
        let lines = r.read_u32()?;
        if lines != self.regs.lines | tag {
            return Err(Error::State(format!(
                "snapshot has an EXTI with {} line(s){}, this one has {}{}",
                lines & !U5_SNAPSHOT_TAG,
                if lines & U5_SNAPSHOT_TAG != 0 {
                    " on the U5 map"
                } else {
                    ""
                },
                self.regs.lines,
                if u5 { " on the U5 map" } else { "" },
            )));
        }
        let mut state = State::default();
        for bank in 0..2 {
            state.imr[bank] = r.read_u32()?;
            state.emr[bank] = r.read_u32()?;
            state.rtsr[bank] = r.read_u32()?;
            state.ftsr[bank] = r.read_u32()?;
            state.swier[bank] = r.read_u32()?;
            state.pr[bank] = r.read_u32()?;
            state.input[bank] = r.read_u32()?;
        }
        if u5 {
            let ports = r.read_u32()?;
            if ports != self.regs.ports {
                return Err(Error::State(format!(
                    "snapshot has an EXTI mux over {ports} port(s), this one has {}",
                    self.regs.ports
                )));
            }
            state.fpr = r.read_u32()?;
            state.privcfgr = r.read_u32()?;
            state.lock = r.read_bool()?;
            for value in &mut state.exticr {
                *value = r.read_u32()?;
            }
            let count = r.read_seq_len(2)?;
            if count != u64::from(MAX_PORTS) {
                return Err(Error::State(format!(
                    "snapshot has {count} port(s) of pin levels, this build keeps {MAX_PORTS}"
                )));
            }
            for value in &mut state.ports {
                *value = r.read_u16()?;
            }
        }
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port == "event" {
            *self.regs.event.lock() = Some(source);
            return Ok(());
        }
        let n = port_index(port, "irq", self.regs.lines).ok_or_else(|| Error::Config {
            at: port.to_string(),
            message: format!(
                "an EXTI drives `irq0`…`irq{}` and `event`",
                self.regs.lines - 1
            ),
        })?;
        self.regs.irq.lock()[n as usize] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        // Every output idles low out of reset, which a fresh net already is,
        // but a machine that wires a line after a snapshot load has to be told
        // about a pending bit that survived it.
        if port_index(port, "irq", self.regs.lines).is_some() {
            self.regs.refresh();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if self.regs.variant == Variant::U5
            && let Some((index, pin)) = parse_pin(port, self.regs.ports)
        {
            let sink = Arc::new(PortPin {
                regs: Arc::clone(&self.regs),
                port: index,
                pin,
                inputs: FanIn::new(sources),
            });
            self.port_pins.lock().push(Arc::clone(&sink));
            return Some(SinkPin { sink, line: pin });
        }
        let n = port_index(port, "line", self.regs.lines)?;
        if self.regs.variant == Variant::U5 && n < GPIO_LINES {
            // The mux drives lines 0–15 on a U5; a direct wire would bypass
            // `EXTICR`. Wire the port pin instead.
            return None;
        }
        let pin = Arc::new(LinePin {
            regs: Arc::clone(&self.regs),
            line: n,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: n })
    }
}

impl Instance for Exti {}

/// The `st.exti` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 external interrupt/event controller: IMR/EMR, RTSR/FTSR, SWIER and PR \
              (and, on a U5, RPR/FPR and the EXTICR port mux)",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register map: `f4` (default; `l4` is the same map) or `u5` (RM0456)",
        },
        PropertySpec {
            name: "lines",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many EXTI lines the part has (23 on an F4, up to 40 on an L4, 24 on a \
                      U5)",
        },
        PropertySpec {
            name: "ports",
            kind: ValueKind::Uint,
            required: false,
            summary: "U5 only: how many GPIO ports EXTICR can select, A upwards (default 9, A-I)",
        },
        PropertySpec {
            name: "imr-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "IMR1 bits that read back as ones however they are written",
        },
        PropertySpec {
            name: "imr2-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "the same for IMR2, on a part with more than 32 lines",
        },
    ],
    construct: |props| Ok(Box::new(Exti::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Exti::new(props)?)))
}

/// What the validator should know about `st.exti`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4", "u5"]))
        .prop(PropSchema::new("lines", ValueKind::Uint).range(1, u64::from(MAX_LINES)))
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, u64::from(MAX_PORTS)))
        .prop(PropSchema::new("imr-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("imr2-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        // The banks are declared at the widest part's size: `lines` narrows
        // what the *device* accepts, and the validator cannot see a property's
        // value. A machine that wires `line30` of a 23-line EXTI is caught by
        // the device, which is where the number lives.
        .port_bank("line", PortDir::In, MAX_LINES)
        .port_bank("irq", PortDir::Out, MAX_LINES)
        .port("event", PortDir::Out);
    // A U5's mux inputs, one bank per port letter, named as `st.syscfg` names
    // its own. Declared for every letter, as there; the device refuses a port
    // the package does not bond, and refuses them all on the classic map.
    for letter in PORT_LETTERS {
        schema = schema.port_bank(format!("p{}", *letter as char), PortDir::In, GPIO_LINES);
    }
    schema
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// One EXTI line, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the *driver that changed* rather than the resolved level of
/// the net (`ROADMAP.md` §4.3).
#[derive(Debug)]
pub struct LinePin {
    regs: Arc<Registers>,
    line: u32,
    inputs: FanIn,
}

impl LinePin {
    /// Which line this is.
    #[must_use]
    pub fn line(&self) -> u32 {
        self.line
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for LinePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_line(self.line, high);
    }
}

/// One GPIO pin into a U5's `EXTICR` mux, wire-ORing its sources as
/// [`LinePin`] does.
#[derive(Debug)]
pub struct PortPin {
    regs: Arc<Registers>,
    port: u32,
    pin: u32,
    inputs: FanIn,
}

impl PortPin {
    /// Which port this pin belongs to, A = 0.
    #[must_use]
    pub fn port(&self) -> u32 {
        self.port
    }

    /// Which pin of that port it is.
    #[must_use]
    pub fn pin(&self) -> u32 {
        self.pin
    }
}

impl WireSink for PortPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_port_pin(self.port, self.pin, high);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// An F4's controller: 23 lines, no reserved ones.
    fn exti() -> Exti {
        Exti::with_lines(DEFAULT_LINES, [0, 0])
    }

    fn peek(e: &Exti, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        e.regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(e: &Exti, offset: u64, value: u32) {
        e.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Watches one output pin, so a test can assert on the wire rather than on
    /// the device's own view of it.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
            if level.is_high() {
                self.edges.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Connect `port` to a fresh probe.
    fn watch(e: &Exti, port: &str) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(e, port, WireSource::new(wire, id)).expect("a pin this device drives");
        probe
    }

    fn high(p: &Probe) -> bool {
        p.level.load(Ordering::Relaxed) == 1
    }

    #[test]
    fn a_rising_edge_with_rtsr_sets_pr_and_asserts_the_line_s_irq() {
        let e = exti();
        let irq = watch(&e, "irq0");
        poke(&e, 0x00, 1); // IMR1
        poke(&e, 0x08, 1); // RTSR1
        e.set_line(0, true);
        assert_eq!(peek(&e, 0x14), 1, "PR1");
        assert!(e.irq_asserted(0));
        assert!(high(&irq), "the request reached the wire");
    }

    #[test]
    fn a_falling_edge_does_nothing_without_ftsr() {
        let e = exti();
        poke(&e, 0x00, 1);
        poke(&e, 0x08, 1); // rising only
        e.set_line(0, true);
        poke(&e, 0x14, 1); // clear it again
        assert_eq!(peek(&e, 0x14), 0);
        e.set_line(0, false);
        assert_eq!(peek(&e, 0x14), 0, "a falling edge is not asked for");

        // And with `FTSR` it is.
        poke(&e, 0x0c, 1);
        e.set_line(0, true);
        e.set_line(0, false);
        assert_eq!(peek(&e, 0x14), 1);
    }

    #[test]
    fn a_line_with_neither_trigger_latches_nothing() {
        let e = exti();
        poke(&e, 0x00, 0xffff);
        for n in 0..16 {
            e.set_line(n, true);
            e.set_line(n, false);
        }
        assert_eq!(peek(&e, 0x14), 0, "reset leaves every line untriggered");
    }

    #[test]
    fn writing_one_to_pr_clears_it_and_drops_the_irq_wire() {
        let e = exti();
        let irq = watch(&e, "irq3");
        poke(&e, 0x00, 1 << 3);
        poke(&e, 0x08, 1 << 3);
        e.set_line(3, true);
        assert!(high(&irq));

        // A write of zero clears nothing — this is write-one-to-clear.
        poke(&e, 0x14, 0);
        assert!(high(&irq));
        poke(&e, 0x14, 1 << 3);
        assert_eq!(peek(&e, 0x14), 0);
        assert!(!high(&irq), "the request went away with the pending bit");
    }

    #[test]
    fn imr_masks_the_interrupt_but_the_pending_bit_still_sets() {
        let e = exti();
        let irq = watch(&e, "irq7");
        poke(&e, 0x08, 1 << 7); // RTSR only; IMR left clear
        e.set_line(7, true);
        assert!(e.pending(7), "the detector runs whatever IMR says");
        assert!(!high(&irq));
        // Unmasking after the fact posts the request that was waiting, which
        // is the trap a driver that enables the line last falls into.
        poke(&e, 0x00, 1 << 7);
        assert!(high(&irq));
    }

    #[test]
    fn swier_sets_pr_without_a_wire_edge_and_pr_clears_it() {
        let e = exti();
        let irq = watch(&e, "irq2");
        poke(&e, 0x00, 1 << 2);
        poke(&e, 0x10, 1 << 2);
        assert_eq!(peek(&e, 0x14), 1 << 2, "PR1");
        assert_eq!(peek(&e, 0x10), 1 << 2, "SWIER1 reads back");
        assert!(high(&irq));

        // Writing the same one again does nothing: it is the 0→1 transition
        // of SWIER that posts the request.
        poke(&e, 0x14, 1 << 2);
        assert_eq!(peek(&e, 0x14), 0);
        assert_eq!(peek(&e, 0x10), 0, "clearing PR clears SWIER with it");
        poke(&e, 0x10, 1 << 2);
        assert_eq!(peek(&e, 0x14), 1 << 2, "and now it can be raised again");
    }

    #[test]
    fn emr_pulses_the_event_pin_and_imr_does_not() {
        let e = exti();
        let event = watch(&e, "event");
        let irq = watch(&e, "irq1");
        poke(&e, 0x08, 1 << 1); // RTSR1
        poke(&e, 0x00, 1 << 1); // IMR1
        e.set_line(1, true);
        assert!(high(&irq));
        assert_eq!(event.edges.load(Ordering::Relaxed), 0, "no event asked for");

        poke(&e, 0x14, 1 << 1);
        poke(&e, 0x04, 1 << 1); // EMR1
        e.set_line(1, false);
        e.set_line(1, true);
        assert_eq!(event.edges.load(Ordering::Relaxed), 1);
        assert!(!high(&event), "an event is a pulse, not a level");
        // And software can raise one too.
        poke(&e, 0x14, 1 << 1);
        poke(&e, 0x10, 1 << 1);
        assert_eq!(event.edges.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn every_line_has_its_own_request_pin_so_a_board_can_share_one_irq() {
        // RM0090 Table 62 puts lines 5–9 on IRQ 23. That is the board's
        // wiring, and what this class owes it is five separate pins.
        let e = exti();
        let probes: Vec<Arc<Probe>> = (5..10).map(|n| watch(&e, &format!("irq{n}"))).collect();
        poke(&e, 0x00, 0x3e0);
        poke(&e, 0x08, 0x3e0);
        e.set_line(7, true);
        assert!(high(&probes[2]), "line 7");
        for (i, p) in probes.iter().enumerate() {
            if i != 2 {
                assert!(!high(p), "only the line that moved");
            }
        }
    }

    #[test]
    fn a_line_the_part_does_not_have_is_neither_a_pin_nor_a_register_bit() {
        let e = exti();
        assert_eq!(e.lines(), 23);
        assert_eq!(e.register_bytes(), 0x18, "one bank on an F4");
        assert!(Device::sink(&e, "line23", &[WireId::new(1)]).is_none());
        assert!(Device::connect(&e, "irq23", dummy_source()).is_err());
        // A write that names them is masked off rather than remembered.
        poke(&e, 0x00, 0xffff_ffff);
        assert_eq!(peek(&e, 0x00), 0x007f_ffff);
        poke(&e, 0x10, 0xffff_ffff);
        assert_eq!(peek(&e, 0x14), 0x007f_ffff);
        // The second bank does not decode, and reads as zero rather than
        // aliasing the first.
        assert_eq!(peek(&e, 0x20), 0);
    }

    #[test]
    fn a_part_with_forty_lines_decodes_the_second_bank() {
        let e = Exti::with_lines(40, [0, 0]);
        assert_eq!(e.register_bytes(), TWO_BANK_BYTES);
        poke(&e, 0x20, 0xffff_ffff); // IMR2
        assert_eq!(peek(&e, 0x20), 0xff, "lines 32–39 and nothing above");
        poke(&e, 0x28, 0xff); // RTSR2
        e.set_line(35, true);
        assert_eq!(peek(&e, 0x34), 1 << 3, "PR2");
        assert!(e.irq_asserted(35));
        // …and the first bank is untouched by any of it.
        assert_eq!(peek(&e, 0x14), 0);
    }

    #[test]
    fn imr_reserved_bits_read_back_as_ones() {
        // An L4 reserves part of `IMR1` as ones (RM0351 §13.5.1); the machine
        // file says which, because the set differs between parts.
        let e = Exti::with_lines(40, [0x0000_00c0, 0x0000_0003]);
        assert_eq!(peek(&e, 0x00), 0xc0, "the reset value is the ones");
        poke(&e, 0x00, 0);
        assert_eq!(
            peek(&e, 0x00),
            0xc0,
            "and a write of zero does not clear it"
        );
        assert_eq!(peek(&e, 0x20), 3);
        Device::reset(&e, ResetKind::Cold);
        assert_eq!(peek(&e, 0x00), 0xc0);
    }

    #[test]
    fn a_wire_drives_a_line_through_its_sink() {
        let e = exti();
        let src = WireId::new(1);
        let pin = Device::sink(&e, "line4", &[src]).expect("line4");
        assert_eq!(pin.line, 4);
        // A net holds its sinks weakly, so the device has to own this one.
        let weak = Arc::downgrade(&pin.sink);
        drop(pin);
        let alive = weak.upgrade().expect("the controller still owns the pin");
        poke(&e, 0x00, 1 << 4);
        poke(&e, 0x08, 1 << 4);
        alive.set_level(src, 0, Level::High);
        assert!(e.irq_asserted(4));
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        let e = exti();
        poke(&e, 0x00, 1);
        poke(&e, 0x08, 1);
        e.set_line(0, true);
        let mut word = [0u8; 4];
        e.regs
            .read(0x14, &mut word, MemAttrs::DEBUG)
            .expect("reading PR is free");
        assert_eq!(u32::from_le_bytes(word), 1);
        assert!(e.pending(0), "a peek did not clear it");
        // A debug write would drop it, so it is refused.
        assert_eq!(
            e.regs.write(0x14, &1u32.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(e.pending(0));
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let e = exti();
        let mut byte = [0u8; 1];
        assert_eq!(
            e.regs.read(0x14, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            e.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn a_reset_clears_every_pending_bit_and_the_detector_with_them() {
        let e = exti();
        poke(&e, 0x00, 0xffff);
        poke(&e, 0x08, 0xffff);
        poke(&e, 0x0c, 0xffff);
        e.set_line(5, true);
        assert!(e.pending(5));
        Device::reset(&e, ResetKind::Warm);
        assert_eq!(peek(&e, 0x14), 0);
        assert_eq!(peek(&e, 0x00), 0);
        // The detector forgot the level too, so the line coming back down is
        // not an edge it reports.
        poke(&e, 0x0c, 0xffff);
        e.set_line(5, false);
        assert_eq!(peek(&e, 0x14), 0);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = Exti::with_lines(40, [0x0000_00c0, 0]);
        poke(&saved, 0x00, 0x0000_1234);
        poke(&saved, 0x04, 0x0000_00f0);
        poke(&saved, 0x08, 0x0000_ffff);
        poke(&saved, 0x0c, 0x000f_0000);
        poke(&saved, 0x20, 0x0000_0055);
        poke(&saved, 0x28, 0x0000_00ff);
        saved.set_line(1, true);
        saved.set_line(33, true);
        poke(&saved, 0x10, 1 << 4);
        assert!(saved.pending(1) && saved.pending(33) && saved.pending(4));

        let mut shape = MachineShape::new();
        shape.add_device("exti", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("exti", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = Exti::with_lines(40, [0x0000_00c0, 0]);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("exti", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let offsets = [
            0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34,
        ];
        let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
        let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
        assert_eq!(before, after);

        // And the edge detector came across with its memory intact: line 1 is
        // *already high*, so driving it high again is not an edge, while
        // taking it low is.
        poke(&restored, 0x14, 0xffff_ffff);
        poke(&restored, 0x0c, 1 << 1);
        restored.set_line(1, true);
        assert!(
            !restored.pending(1),
            "no phantom edge on the restored level"
        );
        restored.set_line(1, false);
        assert!(restored.pending(1));
    }

    #[test]
    fn a_snapshot_from_a_wider_part_is_refused() {
        let saved = Exti::with_lines(40, [0, 0]);
        let mut shape = MachineShape::new();
        shape.add_device("exti", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("exti", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("exti", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&exti(), &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("lines", Value::from(40u64));
        assert_eq!(Exti::new(&props).unwrap().lines(), 40);
        let props = Props::new().with("line-count", Value::from(40u64));
        assert!(Exti::new(&props).is_err());
        let props = Props::new().with("lines", Value::from(41u64));
        assert!(Exti::new(&props).is_err(), "past the widest part modelled");
    }

    #[test]
    fn the_class_is_registrable_and_constructs_through_the_registry() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);
    }

    #[test]
    fn the_schema_and_the_device_agree_about_pins_and_regions() {
        let e = Exti::with_lines(MAX_LINES, [0, 0]);
        let schema = schema();
        let src = WireId::new(1);
        for n in [0u32, MAX_LINES - 1] {
            assert!(schema.port_named(&format!("line{n}")).is_some());
            assert!(schema.port_named(&format!("irq{n}")).is_some());
            assert!(Device::sink(&e, &format!("line{n}"), &[src]).is_some());
            assert!(Device::connect(&e, &format!("irq{n}"), dummy_source()).is_ok());
        }
        assert!(schema.port_named("event").is_some());
        assert!(schema.port_named(&format!("line{MAX_LINES}")).is_none());
        assert!(Device::region(&e, "").is_some());
        assert!(Device::region(&e, "regs").is_some());
        assert!(Device::region(&e, "lines").is_none());
    }

    /// A wire with one source, so a pin has something to drive.
    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }

    // -- the U5 (RM0456 §22; offsets from `EXTI_TypeDef`) --------------------

    const RTSR1: u64 = 0x00;
    const FTSR1: u64 = 0x04;
    const SWIER1: u64 = 0x08;
    const RPR1: u64 = 0x0c;
    const FPR1: u64 = 0x10;
    const SECCFGR1: u64 = 0x14;
    const PRIVCFGR1: u64 = 0x18;
    const EXTICR1: u64 = 0x60;
    const EXTICR4: u64 = 0x6c;
    const LOCKR: u64 = 0x70;
    const IMR1: u64 = 0x80;
    const EMR1: u64 = 0x84;

    /// A U585: 24 lines, ports A–I.
    fn u5() -> Exti {
        Exti::new(&Props::new().with("variant", Value::from("u5"))).expect("a U5 EXTI")
    }

    #[test]
    fn a_u5_has_24_lines_nine_ports_and_the_rm0456_map() {
        let e = u5();
        assert_eq!(e.variant(), Variant::U5);
        assert_eq!(e.lines(), U5_LINES);
        assert_eq!(e.register_bytes(), 0x88, "through EMR1");
        poke(&e, IMR1, 0xffff_ffff);
        assert_eq!(peek(&e, IMR1), 0x00ff_ffff, "IM0..IM23");
        // `ports` is the U5's, and only the U5's.
        let props = Props::new()
            .with("variant", Value::from("u5"))
            .with("ports", Value::from(11u64));
        assert!(Exti::new(&props).is_ok());
        let props = Props::new().with("ports", Value::from(9u64));
        assert!(Exti::new(&props).is_err(), "the F4's mux is st.syscfg");
        let props = Props::new()
            .with("variant", Value::from("u5"))
            .with("ports", Value::from(12u64));
        assert!(Exti::new(&props).is_err());
    }

    #[test]
    fn a_u5_rising_edge_sets_rpr_and_a_falling_one_fpr() {
        let e = u5();
        let irq = watch(&e, "irq16");
        poke(&e, IMR1, 1 << 16);
        poke(&e, RTSR1, 1 << 16);
        poke(&e, FTSR1, 1 << 16);
        e.set_line(16, true);
        assert_eq!(peek(&e, RPR1), 1 << 16);
        assert_eq!(peek(&e, FPR1), 0);
        assert!(high(&irq));
        poke(&e, RPR1, 1 << 16);
        assert!(!high(&irq), "write one to clear");
        e.set_line(16, false);
        assert_eq!(peek(&e, RPR1), 0);
        assert_eq!(peek(&e, FPR1), 1 << 16);
        assert!(high(&irq), "either pending register raises the line");
        poke(&e, RPR1, 1 << 16);
        assert!(high(&irq), "clearing RPR leaves FPR");
        poke(&e, FPR1, 1 << 16);
        assert!(!high(&irq));
    }

    #[test]
    fn a_u5_swier_is_a_strobe_into_rpr_that_reads_zero() {
        let e = u5();
        let event = watch(&e, "event");
        poke(&e, EMR1, 1 << 3);
        poke(&e, SWIER1, 1 << 3);
        assert_eq!(peek(&e, SWIER1), 0);
        assert_eq!(peek(&e, RPR1), 1 << 3);
        assert_eq!(event.edges.load(Ordering::Relaxed), 1);
        // Not a level: writing it again raises it again.
        poke(&e, SWIER1, 1 << 3);
        assert_eq!(event.edges.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn exticr_routes_a_port_pin_to_its_line_and_re_latches_on_a_change() {
        let e = u5();
        let irq = watch(&e, "irq13");
        poke(&e, IMR1, 1 << 13);
        poke(&e, RTSR1, 1 << 13);
        // Port C pin 13 (the user button on ST's U5 boards) moves, but line
        // 13 is on port A out of reset.
        e.set_port_pin(2, 13, true);
        assert!(!e.pending(13));
        // EXTICR4 field for line 13 is bits 11:8; port C is 2. Selecting a pin
        // already high is a rising edge.
        poke(&e, EXTICR4, 2 << 8);
        assert_eq!(peek(&e, EXTICR4), 2 << 8);
        assert!(e.pending(13));
        assert!(high(&irq));
        poke(&e, RPR1, 1 << 13);
        e.set_port_pin(2, 13, false);
        e.set_port_pin(2, 13, true);
        assert!(e.pending(13), "and now the pin itself reaches the line");
        // A pin of another line does nothing to line 13.
        poke(&e, RPR1, 1 << 13);
        e.set_port_pin(2, 12, true);
        assert!(!e.pending(13));
        // The fields are four bits, eight apart.
        poke(&e, EXTICR1, 0xffff_ffff);
        assert_eq!(peek(&e, EXTICR1), 0x0f0f_0f0f);
    }

    #[test]
    fn a_u5_mux_field_naming_a_missing_port_selects_nothing() {
        let e = u5();
        poke(&e, RTSR1, 1);
        poke(&e, FTSR1, 1);
        e.set_port_pin(0, 0, true);
        assert!(e.pending(0), "PA0 on line 0 out of reset");
        poke(&e, RPR1, 1);
        // Port J (9) is not bonded on a nine-port part: the line falls.
        poke(&e, EXTICR1, 9);
        assert_eq!(peek(&e, FPR1), 1);
    }

    #[test]
    fn a_u5_takes_port_pins_and_peripheral_lines_by_name() {
        let e = u5();
        let src = WireId::new(1);
        let pc13 = Device::sink(&e, "pc13", &[src]).expect("pc13");
        assert_eq!(pc13.line, 13);
        assert!(Device::sink(&e, "pi15", &[src]).is_some());
        assert!(
            Device::sink(&e, "pj0", &[src]).is_none(),
            "port J is not bonded"
        );
        assert!(Device::sink(&e, "line16", &[src]).is_some(), "PVD");
        assert!(Device::sink(&e, "line23", &[src]).is_some());
        assert!(Device::sink(&e, "line24", &[src]).is_none());
        assert!(
            Device::sink(&e, "line5", &[src]).is_none(),
            "the mux drives lines 0-15"
        );
        assert!(Device::connect(&e, "irq15", dummy_source()).is_ok());
        assert!(Device::connect(&e, "irq23", dummy_source()).is_ok());
        // And the wire reaches the detector through the mux.
        poke(&e, EXTICR4, 2 << 8);
        poke(&e, RTSR1, 1 << 13);
        pc13.sink.set_level(src, 0, Level::High);
        assert!(e.pending(13));
        // The classic map has no port pins.
        assert!(Device::sink(&exti(), "pa0", &[src]).is_none());
        let schema = schema();
        assert!(schema.port_named("pa0").is_some());
        assert!(schema.port_named("pk15").is_some());
    }

    #[test]
    fn a_u5_lock_freezes_privcfgr_and_seccfgr_is_raz_wi() {
        let e = u5();
        poke(&e, SECCFGR1, 0xffff_ffff);
        assert_eq!(peek(&e, SECCFGR1), 0, "TZEN = 0");
        poke(&e, PRIVCFGR1, 0x0000_00ff);
        assert_eq!(peek(&e, PRIVCFGR1), 0xff);
        poke(&e, LOCKR, 1);
        assert_eq!(peek(&e, LOCKR), 1);
        poke(&e, PRIVCFGR1, 0);
        assert_eq!(peek(&e, PRIVCFGR1), 0xff, "locked");
        poke(&e, LOCKR, 0);
        assert_eq!(peek(&e, LOCKR), 1, "set once");
        Device::reset(&e, ResetKind::Warm);
        assert_eq!(peek(&e, LOCKR), 0, "until reset");
        assert_eq!(peek(&e, PRIVCFGR1), 0);
    }

    #[test]
    fn a_u5_reset_keeps_the_port_levels_and_latches_them_without_an_edge() {
        let e = u5();
        e.set_port_pin(0, 4, true);
        Device::reset(&e, ResetKind::Warm);
        // PA4 is still high and line 4 is on port A: no phantom edge, and a
        // falling edge is still seen.
        poke(&e, RTSR1, 1 << 4);
        poke(&e, FTSR1, 1 << 4);
        e.set_port_pin(0, 4, true);
        assert!(!e.pending(4));
        e.set_port_pin(0, 4, false);
        assert_eq!(peek(&e, FPR1), 1 << 4);
    }

    #[test]
    fn a_u5_debug_read_moves_nothing_and_a_debug_write_is_refused() {
        let e = u5();
        poke(&e, RTSR1, 1);
        e.set_port_pin(0, 0, true);
        let mut word = [0u8; 4];
        e.regs.read(RPR1, &mut word, MemAttrs::DEBUG).unwrap();
        assert_eq!(u32::from_le_bytes(word), 1);
        assert!(e.pending(0));
        assert_eq!(
            e.regs.write(RPR1, &1u32.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(e.pending(0));
    }

    #[test]
    fn a_u5_snapshot_round_trips_to_identical_state() {
        let saved = u5();
        poke(&saved, IMR1, 0x00f0_1234);
        poke(&saved, EMR1, 0x0000_00f0);
        poke(&saved, RTSR1, 0x0000_ffff);
        poke(&saved, FTSR1, 0x00ff_0000);
        poke(&saved, EXTICR4, 0x0002_0200);
        poke(&saved, PRIVCFGR1, 0x55);
        poke(&saved, LOCKR, 1);
        saved.set_port_pin(2, 13, true);
        saved.set_port_pin(8, 0, true);
        saved.set_line(17, true);
        saved.set_line(17, false);
        assert!(saved.pending(13) && saved.pending(17));

        let save = |e: &Exti| {
            let mut shape = MachineShape::new();
            shape.add_device("exti", CLASS_NAME).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("exti", CLASS_NAME, STATE_VERSION).unwrap();
                Device::save(e, &mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };
        let load = |e: &Exti, bytes: &[u8]| {
            let reader = StateReader::new(bytes).unwrap();
            let chunk = reader
                .load("exti", CLASS_NAME, STATE_VERSION, &Migrations::new())
                .unwrap();
            Device::load(e, &mut chunk.reader())
        };
        let bytes = save(&saved);
        let restored = u5();
        load(&restored, &bytes).unwrap();
        assert_eq!(save(&restored), bytes, "identical state");
        for offset in [
            RTSR1, FTSR1, RPR1, FPR1, PRIVCFGR1, EXTICR1, EXTICR4, LOCKR, IMR1, EMR1,
        ] {
            assert_eq!(peek(&saved, offset), peek(&restored, offset), "{offset:#x}");
        }
        // The port levels came across: PC13 is already high, so driving it
        // high again is no edge.
        poke(&restored, RPR1, 0xffff_ffff);
        restored.set_port_pin(2, 13, true);
        assert!(!restored.pending(13));

        // And neither map loads the other's chunk.
        assert!(load(&exti(), &bytes).is_err());
        let classic = Exti::with_lines(U5_LINES, [0, 0]);
        assert!(load(&u5(), &save(&classic)).is_err());
    }
}
