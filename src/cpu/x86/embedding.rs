//! Bounded interpreter stepping for embedders that service firmware calls.
//!
//! Unlike [`X86::step`], this interface returns exceptions to the caller before
//! delivering a guest handler. Software interrupts can be handled by the host
//! or entered with [`X86::enter_software_interrupt`]. Exit handling happens after
//! the CPU execution lock is released. Memory/I/O callbacks still run inside a
//! step and must not re-enter the CPU. This is interpreter-only; it never runs a JIT.

use super::exec::{Exec, Fault, VEC_BREAKPOINT, VEC_OVERFLOW, VEC_UD};
use super::isa::{Fields, Op};
use super::{X86, flags};

/// An event observed at an embedded interpreter boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exit {
    /// An ordinary instruction completed.
    Continue,
    /// REP has more iterations; registers describe the completed iterations
    /// and IP points back at the prefix. Calling step again resumes it.
    RepeatYield,
    /// INT/INT3/INTO completed decode, but has not pushed an interrupt frame.
    /// IP already names the following instruction.
    SoftwareInterrupt(u8),
    /// CPUID intercepted before modifying EAX/EBX/ECX/EDX.
    Cpuid,
    /// RDMSR intercepted; ECX names the register.
    ReadMsr,
    /// WRMSR intercepted; ECX and EDX:EAX are its operands.
    WriteMsr,
    /// RDTSC intercepted before modifying EDX:EAX.
    ReadTsc,
    /// A cache invalidation operation intercepted after its privilege check.
    CacheInvalidate {
        /// True for WBINVD, false for INVD.
        write_back: bool,
    },
    /// An architectural exception. Restartable faults leave CS:IP at the
    /// instruction, with no guest exception frame written by this interface.
    Exception {
        /// Architectural exception vector.
        vector: u8,
        /// Architectural error code, if this vector supplies one.
        error_code: Option<u32>,
    },
    /// An address-space operation was refused. This overrides any other exit.
    /// Further bus transactions in this step are suppressed. Earlier effects
    /// are not rolled back; the caller must stop.
    BusFault {
        /// Last refused physical address.
        address: u64,
    },
    /// HLT, shutdown, wait-for-SIPI, or no executable step.
    Halted,
}

/// Policy for one bounded interpreter step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepOptions {
    /// Maximum completed REP iterations per call (zero is treated as one).
    pub repeat_limit: u32,
    /// Exit on CPUID, RDMSR/WRMSR, RDTSC and INVD/WBINVD instead of using
    /// virtual-machine semantics. The host must supply the result or stop.
    /// This never enables passthrough to the host CPU by itself.
    pub intercept_system: bool,
}

impl Default for StepOptions {
    fn default() -> Self {
        Self {
            repeat_limit: 256,
            intercept_system: false,
        }
    }
}

/// A bounded interpreter result. All accesses made by this step have completed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepResult {
    /// Emulated clocks charged by the step.
    pub cycles: u64,
    /// Why execution returned to the embedder.
    pub exit: Exit,
}

impl X86 {
    /// Interpret one instruction, event, or bounded chunk of a REP instruction.
    ///
    /// Exceptions and software interrupts are returned rather than delivered.
    /// System intercepts leave IP after the intercepted instruction: update
    /// registers with the host result before the next step. A BusFault is
    /// terminal for this invocation; it does not undo earlier device effects.
    pub fn step_embedded(&self, options: StepOptions) -> StepResult {
        self.lines.poll_intc();
        let reset = self.lines.take_reset_request();
        let cfg = self.config();
        let mut session = self.session.lock();
        session.state.reset_pending |= reset;
        let Some(memory) = session.memory.clone() else {
            return StepResult {
                cycles: 0,
                exit: Exit::Halted,
            };
        };
        let io = session.io.clone();
        let faults = session.state.faults;
        let super::Session {
            state, position, ..
        } = &mut *session;
        let mut exec = Exec::new(state, &memory, io.as_deref(), &cfg, &self.lines)
            .with_position(position.as_ref());
        exec.embedded = Some(options);
        let cycles = exec.step();
        let mut exit = exec.exit.unwrap_or(if exec.state.halted || cycles == 0 {
            Exit::Halted
        } else {
            Exit::Continue
        });
        if exec.state.faults != faults {
            exit = Exit::BusFault {
                address: exec.state.last_fault,
            };
        }
        // Embedded execution changes memory without the translating engine's
        // write ledger. Invalidate cached translations before any engine switch.
        #[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
        if let Some(jit) = session.jit.as_mut() {
            jit.flush();
        }
        if let Some(position) = session.position.as_mut() {
            position.origin = position.origin.wrapping_add(cycles);
        }
        StepResult { cycles, exit }
    }

    /// Enter a software interrupt using the current (already advanced) CS:IP.
    ///
    /// This performs the interpreter's ordinary descriptor/privilege checks.
    /// On failure, return the exception without attempting guest delivery.
    /// This also wakes a core halted at an embedder's return sentinel.
    pub fn enter_software_interrupt(&self, vector: u8) -> StepResult {
        let cfg = self.config();
        let mut session = self.session.lock();
        let Some(memory) = session.memory.clone() else {
            return StepResult {
                cycles: 0,
                exit: Exit::Halted,
            };
        };
        let io = session.io.clone();
        let faults = session.state.faults;
        session.state.halted = false;
        let super::Session {
            state, position, ..
        } = &mut *session;
        let mut exec = Exec::new(state, &memory, io.as_deref(), &cfg, &self.lines)
            .with_position(position.as_ref());
        exec.embedded = Some(StepOptions::default());
        let result = exec.software_interrupt(vector);
        let cycles = exec.used;
        let exit = if exec.state.faults != faults {
            Exit::BusFault {
                address: exec.state.last_fault,
            }
        } else {
            result.map_or_else(exception_exit, |()| Exit::Continue)
        };
        #[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
        if let Some(jit) = session.jit.as_mut() {
            jit.flush();
        }
        if let Some(position) = session.position.as_mut() {
            position.origin = position.origin.wrapping_add(cycles);
        }
        StepResult { cycles, exit }
    }
}

pub(super) fn exception_exit(fault: Fault) -> Exit {
    Exit::Exception {
        vector: fault.vector,
        error_code: fault.error,
    }
}

impl Exec<'_> {
    /// Intercepts run after decode but before operand or stack side effects.
    pub(super) fn embedded_intercept(&mut self, f: &Fields) -> Result<bool, Fault> {
        let Some(options) = self.embedded else {
            return Ok(false);
        };
        let exit = match f.insn.op {
            Op::INT => {
                if self.v86() && self.state.regs.iopl() != 3 {
                    return Err(Fault::gp(0));
                }
                Exit::SoftwareInterrupt(f.imm as u8)
            }
            Op::INT3 => Exit::SoftwareInterrupt(VEC_BREAKPOINT),
            Op::INTO if self.flag(flags::OF) => Exit::SoftwareInterrupt(VEC_OVERFLOW),
            Op::CPUID if options.intercept_system => {
                if !self.cfg.features.extras_486 {
                    return Err(Fault::bare(VEC_UD));
                }
                Exit::Cpuid
            }
            Op::RDMSR | Op::WRMSR | Op::RDTSC if options.intercept_system => {
                if !self.cfg.features.msr {
                    return Err(Fault::bare(VEC_UD));
                }
                if f.insn.op != Op::RDTSC || self.state.sys.cr4 & super::prot::cr4::TSD != 0 {
                    self.require_ring0()?;
                }
                match f.insn.op {
                    Op::RDMSR => Exit::ReadMsr,
                    Op::WRMSR => Exit::WriteMsr,
                    _ => Exit::ReadTsc,
                }
            }
            Op::INVD | Op::WBINVD if options.intercept_system => {
                self.require_ring0()?;
                Exit::CacheInvalidate {
                    write_back: f.insn.op == Op::WBINVD,
                }
            }
            _ => return Ok(false),
        };
        self.exit = Some(exit);
        Ok(true)
    }
}
