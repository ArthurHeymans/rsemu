//! Firmware embedding must not hide faults or run unbounded REP instructions.
#![cfg(feature = "cpu-x86")]

use rsemu::core::space::{AddressSpace, RamStore, Region};
use rsemu::cpu::x86::{
    Config, Regs, X86,
    embedding::{Exit, StepOptions},
};
use std::sync::Arc;

fn machine(code: &[u8]) -> (X86, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(0x100000));
    for (i, &byte) in code.iter().enumerate() {
        ram.write_u8(0x100 + i as u64, byte).unwrap();
    }
    let memory = Arc::new(AddressSpace::new("memory", 32));
    memory
        .topology()
        .map(Region::ram("ram", ram.clone()), 0)
        .unwrap();
    let cpu = X86::new(Config::X86_64);
    cpu.attach_space(memory);
    cpu.step(); // reset, without a fetch
    let mut regs = Regs::new();
    regs.cs = 0;
    regs.rip = 0x100;
    regs.ss = 0;
    regs.rsp = 0x8000;
    cpu.set_regs(regs);
    (cpu, ram)
}

#[test]
fn software_interrupt_exits_before_stack_effects_and_can_enter_a_guest_iret() {
    let (cpu, ram) = machine(&[0xcd, 0x10, 0xf4]);
    ram.write_u8(0x40, 0x00).unwrap();
    ram.write_u8(0x41, 0x02).unwrap();
    ram.write_u8(0x200, 0xcf).unwrap();
    let result = cpu.step_embedded(StepOptions::default());
    assert_eq!(result.exit, Exit::SoftwareInterrupt(0x10));
    assert_eq!((cpu.regs().rip, cpu.regs().rsp), (0x102, 0x8000));
    assert_eq!(cpu.enter_software_interrupt(0x10).exit, Exit::Continue);
    assert_eq!((cpu.regs().rip, cpu.regs().rsp), (0x200, 0x7ffa));
    assert_eq!(
        cpu.step_embedded(StepOptions::default()).exit,
        Exit::Continue
    );
    assert_eq!((cpu.regs().rip, cpu.regs().rsp), (0x102, 0x8000));
    assert_eq!(cpu.step_embedded(StepOptions::default()).exit, Exit::Halted);
}

#[test]
fn invalid_opcode_exits_without_delivering_an_exception_frame() {
    let (cpu, _) = machine(&[0x0f, 0x0b]); // UD2
    let result = cpu.step_embedded(StepOptions::default());
    assert_eq!(
        result.exit,
        Exit::Exception {
            vector: 6,
            error_code: None
        }
    );
    assert_eq!((cpu.regs().rip, cpu.regs().rsp), (0x100, 0x8000));
}

#[test]
fn address_size_32_rep_yields_and_resumes_with_progress() {
    let (cpu, ram) = machine(&[0x67, 0xf3, 0xaa]); // rep stosb, ECX/EDI
    let mut regs = cpu.regs();
    regs.rcx = 100_000;
    regs.rdi = 0x1000;
    regs.rax = 0x5a;
    cpu.set_regs(regs);
    let options = StepOptions {
        repeat_limit: 7,
        ..Default::default()
    };
    assert_eq!(cpu.step_embedded(options).exit, Exit::RepeatYield);
    assert_eq!(
        (cpu.regs().rip, cpu.regs().rcx, cpu.regs().rdi),
        (0x100, 99_993, 0x1007)
    );
    assert_eq!(ram.read_u8(0x1006).unwrap(), 0x5a);
    assert_eq!(ram.read_u8(0x1007).unwrap(), 0);
    assert_eq!(cpu.step_embedded(options).exit, Exit::RepeatYield);
    assert_eq!((cpu.regs().rcx, cpu.regs().rdi), (99_986, 0x100e));
    let mut regs = cpu.regs();
    regs.rcx = 2;
    cpu.set_regs(regs);
    assert_eq!(cpu.step_embedded(options).exit, Exit::Continue);
    assert_eq!((cpu.regs().rcx, cpu.regs().rip), (0, 0x103));
}

#[test]
fn system_intercepts_leave_operands_and_advance_ip() {
    for (code, exit) in [
        ([0x0f, 0xa2], Exit::Cpuid),
        ([0x0f, 0x32], Exit::ReadMsr),
        ([0x0f, 0x30], Exit::WriteMsr),
        ([0x0f, 0x31], Exit::ReadTsc),
        ([0x0f, 0x09], Exit::CacheInvalidate { write_back: true }),
    ] {
        let (cpu, _) = machine(&code);
        let mut regs = cpu.regs();
        regs.rax = 0x1234;
        regs.rcx = 0xc001_0010;
        regs.rdx = 0x5678;
        cpu.set_regs(regs);
        assert_eq!(
            cpu.step_embedded(StepOptions {
                intercept_system: true,
                ..Default::default()
            })
            .exit,
            exit
        );
        regs.rip += 2;
        assert_eq!(cpu.regs(), regs);
    }
}

#[test]
fn port_cycles_keep_byte_word_and_dword_widths() {
    use rsemu::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult};
    use std::sync::Mutex;
    #[derive(Debug, Default)]
    struct Ports(Mutex<Vec<(u64, Vec<u8>)>>);
    impl MemOps for Ports {
        fn read(&self, offset: u64, dst: &mut [u8], _: MemAttrs) -> MemResult {
            dst.copy_from_slice(&0x12345678u32.to_le_bytes()[..dst.len()]);
            self.0.lock().unwrap().push((offset, dst.to_vec()));
            Ok(())
        }
        fn write(&self, offset: u64, src: &[u8], _: MemAttrs) -> MemResult {
            self.0.lock().unwrap().push((offset, src.to_vec()));
            Ok(())
        }
        fn constraints(&self) -> AccessConstraints {
            AccessConstraints::ANY
        }
    }
    let (cpu, _) = machine(&[0xec, 0xed, 0x66, 0xed, 0xee, 0xef, 0x66, 0xef]);
    let ports = Arc::new(Ports::default());
    let io = Arc::new(AddressSpace::new("io", 16));
    io.topology()
        .map(Region::io("ports", 0x10000, ports.clone()), 0)
        .unwrap();
    cpu.attach_io_space(io);
    let mut regs = cpu.regs();
    regs.rdx = 0xe000;
    cpu.set_regs(regs);
    for _ in 0..6 {
        assert_eq!(
            cpu.step_embedded(StepOptions::default()).exit,
            Exit::Continue
        );
    }
    let cycles = ports.0.lock().unwrap();
    assert_eq!(
        cycles
            .iter()
            .map(|(port, bytes)| (*port, bytes.len()))
            .collect::<Vec<_>>(),
        [
            (0xe000, 1),
            (0xe000, 2),
            (0xe000, 4),
            (0xe000, 1),
            (0xe000, 2),
            (0xe000, 4)
        ]
    );
    assert_eq!(cycles[5].1, [0x78, 0x56, 0x34, 0x12]);
}

#[test]
fn mxcsr_load_store_work_without_cr0_mp() {
    use rsemu::cpu::x86::prot::{cr0, cr4};
    let (cpu, ram) = machine(&[
        0x0f, 0xae, 0x16, 0x00, 0x20, // ldmxcsr [2000]
        0x0f, 0xae, 0x1e, 0x04, 0x20, // stmxcsr [2004]
    ]);
    let mut sys = cpu.sys();
    sys.cr0 &= !(cr0::MP | cr0::EM | cr0::TS);
    sys.cr4 |= cr4::OSFXSR;
    cpu.set_sys(sys);
    for (i, byte) in 0x1f80u32.to_le_bytes().into_iter().enumerate() {
        ram.write_u8(0x2000 + i as u64, byte).unwrap();
    }
    assert_eq!(
        cpu.step_embedded(StepOptions::default()).exit,
        Exit::Continue
    );
    assert_eq!(
        cpu.step_embedded(StepOptions::default()).exit,
        Exit::Continue
    );
    for (i, byte) in 0x1f80u32.to_le_bytes().into_iter().enumerate() {
        assert_eq!(ram.read_u8(0x2004 + i as u64).unwrap(), byte);
    }
}

#[test]
fn refused_memory_is_not_an_ordinary_completed_instruction() {
    use rsemu::core::{
        error::BusError,
        space::{MemAttrs, MemOps, MemResult},
    };
    #[derive(Debug)]
    struct Refused;
    impl MemOps for Refused {
        fn read(&self, _: u64, _: &mut [u8], _: MemAttrs) -> MemResult {
            Err(BusError::BadAccess)
        }
        fn write(&self, _: u64, _: &[u8], _: MemAttrs) -> MemResult {
            Err(BusError::BadAccess)
        }
    }
    let (cpu, _) = machine(&[0xa1, 0x00, 0x20]);
    cpu.space()
        .unwrap()
        .topology()
        .map(Region::io("refused", 0x1000, Arc::new(Refused)), 0x2000)
        .unwrap();
    assert!(matches!(
        cpu.step_embedded(StepOptions::default()).exit,
        Exit::BusFault { .. }
    ));
    // A refused MOVSB source must not fabricate a value and write it to the
    // destination before reporting the error. REP must stop at that refusal.
    let (cpu, ram) = machine(&[0xf3, 0xa4]);
    cpu.space()
        .unwrap()
        .topology()
        .map(Region::io("refused", 0x1000, Arc::new(Refused)), 0x2000)
        .unwrap();
    ram.write_u8(0x3000, 0x5a).unwrap();
    let mut regs = cpu.regs();
    regs.rsi = 0x2000;
    regs.rdi = 0x3000;
    regs.rcx = 0xffff;
    cpu.set_regs(regs);
    assert!(matches!(
        cpu.step_embedded(StepOptions::default()).exit,
        Exit::BusFault { .. }
    ));
    assert_eq!(ram.read_u8(0x3000).unwrap(), 0x5a);
}
