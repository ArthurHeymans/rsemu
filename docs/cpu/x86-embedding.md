# Bounded firmware embedding

`cpu::x86::embedding` is an interpreter-only interface for a caller that owns
its memory/I/O policy and services firmware calls itself. It does not construct
an emulated PC or authorize hardware passthrough.

1. Construct an `X86`, attach memory and (separately) an I/O `AddressSpace`,
   execute its reset sequence, and install the desired registers/system state.
   `set_regs` alone does not construct protected-mode descriptor caches.
2. Call `step_embedded(StepOptions { repeat_limit: 256,
   intercept_system: true })`. Service its exit only after it returns. Memory
   callbacks still execute under the CPU lock and must never re-enter the CPU.
3. `Continue` completes a step. `RepeatYield` is a resumable boundary within a
   REP: the count and index registers include completed iterations, while IP
   points back at the instruction's prefix. Poll a deadline/cancellation token
   here, not only at ordinary instruction completion.
4. `SoftwareInterrupt(vector)` leaves IP after the INT, with no frame pushed.
   A host BIOS service can update registers directly. To run the guest handler
   instead, call `enter_software_interrupt(vector)`, which performs the normal
   interrupt-table and privilege checks and pushes the architectural frame.
5. With system interception enabled, CPUID, RDMSR/WRMSR, RDTSC and INVD/WBINVD
   exit before modifying their result registers or doing their modeled action.
   IP already names the following instruction. Validate the host policy, supply
   the result, or stop. Disabling interception uses the ordinary virtual CPU's
   semantics; it never falls back to executing a privileged host instruction.
6. `Exception` reports an architectural fault/trap before guest delivery.
   `BusFault` reports an actual address-space refusal and suppresses subsequent
   bus transactions in that step. **Stop on BusFault**: earlier device effects
   cannot be rolled back, and intermediate CPU state is not a retry contract.
   Configure the address spaces to refuse disallowed operations; an explicitly
   modeled open bus is still a successful access, not an error.
7. `Halted` is a stop condition, not a successful firmware return. An embedder
   must validate its own return sentinel and distinguish it from a stray HLT.

The existing `step`/machine execution path retains guest exception delivery and
unintercepted virtual system operations. The embedded path flushes cached JIT
translations so switching back to a translating engine cannot reuse code that
an embedded step modified. REP chunking can add re-decode/bus cycles relative
to a single unbounded step; this API is not a cycle-identical scheduling mode.

Firmware identification is policy. Advertising host CPUID features unsupported
by the interpreter changes the guest's execution paths and may lead it to
unsupported instructions. Likewise, modeled cache invalidation and virtual
MSRs are not evidence that the real board received a cache operation or an MSR
write. Keep these decisions explicit in the embedding layer.

## Extending the ISA

Add encoding information in `src/cpu/x86/isa.rs`, instruction semantics in
`exec.rs`, `prot.rs` or `fpexec.rs`, and regression vectors in `tests.rs` or a
focused integration test. Do not hide missing instructions by inventing CPUID
features or silently treating opcodes as NOPs. Decode and execution remain the
same implementation used by the normal interpreter.

The embedding regressions live in `tests/x86_embedding.rs`. They include the
firmware-relevant LDMXCSR/STMXCSR sequence with CR0.MP clear, explicit error
exits, REP progress/cancellation boundaries, and byte/word/dword port cycles.
Run them without default features as well as the existing x86 unit suite:

```sh
cargo test --no-default-features --features cpu-x86 --test x86_embedding
cargo test --no-default-features --features cpu-x86 --lib cpu::x86::
cargo check --no-default-features --features cpu-x86 --lib \
    --target x86_64-unknown-none
```
