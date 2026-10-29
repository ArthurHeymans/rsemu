//! The `stm32u575` and `stm32u585` boards, end to end, with no corpus.
//!
//! `tests/stm32u5_hal.rs` runs ST's own HAL on these boards, but only where
//! someone has fetched and built it. This is what `cargo test` always proves:
//!
//! * the core boots through `VTOR` at 0x08000000 — a U5 has **nothing** at
//!   address zero, unlike every F4 board, which aliases its boot memory there;
//! * USART1 is **clock-gated** by RCC: the same program with its
//!   `RCC_APB2ENR.USART1EN` write removed prints nothing, because the board
//!   wires `rcc.apb2en14 -> usart1.enable` and a U5's peripheral clocks come
//!   out of reset off;
//! * the factory signature is where the HAL looks for it, and the U585 has a
//!   HASH unit the U575 does not.
//!
//! The program is ten instructions, assembled by GNU as 2.45 for
//! `-mcpu=cortex-m33` from the source written out beside the bytes, so a
//! reader can check each halfword against its line.

#![cfg(all(feature = "machine-stm32u575", feature = "machine-stm32u585"))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::chardev::{CharPort, ports};
use rsemu::machine::{Machine, catalog};

/// Where the flash array is, and so where `VTOR` points out of reset.
const FLASH: usize = 0x0800_0000;
/// The initial stack pointer: the top of SRAM1.
const STACK: u32 = 0x2003_0000;
/// The program's entry point, an offset into flash.
const ENTRY: usize = 0x100;

/// The program, from `ENTRY`. `GNU as` output, source alongside:
///
/// ```text
/// 100: 4c0b       ldr   r4, =0x46020c00   @ RCC
/// 102: 2101       movs  r1, #1
/// 104: 0389       lsls  r1, r1, #14       @ APB2ENR.USART1EN
/// 106: f8c4 10a4  str.w r1, [r4, #0xa4]
/// 10a: 4d0a       ldr   r5, =0x40013800   @ USART1
/// 10c: 2109       movs  r1, #9            @ CR1: UE | TE
/// 10e: 6029       str   r1, [r5, #0]
/// 110: a205       adr   r2, msg
/// 112: 7813  next: ldrb r3, [r2]
/// 114: 2b00       cmp   r3, #0
/// 116: d006       beq   done
/// 118: 69ee  wait: ldr  r6, [r5, #0x1c]   @ ISR
/// 11a: 2780       movs  r7, #0x80         @ TXE
/// 11c: 423e       tst   r6, r7
/// 11e: d0fb       beq   wait
/// 120: 62ab       str   r3, [r5, #0x28]   @ TDR
/// 122: 3201       adds  r2, #1
/// 124: e7f5       b     next
/// 126: e7fe  done: b    done
/// 128: "U5!\n\0"  msg, then the literal pool at 0x130
/// ```
const PROGRAM: [u8; 0x38] = [
    0x0b, 0x4c, 0x01, 0x21, 0x89, 0x03, 0xc4, 0xf8, 0xa4, 0x10, 0x0a, 0x4d, 0x09, 0x21, 0x29, 0x60,
    0x05, 0xa2, 0x13, 0x78, 0x00, 0x2b, 0x06, 0xd0, 0xee, 0x69, 0x80, 0x27, 0x3e, 0x42, 0xfb, 0xd0,
    0xab, 0x62, 0x01, 0x32, 0xf5, 0xe7, 0xfe, 0xe7, 0x55, 0x35, 0x21, 0x0a, 0x00, 0x00, 0x00, 0xbf,
    0x00, 0x0c, 0x02, 0x46, 0x00, 0x38, 0x01, 0x40,
];

/// Where `str.w r1, [r4, #0xa4]` — the clock enable — sits in [`PROGRAM`].
const CLOCK_ENABLE: usize = 0x06;

/// The flash image: a two-word vector table and the program.
fn image(enable_clock: bool) -> Vec<u8> {
    let mut image = vec![0u8; ENTRY + PROGRAM.len()];
    image[0..4].copy_from_slice(&STACK.to_le_bytes());
    image[4..8].copy_from_slice(&((FLASH + ENTRY) as u32 | 1).to_le_bytes());
    image[ENTRY..].copy_from_slice(&PROGRAM);
    if !enable_clock {
        // Two NOPs where the RCC write was.
        image[ENTRY + CLOCK_ENABLE..ENTRY + CLOCK_ENABLE + 4]
            .copy_from_slice(&[0x00, 0xbf, 0x00, 0xbf]);
    }
    image
}

/// Build `board` out of the catalog with `firmware` in flash.
fn boot(board: &str, firmware: Vec<u8>) -> (Machine, Arc<CharPort>) {
    let entry = catalog::machine(board).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{board} does not realize: {e}"));
    let port = ports::open(&options.realize.hosts, "console").expect("USART1 opened it");
    (machine, port)
}

/// Run for `ms` emulated milliseconds and return what reached the console.
fn run(m: &mut Machine, port: &CharPort, ms: u64) -> String {
    m.run_for(GlobalTime::from_nanos(ms * 1_000_000))
        .expect("it runs");
    String::from_utf8_lossy(&port.drain()).into_owned()
}

/// One word of the core's address space, without side effects; `None` where
/// nothing answers.
fn peek(m: &Machine, addr: u64, width: Width) -> Option<u64> {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEBUG)
        .ok()
}

#[test]
fn both_boards_boot_through_vtor_and_print_once_usart1_has_its_clock() {
    for board in ["stm32u575", "stm32u585"] {
        let (mut m, port) = boot(board, image(true));
        assert_eq!(peek(&m, FLASH as u64, Width::U32), Some(u64::from(STACK)));
        assert_eq!(
            peek(&m, 0, Width::U32),
            None,
            "{board}: nothing is mapped at zero on a U5"
        );
        // The USART moves at most one character per scheduler round, as on
        // the stm32f407 board, so four characters want a few rounds: 20 ms,
        // the same allowance `tests/stm32f407_board.rs` gives.
        assert_eq!(run(&mut m, &port, 20), "U5!\n", "{board}");
    }
}

#[test]
fn a_usart_whose_clock_was_never_enabled_says_nothing() {
    let (mut m, port) = boot("stm32u575", image(false));
    assert_eq!(
        run(&mut m, &port, 20),
        "",
        "USART1 is gated off by RCC out of reset, as on the part"
    );
}

#[test]
fn the_factory_signature_and_the_hash_unit_are_where_the_hal_looks() {
    let (m, _port) = boot("stm32u575", image(true));
    // `FLASH_SIZE` in the HAL: a halfword at FLASHSIZE_BASE, in KiB.
    assert_eq!(peek(&m, 0x0bfa_07a0, Width::U16), Some(2048));
    // The 96-bit UID is drawn from the board seed, so it is not all zeros.
    let uid: Vec<u64> = (0..3)
        .map(|k| peek(&m, 0x0bfa_0700 + 4 * k, Width::U32).expect("UID word"))
        .collect();
    assert_ne!(uid, vec![0, 0, 0]);
    assert_eq!(
        peek(&m, 0x420c_0400, Width::U32),
        None,
        "a U575 has no HASH unit"
    );

    let (m, _port) = boot("stm32u585", image(true));
    assert!(
        peek(&m, 0x420c_0400, Width::U32).is_some(),
        "a U585's HASH_CR answers"
    );
}
