//! ST's own STM32U5 HAL, running on the `stm32u575` and `stm32u585` boards.
//!
//! `tests/stm32u5_board.rs` proves the boards with programs written here. This
//! proves them with **vendor code**: the firmware is ST's HAL and CMSIS startup
//! (BSD-3-Clause and Apache-2.0) linked with a small app of ours
//! (`tests/fixtures/stm32u5-hal/main.c`), and every wait loop in it — PWR's
//! `VOSRDY`, RCC's `PLL1RDY` and `SWS`, FLASH's `BSY` and `WDW`, ICACHE's
//! `BSYENDF`, the RNG's `CONDRST` handshake, the USART's `TEACK` — is a
//! question a model has to answer the way the silicon does. One that answers
//! wrong shows up here as a `FAIL <step> status 3` (`HAL_TIMEOUT`) line rather
//! than as a hang, because the app checks every status.
//!
//! The firmware is not in the repository. `scripts/fetch-testdata.sh
//! stm32u5-hal` fetches the vendor sources at pinned commits and builds it with
//! `arm-none-eabi-gcc`; point `RSEMU_STM32U5_HAL_DIR` at the result. Without the
//! variable every test here passes vacuously, as the other corpus-backed tests
//! do (CLAUDE.md, Testing).

#![cfg(all(feature = "machine-stm32u575", feature = "machine-stm32u585"))]

use std::path::PathBuf;

use rsemu::core::clock::GlobalTime;
use rsemu::host::chardev::ports;
use rsemu::machine::catalog;

/// Where `fetch-testdata.sh stm32u5-hal` put the images, if anyone ran it.
fn corpus() -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("RSEMU_STM32U5_HAL_DIR")?);
    assert!(
        dir.is_dir(),
        "RSEMU_STM32U5_HAL_DIR={} is not a directory",
        dir.display()
    );
    Some(dir)
}

/// Boot `board` with `image` in flash and run it until the app says `PASS`,
/// reports a failure, or a generous emulated-time budget runs out. Returns the
/// console transcript.
fn run(board: &str, image: Vec<u8>) -> String {
    let entry = catalog::machine(board).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{board} does not realize: {e}"));
    let port = ports::open(&options.realize.hosts, "console").expect("USART1 opened it");

    let mut out = Vec::new();
    // Two emulated seconds at most, in 20 ms slices. The app needs about 50 ms:
    // the HAL's 10 ms delay and TIM6's ten 1 ms periods are the long parts.
    for _ in 0..100 {
        m.run_for(GlobalTime::from_nanos(20_000_000))
            .expect("it runs");
        port.drain_into(&mut out);
        let text = String::from_utf8_lossy(&out);
        if text.contains("PASS") || text.contains("FAIL") || text.contains("FAULT") {
            break;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// The value the app printed for `name: 0x........`.
fn value(transcript: &str, name: &str) -> u32 {
    let prefix = format!("{name}: 0x");
    let line = transcript
        .lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .unwrap_or_else(|| panic!("no `{name}` line in:\n{transcript}"));
    u32::from_str_radix(line.trim(), 16).unwrap_or_else(|_| panic!("`{name}` is not hex: {line}"))
}

/// Everything both parts must print.
fn check_common(transcript: &str) {
    assert!(
        transcript.contains("PASS"),
        "the app did not finish:\n{transcript}"
    );
    // A Cortex-M33: implementer Arm, architecture 0xF, part number 0xD21.
    let cpuid = value(transcript, "cpuid");
    assert_eq!(cpuid & 0xff0f_fff0, 0x410f_d210, "CPUID {cpuid:#010x}");
    // MSIS 4 MHz -> PLL1 (M=1, N=80, R=2) -> 160 MHz, as RCC computed it.
    assert_eq!(value(transcript, "sysclk"), 160_000_000);
    assert_eq!(value(transcript, "hclk"), 160_000_000);
    assert_eq!(value(transcript, "icache-cr") & 1, 1, "ICACHE_CR.EN");
    assert_eq!(value(transcript, "delay-ok"), 1, "HAL_Delay on SysTick");
    assert_eq!(value(transcript, "led"), 1, "PC7 driven high");
    assert_eq!(value(transcript, "exti13"), 1, "EXTI13 software trigger");
    assert_eq!(
        value(transcript, "tim6-ok"),
        1,
        "ten TIM6 update interrupts"
    );
    // CRC-32/MPEG-2 check value: the CRC unit's reset configuration.
    assert_eq!(value(transcript, "crc"), 0x0376_e6e7);
    assert_eq!(value(transcript, "rng-ok"), 1);
    assert_eq!(value(transcript, "sqrt2x1000"), 1414, "VSQRT.F32");
    assert_eq!(value(transcript, "atomic"), 42, "LDA/STL and LDAEX/STLEX");
    assert_eq!(value(transcript, "msplim"), 0x2000_0000, "MSPLIM");
    assert_eq!(value(transcript, "flash-erased"), 0xffff_ffff, "page erase");
    assert_eq!(
        value(transcript, "flash-word0"),
        0x0123_4567,
        "quad-word program"
    );
    assert_eq!(
        value(transcript, "flash-word3"),
        0x0bad_f00d,
        "quad-word program"
    );
    assert_eq!(value(transcript, "faults"), 0);
}

#[test]
fn the_u575_runs_sts_hal() {
    let Some(dir) = corpus() else { return };
    let image = std::fs::read(dir.join("u575.bin")).expect("u575.bin");
    let transcript = run("stm32u575", image);
    println!("{transcript}");
    check_common(&transcript);
}

#[test]
fn the_u585_runs_sts_hal_and_its_hash_unit() {
    let Some(dir) = corpus() else { return };
    let image = std::fs::read(dir.join("u585.bin")).expect("u585.bin");
    let transcript = run("stm32u585", image);
    println!("{transcript}");
    check_common(&transcript);
    // SHA-256("abc"), FIPS 180-2 appendix B.1: ba7816bf 8f01cfea ...
    assert_eq!(value(&transcript, "sha256-abc"), 0xba78_16bf);
}
