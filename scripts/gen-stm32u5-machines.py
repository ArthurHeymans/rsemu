#!/usr/bin/env python3
"""Generate machines/stm32u575.machine and machines/stm32u585.machine.

Usage: scripts/gen-stm32u5-machines.py machines

The two parts share everything but the crypto blocks, and a `.machine` file
has no include the catalog can resolve, so both are written from here.
"""
import sys

PARTS = {
    "stm32u575": dict(
        title="STMicroelectronics STM32U575ZIT6Q",
        board="ST's NUCLEO-U575ZI-Q",
        package="LQFP144",
        hash=False,
    ),
    "stm32u585": dict(
        title="STMicroelectronics STM32U585AII6Q",
        board="ST's B-U585I-IOT02A Discovery kit",
        package="UFBGA169",
        hash=True,
    ),
}

PORTS = "abcdefghi"


def header(name, p):
    other = "stm32u585" if name == "stm32u575" else "stm32u575"
    crypto = (
        "It is the U575 plus the HASH processor (SHA-1/SHA-224/SHA-256/MD5,\n"
        "# HMAC) at AHB2 + 0xa0400. The AES, SAES, PKA and OTFDEC units the U585\n"
        "# also has are not modelled; their addresses are unmapped."
        if p["hash"]
        else "The U585 is the same die with the crypto blocks bonded out: see\n"
        f"# machines/{other}.machine, which adds the HASH unit."
    )
    return f"""# {name} — an {p['title']}.
#
#   rsemu run {name} --media firmware=app.bin
#
# A Cortex-M33 with FPv5-SP-D16 and the DSP extension, 2 MiB of flash in two
# 1 MiB banks, 768 KiB of SRAM in four blocks plus 2 KiB of backup SRAM, in
# the {p['package']} package used on {p['board']}.
# {crypto}
#
# Generated, with its sibling, by scripts/gen-stm32u5-machines.py: the two
# parts differ only in their crypto blocks, so both files come from one
# description and cannot drift. Edit the script, not this file.
#
# Sources: ST **RM0456** (STM32U5 reference manual) for the memory map, the
# reset and clock controller, the flash interface, EXTI and every peripheral
# placed below. ST's CMSIS device header `stm32u585xx.h` (Apache-2.0,
# cmsis-device-u5 v1.4.3) for every base address and for the vector table
# (`IRQn_Type`), which is where each `cpu.irqN` number below comes from. The
# Armv8-M Architecture Reference Manual, Arm DDI 0553, and the Cortex-M33
# TRM for the core. No emulator source of any licence was consulted
# (ROADMAP.md §1). docs/platforms/stm32u5.md has the full account.
#
# ---------------------------------------------------------------------------
# What is different from machines/stm32f407.machine
# ---------------------------------------------------------------------------
#
# * **TrustZone is off.** The silicon has it; the TZEN option bit is clear
#   out of the factory, and that is the part modelled: everything is
#   non-secure, firmware uses the non-secure aliases (flash at 0x08000000,
#   SRAM at 0x20000000, peripherals at 0x40000000), and the core is a v8-M
#   Mainline core without the Security Extension. The secure aliases at
#   0x0c000000, 0x30000000 and 0x50000000 are unmapped.
# * **Nothing is mapped at zero.** An F4 aliases its boot memory at
#   0x00000000; a U5 starts the core with VTOR already pointing at the boot
#   address (NSBOOTADD0, 0x08000000 out of the factory), so `cpu.vtor` is the
#   boot alias here and the code region below 0x08000000 is empty.
# * **The clock tree is live.** `st.rcc` drives the scheduler: the board comes
#   out of reset on MSIS at 4 MHz, as the part does, and runs at whatever
#   the firmware programs PLL1 to — 160 MHz for every Cube example. The
#   `clock` objects below are the nodes of that tree.
# * **The peripheral clocks are gated.** Every peripheral whose enable bit
#   resets to zero has its `enable` and `reset` pins wired from RCC, so a
#   driver that forgets `__HAL_RCC_xxx_CLK_ENABLE()` talks to a dead block,
#   exactly as on the silicon.
# * **EXTI owns the GPIO mux.** On a U5 the EXTICR registers are in EXTI, not
#   in SYSCFG, so every pin of every port is wired straight into `exti`, and
#   each of EXTI0..EXTI15 has its own vector (11..26).
"""


def body(name, p):
    L = []
    w = L.append
    w(f'machine "{name}" {{')
    w('  param console = "console"     # USART1 — the ST-LINK virtual COM port')
    w("  param flash   = 2M")
    w("  param seed    = 1")
    w("")
    w("  # The memory map, RM0456 §2.3 and the CMSIS header's *_BASE_NS.")
    w("  param flash-base   = 0x08000000")
    w("  param sram1-base   = 0x20000000   # SRAM1 192K, SRAM2 64K, SRAM3 512K: contiguous")
    w("  param sram4-base   = 0x28000000")
    w("  param apb1-base    = 0x40000000")
    w("  param apb2-base    = 0x40010000")
    w("  param ahb1-base    = 0x40020000")
    w("  param ahb2-base    = 0x42020000")
    w("  param apb3-base    = 0x46000000")
    w("  param ahb3-base    = 0x46020000")
    w("  param dbgmcu-base  = 0xE0044000")
    w("")
    w("  # The reference oscillator. The Discovery kit fits a 16 MHz HSE crystal;")
    w("  # every internal source is an exact ratio of it (docs/techniques/")
    w("  # clock-control.md, \"It does not reparent\"). MSIS's 4 MHz is hse / 4.")
    w("  osc hse = 16000000 Hz")
    w("  osc lsi = 32000 Hz")
    w("  # One tick of a USART's domain is one character time: 115200 baud, 10")
    w("  # bits a character. The baud rate the firmware programs is not modelled.")
    w("  osc uartclk = 11520 Hz")
    w("")
    w("  # The clock tree RCC drives, at its reset values: SYSCLK on MSIS, every")
    w("  # prescaler at /1. RCC re-rates these as the firmware reprograms it.")
    w("  object sysclk  \"clock\" { clock = hse / 4 }")
    w("  object hclk    \"clock\" { clock = sysclk }")
    w("  object pclk1   \"clock\" { clock = hclk }")
    w("  object pclk2   \"clock\" { clock = hclk }")
    w("  object pclk3   \"clock\" { clock = hclk }")
    w("  object timclk1 \"clock\" { clock = pclk1 }")
    w("  object timclk2 \"clock\" { clock = pclk2 }")
    w("")
    w("  space mem { width = 32 }")
    w("")
    w("  object cpu \"cpu.arm.v7m\" {")
    w("    clock         = hclk")
    w("    space         = mem")
    w("    part          = \"cortex-m33f\"   # FPv5-SP-D16 and DSP, the U5's configuration")
    w("    priority-bits = 4               # __NVIC_PRIO_BITS")
    w("    vtor          = flash-base      # NSBOOTADD0's factory value")
    w("  }")
    w("")
    w("  object eflash \"st.flash\" {")
    w("    clock   = hse")
    w("    variant = \"u5\"")
    w("    size    = flash")
    w("    image   = \"firmware\"")
    w("  }")
    w("  object sram1 \"ram\" { size = 192K }")
    w("  object sram2 \"ram\" { size = 64K }")
    w("  object sram3 \"ram\" { size = 512K }")
    w("  object sram4 \"ram\" { size = 16K }")
    w("  object bkpsram \"ram\" { size = 2K }")
    w("")
    w("  object rcc \"st.rcc\" {")
    w("    clock   = hse")
    w("    variant = \"u5\"")
    w("    hse     = 16000000")
    w("    sysclk  = \"sysclk\", hclk = \"hclk\"")
    w("    pclk1   = \"pclk1\", pclk2 = \"pclk2\", pclk3 = \"pclk3\"")
    w("    timclk1 = \"timclk1\", timclk2 = \"timclk2\"")
    w("  }")
    w("  object pwr \"st.pwr\" { clock = hse, variant = \"u5\" }")
    w("  object icache \"st.icache\" { clock = hclk, variant = \"icache\" }")
    w("  object dcache1 \"st.icache\" { clock = hclk, variant = \"dcache\" }")
    w("  # The factory signature in system memory: FLASH_SIZE (which the HAL's")
    w("  # page arithmetic reads), the 96-bit UID (drawn from `seed`) and the")
    w("  # package word, which reads zero: its encoding was not checked against")
    w("  # RM0456 and a guessed code would be worse than an obviously blank one.")
    w("  object devinfo \"st.devinfo\" {")
    w("    size = 0x2a4, flash-kb = 2048, seed = seed")
    w("    package-offset = 0x0, uid-offset = 0x200, flash-size-offset = 0x2a0")
    w("  }")
    w("  object exti \"st.exti\" { variant = \"u5\" }")
    w("  object dbgmcu \"st.dbgmcu\" { variant = \"u5\" }")
    w("")
    w("  # GPIO A–I. Out of reset every pin is analog (MODER all ones) except the")
    w("  # debug pins: PA13/PA14 (SWD) and PA15, PB3, PB4 (JTAG) in alternate")
    w("  # function, with their pulls (RM0456 §13.4).")
    w("  object gpioa \"st.gpio\" { variant = \"u5\", moder-reset = 0xabffffff, ospeedr-reset = 0x0c000000, pupdr-reset = 0x64000000 }")
    w("  object gpiob \"st.gpio\" { variant = \"u5\", moder-reset = 0xfffffebf, ospeedr-reset = 0x000000c0, pupdr-reset = 0x00000100 }")
    for c in PORTS[2:]:
        w(f"  object gpio{c} \"st.gpio\" {{ variant = \"u5\", moder-reset = 0xffffffff }}")
    w("")
    w("  # The U(S)ARTs. USART1 is the console: PA9/PA10, the ST-LINK VCP.")
    usarts = [
        ("usart1", "console", ""),
        ("usart2", '"usart2"', ""),
        ("usart3", '"usart3"', ""),
        ("uart4", '"uart4"', ""),
        ("uart5", '"uart5"', ""),
        ("lpuart1", '"lpuart1"', ", lpuart = true"),
    ]
    for n, port, extra in usarts:
        w(f"  object {n:<7} \"st.usart\" {{ clock = uartclk, variant = \"u5\", port = {port}{extra} }}")
    w("")
    w("  # Timers, all on the U5 register layout (CCR5/CCR6..OR1 at 0x48, DCR/DMAR")
    w("  # at 0x3dc). TIM2 and TIM5 are 32-bit.")
    tims = [
        ("tim1", "timclk2", "advanced", 4, None),
        ("tim2", "timclk1", "general", 4, 32),
        ("tim3", "timclk1", "general", 4, None),
        ("tim4", "timclk1", "general", 4, None),
        ("tim5", "timclk1", "general", 4, 32),
        ("tim6", "timclk1", "basic", None, None),
        ("tim7", "timclk1", "basic", None, None),
        ("tim8", "timclk2", "advanced", 4, None),
        ("tim15", "timclk2", "general", 2, None),
        ("tim16", "timclk2", "general", 1, None),
        ("tim17", "timclk2", "general", 1, None),
    ]
    for n, clk, var, ch, width in tims:
        s = f"  object {n:<5} \"st.tim\" {{ clock = {clk}, variant = \"{var}\", layout = \"u5\""
        if ch:
            s += f", channels = {ch}"
        if width:
            s += f", width = {width}"
        w(s + " }")
    w("")
    w("  object iwdg \"st.iwdg\" { clock = lsi, variant = \"u5\" }")
    w("  # WWDG_CFR.WDGTB is three bits at 13:11 on a U5 (two at 8:7 on an F4).")
    w("  object wwdg \"st.wwdg\" { clock = pclk1, wdgtb-shift = 11, wdgtb-width = 3 }")
    w("  object crc  \"st.crc\"  { variant = \"u5\" }")
    w("  # The RNG runs on HSI48. Its numbers come from `seed`, not from the host:")
    w("  # docs/platforms/stm32f407.md has why.")
    w("  object rng  \"st.rng\"  { clock = hse * 3, seed = seed, variant = \"u5\" }")
    if p["hash"]:
        w("  object hash \"st.hash\" { variant = \"u5\" }")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # The map")
    w("  # ---------------------------------------------------------------------")
    w("  map mem flash-base size flash = eflash.array")
    w("  map mem sram1-base             size 192K = sram1")
    w("  map mem sram1-base + 0x30000   size 64K  = sram2")
    w("  map mem sram1-base + 0x40000   size 512K = sram3")
    w("  map mem sram4-base             size 16K  = sram4")
    w("  map mem 0x0bfa0500             size 0x2a4 = devinfo   # PACKAGE_BASE")
    w("")
    maps = [
        ("apb1-base + 0x0000", "0x3e4", "tim2"),
        ("apb1-base + 0x0400", "0x3e4", "tim3"),
        ("apb1-base + 0x0800", "0x3e4", "tim4"),
        ("apb1-base + 0x0c00", "0x3e4", "tim5"),
        ("apb1-base + 0x1000", "0x3e4", "tim6"),
        ("apb1-base + 0x1400", "0x3e4", "tim7"),
        ("apb1-base + 0x2c00", "0x0c", "wwdg"),
        ("apb1-base + 0x3000", "0x18", "iwdg"),
        ("apb1-base + 0x4400", "0x34", "usart2"),
        ("apb1-base + 0x4800", "0x34", "usart3"),
        ("apb1-base + 0x4c00", "0x34", "uart4"),
        ("apb1-base + 0x5000", "0x34", "uart5"),
        ("apb2-base + 0x2c00", "0x3e4", "tim1"),
        ("apb2-base + 0x3400", "0x3e4", "tim8"),
        ("apb2-base + 0x3800", "0x34", "usart1"),
        ("apb2-base + 0x4000", "0x3e4", "tim15"),
        ("apb2-base + 0x4400", "0x3e4", "tim16"),
        ("apb2-base + 0x4800", "0x3e4", "tim17"),
        ("ahb1-base + 0x2000", "0x100", "eflash"),
        ("ahb1-base + 0x3000", "0x18", "crc"),
        ("ahb1-base + 0x10400", "0x30", "icache"),
        ("ahb1-base + 0x11400", "0x30", "dcache1"),
        ("ahb1-base + 0x16400", "2K", "bkpsram"),
    ]
    for i, c in enumerate(PORTS):
        maps.append((f"ahb2-base + 0x{i * 0x400:04x}", "0x34", f"gpio{c}"))
    if p["hash"]:
        maps.append(("ahb2-base + 0xa0400", "0x330", "hash"))
    maps += [
        ("ahb2-base + 0xa0800", "0x14", "rng"),
        ("apb3-base + 0x2400", "0x34", "lpuart1"),
        ("ahb3-base + 0x0800", "0x98", "pwr"),
        ("ahb3-base + 0x0c00", "0x118", "rcc"),
        ("ahb3-base + 0x2000", "0x88", "exti"),
        ("dbgmcu-base", "0x20", "dbgmcu"),
    ]
    for base, size, dev in maps:
        w(f"  map mem {base:<20} size {size:<5} = {dev}")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # Interrupts: `IRQn_Type` in stm32u585xx.h, interrupt n -> cpu.irq{n}")
    w("  # ---------------------------------------------------------------------")
    irqs = [
        ("wwdg.ewi", 0, "WWDG"),
        ("eflash.irq", 6, "FLASH"),
        ("rcc.irq", 9, "RCC"),
        ("iwdg.irq", 27, "IWDG (early wakeup)"),
        ("tim1.irq-up", 42, "TIM1_UP"),
        ("tim1.irq-trg", 43, "TIM1_TRG_COM"),
        ("tim1.irq-cc", 44, "TIM1_CC"),
        ("tim2.irq", 45, "TIM2"),
        ("tim3.irq", 46, "TIM3"),
        ("tim4.irq", 47, "TIM4"),
        ("tim5.irq", 48, "TIM5"),
        ("tim6.irq", 49, "TIM6"),
        ("tim7.irq", 50, "TIM7"),
        ("tim8.irq-up", 52, "TIM8_UP"),
        ("tim8.irq-trg", 53, "TIM8_TRG_COM"),
        ("tim8.irq-cc", 54, "TIM8_CC"),
        ("usart1.irq", 61, "USART1"),
        ("usart2.irq", 62, "USART2"),
        ("usart3.irq", 63, "USART3"),
        ("uart4.irq", 64, "UART4"),
        ("uart5.irq", 65, "UART5"),
        ("lpuart1.irq", 66, "LPUART1"),
        ("tim15.irq", 69, "TIM15"),
        ("tim16.irq", 70, "TIM16"),
        ("tim17.irq", 71, "TIM17"),
        ("rng.irq", 94, "RNG"),
        ("icache.irq", 107, "ICACHE"),
        ("dcache1.irq", 111, "DCACHE1"),
    ]
    if p["hash"]:
        irqs.append(("hash.irq", 96, "HASH"))
    irqs.sort(key=lambda t: t[1])
    for src, n, label in irqs:
        w(f"  wire {src:<13} -> cpu.irq{n:<4} # {label}")
    w("  # EXTI0..EXTI15 each have their own vector on a U5.")
    w("  for n in 0..16 {")
    w("    wire exti.irq${n} -> cpu.irq${n + 11}")
    w("  }")
    w("")
    w("  # Every pin of every port into the EXTI mux; EXTICR picks one per line.")
    for c in PORTS:
        w(f"  for n in 0..16 {{ wire gpio{c}.p${{n}} -> exti.p{c}${{n}} }}")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # Clock enables and resets, RCC -> peripheral. Bit numbers are the")
    w("  # header's RCC_xxxENR_yyyEN_Pos; RCC names the second register of a pair")
    w("  # with a `b` (AHB2ENR2 is `ahb2enb`, APB1ENR2 is `apb1enb`).")
    w("  # ---------------------------------------------------------------------")
    gates = [("ahb3", 2, "pwr"), ("ahb1", 12, "crc")]
    gates += [("ahb2", i, f"gpio{c}") for i, c in enumerate(PORTS)]
    if p["hash"]:
        gates.append(("ahb2", 17, "hash"))
    gates.append(("ahb2", 18, "rng"))
    gates += [("apb1", i, f"tim{i + 2}") for i in range(6)]
    gates += [
        ("apb1", 17, "usart2"),
        ("apb1", 18, "usart3"),
        ("apb1", 19, "uart4"),
        ("apb1", 20, "uart5"),
        ("apb2", 11, "tim1"),
        ("apb2", 13, "tim8"),
        ("apb2", 14, "usart1"),
        ("apb2", 16, "tim15"),
        ("apb2", 17, "tim16"),
        ("apb2", 18, "tim17"),
        ("apb3", 6, "lpuart1"),
    ]
    for bus, bit, dev in gates:
        w(f"  wire rcc.{bus}en{bit:<2} -> {dev}.enable")
        # PWR has no reset bit in AHB3RSTR.
        if dev != "pwr":
            w(f"  wire rcc.{bus}rst{bit:<2} -> {dev}.reset")
    w("")
    w("  # Backup-domain write protection: PWR_DBPR.DBP unlocks RCC_BDCR.")
    w("  wire pwr.dbp -> rcc.dbp")
    w("")
    w("  # Resets. Each watchdog latches its own cause in RCC_CSR; they meet at a")
    w("  # wire.or rather than sharing a net, so firmware can still tell which one")
    w("  # rebooted it. OBL_LAUNCH reloads the option bytes, which is a reset too.")
    w("  object sysreset \"wire.or\" { inputs = 3 }")
    w("  wire iwdg.reset   -> rcc.iwdgrst")
    w("  wire iwdg.reset   -> sysreset.in0")
    w("  wire wwdg.reset   -> rcc.wwdgrst")
    w("  wire wwdg.reset   -> sysreset.in1")
    w("  wire eflash.reset -> rcc.oblrst")
    w("  wire eflash.reset -> sysreset.in2")
    w("  wire sysreset.out -> cpu.reset")
    w("  wire dbgmcu.iwdg  -> iwdg.freeze")
    w("  wire dbgmcu.wwdg  -> wwdg.freeze")
    w("}")
    return "\n".join(L) + "\n"


def main(outdir):
    for name, p in PARTS.items():
        with open(f"{outdir}/{name}.machine", "w") as f:
            f.write(header(name, p) + "\n" + body(name, p))


if __name__ == "__main__":
    main(sys.argv[1])
