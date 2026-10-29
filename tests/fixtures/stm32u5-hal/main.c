/*
 * STM32U5 HAL smoke firmware for rsemu's `stm32u575` / `stm32u585` boards.
 *
 * Written for rsemu; MIT, like the rest of the repository. It is built by
 * `scripts/fetch-testdata.sh stm32u5-hal` against ST's own CMSIS device files
 * (Apache-2.0) and HAL driver (BSD-3-Clause), which that script downloads and
 * which are never committed here. tests/stm32u5_hal.rs runs the result.
 *
 * The point is the vendor code, not this file: every wait loop the HAL runs
 * against RCC, PWR, FLASH, ICACHE, RNG and the USART is a question the board's
 * models have to answer the way the silicon does, and a model that answers one
 * wrong shows up here as a HAL_TIMEOUT rather than as a hang. Each step prints
 * one line on USART1 (the NUCLEO-U575ZI-Q's ST-LINK virtual COM port), and the
 * test compares the transcript.
 */
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "stm32u5xx_hal.h"

static UART_HandleTypeDef huart1;
static TIM_HandleTypeDef htim6;
static RNG_HandleTypeDef hrng;
static CRC_HandleTypeDef hcrc;

static volatile uint32_t tim6_ticks;
static volatile uint32_t exti13_hits;
static volatile uint32_t faults;

/* -------------------------------------------------------------------------
 * The three C library routines the compiler and this file need. The firmware
 * is freestanding: the toolchain it is built with may ship no libc at all.
 * ------------------------------------------------------------------------- */

void *memset(void *dst, int c, size_t n)
{
    unsigned char *d = dst;
    while (n--) {
        *d++ = (unsigned char)c;
    }
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    unsigned char *d = dst;
    const unsigned char *s = src;
    while (n--) {
        *d++ = *s++;
    }
    return dst;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n] != 0) {
        n++;
    }
    return n;
}

/* ST's startup file calls this to run C++ constructors; there are none. */
void __libc_init_array(void) {}

/* -------------------------------------------------------------------------
 * Output
 * ------------------------------------------------------------------------- */

static void put(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 1000);
}

static void put_hex(uint32_t v)
{
    char buf[11] = "0x";
    for (int i = 0; i < 8; i++) {
        uint32_t nib = (v >> (28 - 4 * i)) & 0xf;
        buf[2 + i] = (char)(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
    buf[10] = 0;
    put(buf);
}

static void put_dec(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = 0;
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v != 0);
    put(&buf[i]);
}

static void line(const char *name, uint32_t v)
{
    put(name);
    put(": ");
    put_hex(v);
    put("\r\n");
}

static void check(const char *step, HAL_StatusTypeDef st)
{
    if (st != HAL_OK) {
        put("FAIL ");
        put(step);
        put(" status ");
        put_dec((uint32_t)st);
        put("\r\n");
        for (;;) {
        }
    }
}

/* -------------------------------------------------------------------------
 * Clocks: MSIS 4 MHz -> PLL1 -> 160 MHz, the NUCLEO examples' configuration
 * ------------------------------------------------------------------------- */

static void check(const char *step, HAL_StatusTypeDef st);

static HAL_StatusTypeDef clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    check("clock-vos", HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1));

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_MSI;
    osc.HSI48State = RCC_HSI48_ON;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.MSIClockRange = RCC_MSIRANGE_4;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    osc.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
    osc.PLL.PLLM = 1;
    osc.PLL.PLLN = 80;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 2;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
    osc.PLL.PLLFRACN = 0;
    check("clock-osc", HAL_RCC_OscConfig(&osc));

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_HCLK_DIV1;
    check("clock-bus", HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4));
    return HAL_OK;
}

/* -------------------------------------------------------------------------
 * MSP hooks the HAL calls from its Init functions
 * ------------------------------------------------------------------------- */

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    (void)h;
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h)
{
    (void)h;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *h)
{
    RCC_PeriphCLKInitTypeDef p = {0};
    (void)h;
    p.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    p.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    check("rng-clk", HAL_RCCEx_PeriphCLKConfig(&p));
    __HAL_RCC_RNG_CLK_ENABLE();
}

void HAL_CRC_MspInit(CRC_HandleTypeDef *h)
{
    (void)h;
    __HAL_RCC_CRC_CLK_ENABLE();
}

/* -------------------------------------------------------------------------
 * Interrupt handlers (the startup file's vector table names these)
 * ------------------------------------------------------------------------- */

void SysTick_Handler(void) { HAL_IncTick(); }
void TIM6_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }
void EXTI13_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if (h->Instance == TIM6) {
        tim6_ticks++;
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_13) {
        exti13_hits++;
    }
}

static void fault(const char *what)
{
    faults++;
    put("FAULT ");
    put(what);
    put(" CFSR ");
    put_hex(SCB->CFSR);
    put("\r\n");
    for (;;) {
    }
}

void HardFault_Handler(void) { fault("hard"); }
void MemManage_Handler(void) { fault("memmanage"); }
void BusFault_Handler(void) { fault("bus"); }
void UsageFault_Handler(void) { fault("usage"); }

static HAL_StatusTypeDef console_init(void)
{
    HAL_StatusTypeDef st;
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    st = HAL_UART_Init(&huart1);
    if (st != HAL_OK) {
        return st;
    }
    return HAL_UARTEx_DisableFifoMode(&huart1);
}

/* -------------------------------------------------------------------------
 * The steps
 * ------------------------------------------------------------------------- */

static void step_flash(void)
{
    /* The last 8 KiB page of bank 2: nothing the image itself lives in. */
    const uint32_t page_addr = FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE;
    static const uint32_t quad[4] = {0x01234567u, 0x89abcdefu, 0xdeadbeefu, 0x0badf00du};
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t bad = 0;

    check("flash-unlock", HAL_FLASH_Unlock());
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = FLASH_PAGE_NB - 1;
    erase.NbPages = 1;
    check("flash-erase", HAL_FLASHEx_Erase(&erase, &bad));
    line("flash-erased", *(volatile uint32_t *)page_addr);
    check("flash-program", HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, page_addr, (uint32_t)quad));
    check("flash-lock", HAL_FLASH_Lock());
    line("flash-word0", *(volatile uint32_t *)page_addr);
    line("flash-word3", *(volatile uint32_t *)(page_addr + 12));
}

#if defined(STM32U585xx)
static void step_hash(void)
{
    HASH_HandleTypeDef hhash = {0};
    static const uint8_t abc[3] = {'a', 'b', 'c'};
    uint8_t digest[32];

    __HAL_RCC_HASH_CLK_ENABLE();
    hhash.Init.DataType = HASH_DATATYPE_8B;
    check("hash-init", HAL_HASH_Init(&hhash));
    check("hash", HAL_HASHEx_SHA256_Start(&hhash, abc, sizeof abc, digest, 1000));
    line("sha256-abc", ((uint32_t)digest[0] << 24) | ((uint32_t)digest[1] << 16) |
                           ((uint32_t)digest[2] << 8) | digest[3]);
}
#endif

int main(void)
{
    static const uint8_t check_input[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    GPIO_InitTypeDef g = {0};
    uint32_t start, random;

    /* The console first, on the 4 MHz MSIS the part resets on, so that a
     * failure in the clock configuration below has somewhere to be reported.
     * It is initialised again afterwards: BRR is computed from PCLK2. */
    if (HAL_Init() != HAL_OK || console_init() != HAL_OK) {
        for (;;) {
        }
    }
    put("boot\r\n");
    check("clock", clock_config());
    check("console", console_init());

    put("rsemu stm32u5 hal smoke\r\n");
    line("cpuid", SCB->CPUID);
    line("sysclk", HAL_RCC_GetSysClockFreq());
    line("hclk", HAL_RCC_GetHCLKFreq());

    check("icache", HAL_ICACHE_Enable());
    line("icache-cr", ICACHE->CR);

    /* SysTick: HAL_Delay is a wait on the tick interrupt. */
    start = HAL_GetTick();
    HAL_Delay(10);
    line("delay-ok", (HAL_GetTick() - start) >= 10);

    /* GPIO: the NUCLEO's green LED is PC7. */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    g.Pin = GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    line("led", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7));

    /* EXTI: the user button is PC13. A software trigger stands in for it. */
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_NVIC_SetPriority(EXTI13_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI13_IRQn);
    EXTI->SWIER1 = EXTI_SWIER1_SWI13;
    __DSB();
    __ISB();
    line("exti13", exti13_hits);

    /* TIM6 at 1 kHz from the 160 MHz timer clock; wait for ten updates. */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 159;
    htim6.Init.Period = 999;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    check("tim6-init", HAL_TIM_Base_Init(&htim6));
    check("tim6-start", HAL_TIM_Base_Start_IT(&htim6));
    start = HAL_GetTick();
    while (tim6_ticks < 10 && HAL_GetTick() - start < 100) {
    }
    HAL_TIM_Base_Stop_IT(&htim6);
    line("tim6-ok", tim6_ticks >= 10);

    /* CRC: the default configuration is CRC-32/MPEG-2. */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    check("crc-init", HAL_CRC_Init(&hcrc));
    line("crc", HAL_CRC_Calculate(&hcrc, (uint32_t *)check_input, sizeof check_input));

    /* RNG: the U5 block's conditioning reset handshake. */
    hrng.Instance = RNG;
    hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
    check("rng-init", HAL_RNG_Init(&hrng));
    check("rng", HAL_RNG_GenerateRandomNumber(&hrng, &random));
    line("rng-ok", 1);

    /* FPv5-SP: VSQRT and a float-to-integer conversion. */
    {
        volatile float two = 2.0f;
        line("sqrt2x1000", (uint32_t)(sqrtf(two) * 1000.0f));
    }

    /* Armv8-M acquire/release: C11 atomics compile to LDA/STL and LDAEX/STLEX. */
    {
        static _Atomic uint32_t counter;
        atomic_store_explicit(&counter, 40, memory_order_release);
        atomic_fetch_add_explicit(&counter, 2, memory_order_acq_rel);
        line("atomic", atomic_load_explicit(&counter, memory_order_acquire));
    }

    /* The Armv8-M stack limit: a limit below the stack in use must not fire. */
    __set_MSPLIM(SRAM1_BASE);
    line("msplim", __get_MSPLIM());

    step_flash();
#if defined(STM32U585xx)
    step_hash();
#endif

    line("faults", faults);
    put("PASS\r\n");
    for (;;) {
        __WFI();
    }
}
