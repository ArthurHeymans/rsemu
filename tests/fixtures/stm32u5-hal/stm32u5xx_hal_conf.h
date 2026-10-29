/*
 * HAL configuration for the STM32U5 HAL smoke firmware (tests/stm32u5_hal.rs).
 *
 * Written for rsemu (MIT). ST's HAL reads this header to learn which modules
 * are compiled in and what the board's oscillators are. Only the modules the
 * app uses are enabled, so a HAL file the board cannot back is not linked.
 */
#ifndef STM32U5xx_HAL_CONF_H
#define STM32U5xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_ICACHE_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#if defined(STM32U585xx)
#define HAL_HASH_MODULE_ENABLED
#endif

/* The NUCLEO-U575ZI-Q has no HSE crystal fitted; MSIS starts the part. */
#define HSE_VALUE            16000000UL
#define HSE_STARTUP_TIMEOUT  100UL
#define MSI_VALUE            4000000UL
#define HSI_VALUE            16000000UL
#define HSI48_VALUE          48000000UL
#define LSI_VALUE            32000UL
#define LSI_STARTUP_TIMEOUT  130UL
#define LSE_VALUE            32768UL
#define LSE_STARTUP_TIMEOUT  5000UL
#define EXTERNAL_SAI1_CLOCK_VALUE 48000UL
#define EXTERNAL_SAI2_CLOCK_VALUE 48000UL

#define VDD_VALUE            3300UL
#define TICK_INT_PRIORITY    ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define USE_RTOS             0U
#define PREFETCH_ENABLE      1U
#define USE_SPI_CRC          0U

#define assert_param(expr) ((void)0U)

#include "stm32u5xx_hal_rcc.h"
#include "stm32u5xx_hal_gpio.h"
#include "stm32u5xx_hal_dma.h"
#include "stm32u5xx_hal_cortex.h"
#include "stm32u5xx_hal_crc.h"
#include "stm32u5xx_hal_exti.h"
#include "stm32u5xx_hal_flash.h"
#include "stm32u5xx_hal_icache.h"
#include "stm32u5xx_hal_pwr.h"
#include "stm32u5xx_hal_rng.h"
#include "stm32u5xx_hal_tim.h"
#include "stm32u5xx_hal_uart.h"
#if defined(STM32U585xx)
#include "stm32u5xx_hal_hash.h"
#endif

#endif /* STM32U5xx_HAL_CONF_H */
