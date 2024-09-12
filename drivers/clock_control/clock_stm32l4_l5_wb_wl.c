/*
 *
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/sys/time_units.h>
#include "clock_stm32_ll_common.h"

#if defined(STM32_PLL_ENABLED)

#if defined(LL_RCC_MSIRANGESEL_RUN)
#define CALC_RUN_MSI_FREQ(range) __LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, \
							range << RCC_CR_MSIRANGE_Pos);
#else
#define CALC_RUN_MSI_FREQ(range) __LL_RCC_CALC_MSI_FREQ(range << RCC_CR_MSIRANGE_Pos);
#endif

/**
 * @brief Return PLL source
 */
__unused
static uint32_t get_pll_source(void)
{
	/* Configure PLL source */
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return LL_RCC_PLLSOURCE_HSI;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return LL_RCC_PLLSOURCE_HSE;
	} else if (IS_ENABLED(STM32_PLL_SRC_MSI)) {
		return LL_RCC_PLLSOURCE_MSI;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief get the pll source frequency
 */
__unused
uint32_t get_pllsrc_frequency(void)
{
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return STM32_HSI_FREQ;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return STM32_HSE_FREQ;
#if defined(STM32_MSI_ENABLED)
	} else if (IS_ENABLED(STM32_PLL_SRC_MSI)) {
		return CALC_RUN_MSI_FREQ(STM32_MSI_RANGE);
#endif
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief Set up pll configuration
 */
void config_pll_sysclock(void)
{
#ifdef PWR_CR5_R1MODE
	/* set power boost mode for sys clock greater than 80MHz */
	if (sys_clock_hw_cycles_per_sec() >= MHZ(80)) {
		LL_PWR_EnableRange1BoostMode();
	}
#endif /* PWR_CR5_R1MODE */

	LL_RCC_PLL_ConfigDomain_SYS(get_pll_source(),
				    pllm(STM32_PLL_M_DIVISOR),
				    STM32_PLL_N_MULTIPLIER,
				    pllr(STM32_PLL_R_DIVISOR));

	LL_RCC_PLL_EnableDomain_SYS();
}

#endif /* defined(STM32_PLL_ENABLED) */

#ifdef STM32_PLLSAI1_ENABLED

/**
 * @brief Set up PLL SAI1 configuration
 */
__unused
void config_pllsai1(void)
{
#if defined(STM32_SRC_PLLSAI1_Q) && STM32_PLLSAI1_Q_ENABLED && defined(RCC_PLLSAI1CFGR_PLLSAI1Q)
	MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1Q, pllsai1q(STM32_PLLSAI1_Q_DIVISOR));
#endif
#if defined(STM32_SRC_PLLSAI1_R) && STM32_PLLSAI1_R_ENABLED && defined(RCC_PLLSAI1CFGR_PLLSAI1R)
	MODIFY_REG(RCC->PLLSAI1CFGR, RCC_PLLSAI1CFGR_PLLSAI1R, pllsai1r(STM32_PLLSAI1_R_DIVISOR));
#endif
	LL_RCC_PLLSAI1_ConfigDomain_SAI(get_pll_source(),
				       pllm(STM32_PLLSAI1_M_DIVISOR),
				       STM32_PLLSAI1_N_MULTIPLIER,
				       pllsai1p(STM32_PLLSAI1_P_DIVISOR));
}

/**
 * @brief Set up PLL SAI2 configuration
 */
__unused
void config_pllsai2(void)
{
#if defined(STM32_SRC_PLLSAI2_Q) && STM32_PLLSAI2_Q_ENABLED && defined(RCC_PLLSAI2CFGR_PLLSAI2Q)
	MODIFY_REG(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2Q, pllsai2q(STM32_PLLSAI2_Q_DIVISOR));
#endif
#if defined(STM32_SRC_PLLSAI2_R) && STM32_PLLSAI2_R_ENABLED && defined(RCC_PLLSAI2CFGR_PLLSAI2R)
	MODIFY_REG(RCC->PLLSAI2CFGR, RCC_PLLSAI2CFGR_PLLSAI2R, pllsai2r(STM32_PLLSAI2_R_DIVISOR));
#endif
	LL_RCC_PLLSAI2_ConfigDomain_SAI(get_pll_source(),
				       pllm(STM32_PLLSAI2_M_DIVISOR),
				       STM32_PLLSAI2_N_MULTIPLIER,
				       pllsai2p(STM32_PLLSAI2_P_DIVISOR));
}

#endif /* STM32_PLLI2S_ENABLED */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
#ifdef LL_APB1_GRP1_PERIPH_PWR
	/* Enable the power interface clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
#endif
#if defined(CONFIG_SOC_SERIES_STM32WBX)
	/* HW semaphore Clock enable */
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_HSEM);
#endif
}
