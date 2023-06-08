/**
 * System clock configuration.
 *
 */

#include "sys.h"

/**
 * @brief       Start standby mode
 * @param
 * @retval
 */
void sys_standby(void) {
  __HAL_RCC_PWR_CLK_ENABLE();    /* Enable power clock */
  SET_BIT(PWR->CR, PWR_CR_PDDS); /* Start standby mode */
}

/**
 * @brief       System soft reset
 * @param
 * @retval
 */
void sys_soft_reset(void) { NVIC_SystemReset(); }

/**
 * @brief       System clock initialization function
 * @param       plln:PLL multiplication coefficient (PLL multiplication), value
 range: 2~16 The interrupt vector table location is already initialized in
 SystemInit() at startup
 * @retval
 */
void sys_stm32_clock_init(uint32_t plln) {
  RCC_OscInitTypeDef rcc_osc_init = {0};
  RCC_ClkInitTypeDef rcc_clk_init = {0};

  rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE; /* Select HSE */
  rcc_osc_init.HSEState = RCC_HSE_ON;                   /* Turn on HSE */
  rcc_osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;    /* HSE PREDIV 1 */
  rcc_osc_init.PLL.PLLState = RCC_PLL_ON;               /* Turn on PLL */
  rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE; /* Select HSE as PLL source */
  rcc_osc_init.PLL.PLLMUL = plln;         /* PLL multiplication factor */
  if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK) {
    /* The initialization failed, and subsequent programs may not be able to
     * execute normally, you can add your own processing here.
     */
    while (1);
  }

  /* Select PLL as the system clock source and configure HCLK, PCLK1 and PCLK2*/
  rcc_clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* Set system clock from PLL */
  rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
  /* Set the FLASH delay period to 2WS, which is 3 CPU cycles. */
  if (HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_2) != HAL_OK) {
    /* If the clock initialization is fail,
     * and programs will not be able to execute normally.
     * You can add processing here
     */
    while (1);
  }
}