/**
 * LED Driver
 * LED0: PB5
 * LED1: PE5
 */

#include "led.h"

/**
 * @brief
 * @param
 * @retval
 */
void led_init(void) {
  GPIO_InitTypeDef gpio_init_struct;
  LED0_GPIO_CLK_ENABLE(); /* Enable LED0 Clock */
  LED1_GPIO_CLK_ENABLE(); /* Enable LED1 Clock */

  gpio_init_struct.Pin = LED0_GPIO_PIN;         /* LED0 PIN */
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;  /* Push-pull Output */
  gpio_init_struct.Pull = GPIO_PULLUP;          /* Pull up */
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW; /* Low speed */
  HAL_GPIO_Init(LED0_GPIO_PORT,
                &gpio_init_struct); /* Initialize the LED0 pin */

  gpio_init_struct.Pin = LED1_GPIO_PIN; /* Initialize the LED1 pin */
  HAL_GPIO_Init(LED1_GPIO_PORT,
                &gpio_init_struct); /* Initialize the LED1 pin */

  LED0(1); /* Turn off LED0 */
  LED1(1); /* Turn off LED1 */
}
