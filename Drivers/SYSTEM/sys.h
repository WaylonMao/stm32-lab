/**
 * System clock configuration.
 *
 */

#ifndef __SYS_H
#define __SYS_H

#include "stm32f1xx.h"

void sys_standby(void);    /* Start standby mode */
void sys_soft_reset(void); /* System soft reset */
void sys_stm32_clock_init(
    uint32_t plln); /* System clock initialization function */
#endif
