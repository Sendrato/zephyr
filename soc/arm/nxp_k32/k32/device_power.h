//
// Created by ijsbrand on 1-11-23.
//

#include <zephyr/sys/printk.h>
#include <fsl_gpio.h>
#include <fsl_power.h>

#ifndef PM_DEVICE_POWER_H
#define PM_DEVICE_POWER_H

extern WEAK void WarmMain(void);

#define BOARD_SW1_GPIO_PIN 1U
#define LED_PORT_0 0
#define LED_PORT_1 0
#define LED_PIN_0 0
#define LED_PIN_1 3

void init_config(pm_wake_source_t pm_wakeup_src, uint32_t pm_wakeup_io, uint32_t pm_config);

#endif //PM_DEVICE_POWER_H
