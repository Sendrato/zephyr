/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

void main(void)
{
    k_sleep(K_SECONDS(4));
	printk("Hello World! %s\n", CONFIG_BOARD);
}
