/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif

#include "lcm_gpio_get.h"


#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#endif

static int __init lcm_init(void)
{
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	printk("[KE/LCM] GPIO_LCD_RST = 0x%x\n", GPIO_LCD_RST);

	gpio_request(GPIO_LCD_PWR_ENN, "GPIO_LCD_PWR_ENN");
	printk("[KE/LCM] GPIO_LCD_PWR_ENN = 0x%x\n", GPIO_LCD_PWR_ENN);

	gpio_request(GPIO_LCD_PWR_ENP, "GPIO_LCD_PWR_ENP");
	printk("[KE/LCM] GPIO_LCD_PWR_ENP = 0x%x\n", GPIO_LCD_PWR_ENP);

	gpio_request(GPIO_LCD_PWM_EN, "GPIO_LCD_PWM_EN");
	printk("[KE/LCM] GPIO_LCD_PWM_EN = 0x%x\n", GPIO_LCD_PWM_EN);
	
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	printk("[KE/LCM] GPIO_LCD_BL_EN = 0x%x\n", GPIO_LCD_BL_EN);

	gpio_request(GPIO_CTP_RST, "GPIO_CTP_RST");
	printk("[KE/LCM] GPIO_CTP_RST = 0x%x\n", GPIO_CTP_RST);
	return 0;
}

static void __exit lcm_exit(void)
{
//	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
