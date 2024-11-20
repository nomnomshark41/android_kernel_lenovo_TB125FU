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

#include "lcm_drv.h"
#include "lcm_util.h"
#include "Backlight_I2C_map_hx.h"
#include "../sgm37604a.h"
#include "../ocp2138.h"
#include "../lcm_gpio_get.h"


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

#define LCM_PHYSICAL_WIDTH_UM  (138600)
#define LCM_PHYSICAL_HEIGHT_UM (231000)
#define LCM_PHYSICAL_WIDTH    (LCM_PHYSICAL_WIDTH_UM/1000)
#define LCM_PHYSICAL_HEIGHT   (LCM_PHYSICAL_HEIGHT_UM/1000)

#if 1



static void lcm_request_gpio_control(struct device *dev)
{
	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	printk("[KE/LCM] GPIO_LCD_RST = 0x%x\n", GPIO_LCD_RST);

	GPIO_LCD_PWR_ENN = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr_enn", 0);
	gpio_request(GPIO_LCD_PWR_ENN, "GPIO_LCD_PWR_ENN");
	printk("[KE/LCM] GPIO_LCD_PWR_ENN = 0x%x\n", GPIO_LCD_PWR_ENN);

	GPIO_LCD_PWR_ENP = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr_enp", 0);
	gpio_request(GPIO_LCD_PWR_ENP, "GPIO_LCD_PWR_ENP");
	printk("[KE/LCM] GPIO_LCD_PWR_ENP = 0x%x\n", GPIO_LCD_PWR_ENP);

	GPIO_LCD_PWM_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_pwm_en",0);
	gpio_request(GPIO_LCD_PWM_EN, "GPIO_LCD_PWM_EN");
	printk("[KE/LCM] GPIO_LCD_PWM_EN = 0x%x\n", GPIO_LCD_PWM_EN);
	
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en",0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	printk("[KE/LCM] GPIO_LCD_BL_EN = 0x%x\n", GPIO_LCD_BL_EN);
	
	GPIO_CTP_RST = of_get_named_gpio(dev->of_node, "gpio_ctp_rst",0);
	gpio_request(GPIO_CTP_RST, "GPIO_CTP_RST");
	printk("[KE/LCM] GPIO_CTP_RST = 0x%x\n", GPIO_LCD_BL_EN);
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	printk("[KE/LCM] nt36532w lcm_driver_probe Enter\n");
	
	lcm_request_gpio_control(dev);

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "mediatek,lcd_gpio1",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{

	const struct of_device_id *id;
	printk("lcm_platform_probe gpio enter\n");
	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
	
}

static int lcm_platform_remove(struct platform_device *dev)
{
	return 0;
};

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.remove = lcm_platform_remove,
	.driver = {
		.name = "hx83102p_wxga_vdo_incell_boe",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_init(void)
{
	printk("lcm init gpio enter\n");
	platform_driver_register(&lcm_driver);

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");



#endif

/* --------------------------------------------------------------------- */
/*  Local Constants */
/* --------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                         (1200)
#define FRAME_HEIGHT                                        (2000)
#define LCM_ID_NT35595                                      (0x95)
#define GPIO_OUT_ONE                                        1
#define GPIO_OUT_ZERO                                       0

#define REGFLAG_DELAY                                       0xFC
#define REGFLAG_END_OF_TABLE                                0xFD

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------- */
/*  Local Variables */
/* --------------------------------------------------------------------- */
static struct LCM_UTIL_FUNCS lcm_util = {0};
#define SET_RESET_PIN(v)     (lcm_util.set_reset_pin((v)))
#define UDELAY(n)            (lcm_util.udelay(n))
#define MDELAY(n)            (lcm_util.mdelay(n))

/* --------------------------------------------------------------------- */
/*  Local Functions */
/* --------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif
#if 0
static unsigned int GPIO_LCD_RST     = 370;/* GPIO45 */
static unsigned int GPIO_LCD_PWR_ENN = 494;/* GPIO169 */
static unsigned int GPIO_LCD_PWR_ENP = 490;/* GPIO165 */
static unsigned int GPIO_LCD_PWM_EN  = 368;/* GPIO43 */
static unsigned int GPIO_LCD_BL_EN   = 483;/* GPIO158 */
#endif
static unsigned short s_last_backlight_level = 0;
//Gesture flag
volatile bool hx_gesture_flag = false;

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 102, {} }, //<0.7s
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 38, {} }, //2 frames+5ms=33+5
	{0xB1, 1, {0x21} },
	{REGFLAG_DELAY, 50, {} },
};

static struct LCM_setting_table lcm_gesture_suspend_setting[] = {
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 102, {} }, //<0.7s
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 38, {} }, //2 frames+5ms=33+5
	/*{0xB1, 1, {0x21} },
	{REGFLAG_DELAY, 50, {} },*/
};

static struct LCM_setting_table lcm_initinal_setting[] = {
		{0xB9, 3, {0x83,0x10,0x2E}},
        {0xE9, 1, {0xCD}},
        {0xBB, 1, {0x01}},
        {REGFLAG_DELAY, 5, {}},
        {0xE9, 1, {0x00}},
        {0xD1, 4, {0x67,0x0C,0xFF,0x05}},
        {0xB1, 17, {0x10,0xFA,0xAF,0xAF,0x2B,0x2B,0xC1,0x75,0x39,0x36,0x36,0x36,0x36,0x22,0x21,0x15,0x00}},
        {0xB2, 16, {0x00,0xB0,0x47,0xD0,0x00,0x2C,0x50,0x2C,0x00,0x00,0x00,0x00,0x15,0x20,0xD7,0x00}},
        {0xB4, 16, {0x38,0x47,0x38,0x47,0x66,0x4E,0x00,0x00,0x01,0x72,0x01,0x58,0x00,0xFF,0x00,0xFF}},
        {0xBF, 3, {0xFC,0x85,0x80}},
        {0xD2, 2, {0x2B,0x2B}},
        {0xD3, 43, {0x00,0x00,0x00,0x00,0x78,0x04,0x00,0x14,0x00,0x27,0x00,0x44,0x4F,0x29,0x29,0x00,0x00,0x32,0x10,0x25,0x00,0x25,0x32,0x10,0x1F,0x00,0x1F,0x32,0x18,0x10,0x08,0x10,0x00,0x00,0x20,0x30,0x01,0x55,0x21,0x2E,0x01,0x55,0x0F}},
        {REGFLAG_DELAY, 5, {}},
	{0xE0, 46, {0x00,0x03,0x0B,0x11,0x18,0x26,0x3E,0x45,0x4E,0x4B,0x67,0x6E,0x77,0x89,0x89,0x94,0x9E,0xB1,0xB1,0x57,0x5E,0x68,0x70,0x00,0x03,0x0B,0x11,0x18,0x26,0x3E,0x45,0x4E,0x4B,0x67,0x6E,0x77,0x89,0x89,0x94,0x9E,0xB1,0xB1,0x57,0x5E,0x68,0x70}},
	{REGFLAG_DELAY, 5, {}},
	{0xC1, 1, {0x01}},
        {0xBD, 1, {0x01}},
	{0xC1, 58, {0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x45,0x49,0x4D,0x55,0x5D,0x65,0x6D,0x75,0x7D,0x85,0x8D,0x95,0x9C,0xA4,0xAC,0xB5,0xBC,0xC4,0xCD,0xD5,0xDC,0xE4,0xEC,0xF4,0xF8,0xFA,0xFC,0xFE,0xFF,0x00,0x54,0x56,0x5F,0xF5,0xBA,0xA4,0xFB,0x68,0xFF,0xA5,0x00}},
        {0xBD, 1, {0x02}},
	{0xC1, 58, {0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x54,0x5C,0x64,0x6C,0x74,0x7C,0x84,0x8C,0x93,0x9B,0xA3,0xAB,0xB4,0xBB,0xC3,0xCC,0xD4,0xDB,0xE4,0xEC,0xF4,0xF8,0xFA,0xFC,0xFE,0xFF,0x00,0x54,0x56,0x5F,0xF5,0xBA,0xA4,0xFB,0x68,0x30,0x00,0x00}},
        {0xBD, 1, {0x03}},
	{0xC1, 58, {0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C,0x40,0x44,0x48,0x4C,0x54,0x5C,0x64,0x6B,0x73,0x7B,0x83,0x8B,0x92,0x9A,0xA2,0xAA,0xB3,0xBA,0xC2,0xCB,0xD3,0xDA,0xE3,0xEB,0xF3,0xF7,0xF9,0xFC,0xFE,0xFF,0x00,0x54,0x56,0x5F,0xF5,0xBA,0xA4,0xFB,0x68,0x3A,0xFC,0x00}},
        {0xBD, 1, {0x00}},
        {0xCB, 5, {0x00,0x13,0x08,0x02,0x34}},
        {0xBD, 1, {0x01}},
        {0xB1, 4, {0x01,0x9B,0x01,0x31}},
        {0xCB, 10, {0xF4,0x36,0x12,0x16,0xC0,0x28,0x6C,0x85,0x3F,0x04}},
        {0xD3, 11, {0x01,0x00,0x3C,0x00,0x00,0x11,0x10,0x00,0x0E,0x00,0x01}},
        {REGFLAG_DELAY, 5, {}},
        {0xBD, 1, {0x02}},
        {0xB4, 6, {0x4E,0x00,0x33,0x11,0x33,0x88}},
        {0xBF, 3, {0xF2,0x00,0x02}},
        {0xBD, 1, {0x00}},
        {0xC0, 14, {0x23,0x23,0x22,0x11,0xA2,0x17,0x00,0x80,0x00,0x00,0x08,0x00,0x63,0x63}},
        {0xC6, 1, {0xF9}},
        {0xC7, 1, {0x30}},
        {0xC8, 8, {0x00,0x04,0x04,0x00,0x00,0x85,0x43,0xFF}},
        {0xD0, 3, {0x07,0x04,0x05}},
        {0xD5, 44, {0x21,0x20,0x21,0x20,0x25,0x24,0x25,0x24,0x18,0x18,0x18,0x18,0x1A,0x1A,0x1A,0x1A,0x1B,0x1B,0x1B,0x1B,0x03,0x02,0x03,0x02,0x01,0x00,0x01,0x00,0x07,0x06,0x07,0x06,0x05,0x04,0x05,0x04,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
        {REGFLAG_DELAY, 5, {}},
        {0xE7, 23, {0x12,0x13,0x02,0x02,0x57,0x57,0x0E,0x0E,0x1B,0x28,0x29,0x74,0x28,0x74,0x01,0x07,0x00,0x00,0x00,0x00,0x17,0x00,0x68}},
        {0xBD, 1, {0x01}},
        {0xE7, 7, {0x02,0x38,0x01,0x93,0x0D,0xD9,0x0E}},
        {0xBD, 1, {0x02}},
        {0xE7, 28, {0xFF,0x01,0xFF,0x01,0x00,0x00,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,0x00,0x02,0x40}},
        {0xBD, 1, {0x00}},
        {0xBA, 8, {0x70,0x03,0xA8,0x83,0xF2,0x00,0xC0,0x0D}},
        {0xBD, 1, {0x02}},
        {0xD8, 12, {0xAF,0xFF,0xFF,0xFF,0xF0,0x00,0xAF,0xFF,0xFF,0xFF,0xF0,0x00}},
        {0xBD, 1, {0x03}},
        {0xD8, 24, {0xAA,0xAA,0xAA,0xAA,0xA0,0x00,0xAA,0xAA,0xAA,0xAA,0xA0,0x00,0x55,0x55,0x55,0x55,0x50,0x00,0x55,0x55,0x55,0x55,0x50,0x00}},
        {0xBD, 1, {0x00}},
        {0xE1, 2, {0x01,0x05}},
        {0xCC, 1, {0x02}},
        {0xBD, 1, {0x03}},
        {0xB2, 1, {0x80}},
        {0xBD, 1, {0x00}},
        {0x35, 1, {0x00}},
        {0xB2, 16, {0x00,0xB0,0x47,0xD0,0x00,0x2C,0x50,0x2C,0x00,0x00,0x00,0x00,0x15,0x20,0x57,0x00}},
        /*+maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
        {0x35, 1, {0x00}},
        {0x51, 2, {0x00,0x00}},
        {0x53, 1, {0x2C}},
        {0xC9, 4, {0x00,0x0D,0xF0,0x00}},
        {0x5E, 2, {0x00,0x2E}},
        /*-maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
        {0x11, 0, {0x11}},
        {REGFLAG_DELAY, 60, {}}, //T7>10ms //OMAPLET-2 liuxueyou.wt add 20221226 adjust sequence to conform to the IC spec
        {0x29, 0, {0x29}},
        {REGFLAG_DELAY, 20, {}}, //T8>20ms
};

static void push_table(struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned int cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

/* --------------------------------------------------------------------- */
/*  LCM Driver Implementations */
/* --------------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH;   //mm
	params->physical_height = LCM_PHYSICAL_HEIGHT; //mm
	params->physical_width_um=LCM_PHYSICAL_WIDTH_UM;    //um
	params->physical_height_um=LCM_PHYSICAL_HEIGHT_UM;  //um

	params->dsi.mode = BURST_VDO_MODE;
//#if (LCM_DSI_CMD_MODE)
//	params->dsi.mode                    = CMD_MODE;
//#else
//	params->dsi.mode                    = SYNC_PULSE_VDO_MODE;
//#endif

	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size             = 256;

	params->dsi.PS                      = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active    = 8;
	params->dsi.vertical_backporch      = 38;
	params->dsi.vertical_frontporch     = 80;
	params->dsi.vertical_active_line    = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active  = 8;
	params->dsi.horizontal_backporch    = 28;
	params->dsi.horizontal_frontporch   = 42;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable             = 1; */
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK               = 525;
#else
	params->dsi.PLL_CLOCK               = 525;
#endif

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.cont_clock = 1;
	params->dsi.clk_lp_per_line_enable = 0;
}

static void lcm_init_power(void)
{
	printk("[Kernel/LCM] lcm_init_power()\n");
}

static void lcm_init_lcm(void)
{
	printk("[KERNEL/LCM] hx83102p lcm_init_lcm enter\n");
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(1);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	UDELAY(15); //T5>15us,keep least 15us;
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(50); //T6>50ms //OMAPLET-2 liuxueyou.wt add 20221226 adjust sequence to conform to the IC spec
	push_table(lcm_initinal_setting,sizeof(lcm_initinal_setting) / sizeof(struct LCM_setting_table),1);
	//MDELAY(100);
	
	/*+maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	sgm37604a_write_byte(0x11,0x25);
#if 0
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	sgm37604a_write_byte(0x11,0x05);
	sgm37604a_write_byte(0x1A,0);
	sgm37604a_write_byte(0x19,0);
#endif
	/*-maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
}

static void lcm_suspend(void)
{
	printk("[Kernel/LCM] hx83102p lcm_suspend() enter\n");
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);//灭背光
	if(!hx_gesture_flag) {
	printk("[Kernel/LCM] hx83102p lcm_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),1);
	}
	else {//hx_gesture_flag == 1
	printk("[Kernel/LCM] hx83102p lcm_gesture_suspend() enter\n");
	push_table(lcm_gesture_suspend_setting, sizeof(lcm_gesture_suspend_setting) / sizeof(struct LCM_setting_table),1);
	}
	//MDELAY(15);
}

static void lcm_suspend_power(void)
{
if(!hx_gesture_flag){
	printk("[Kernel/LCM] lcm_suspend_power()\n");
	lcm_set_gpio_output(GPIO_CTP_RST, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(3);

	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ZERO);
	MDELAY(3);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ZERO);
	
	//MDELAY(97); //power off to power on, at least 102ms, suspend code satisfied;
	}
}
static void lcm_resume_power(void)
{
	printk("[Kernel/LCM] lcm_resume_power()\n");

	if(strstr(saved_command_line, "AW37503")) {
		ocp2138_write_byte(0x03,0x43);//modify BAIS IC 2st VSN from 80mA to 150mA
	}

	/* set AVDD*/
	/*5.7V + 0.1* 100rmV*/
	ocp2138_write_byte(0x00,0x11);
	/* set AVEE */
	/*-5.70V - 0.1* 100mV*/
	ocp2138_write_byte(0x01,0x11);
	MDELAY(5);
	
	lcm_set_gpio_output(GPIO_CTP_RST, GPIO_OUT_ZERO);
	UDELAY(15); //keep least 15us
	lcm_set_gpio_output(GPIO_CTP_RST, GPIO_OUT_ONE);
	
	MDELAY(1); //T1>1ms

	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ONE);
	MDELAY(3); //T2>1ms //OMAPLET-2 liuxueyou.wt add 20221226 adjust sequence to conform to the IC spec
	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ONE);
	MDELAY(1); //T4>1ms
	lcm_init_power();
}

static void lcm_resume(void)
{
	lcm_init_lcm();
	printk("[Kernel/LCM] hx83102p lcm_resume() end\n");
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x00023700; /* read id return two byte, version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; /* we only need ID */
#ifdef BUILD_LK
	dprintf(0, "%s, [LK/LCM] hx83102p: id = 0x%08x\n", __func__, id);
#else
	printk("%s, [Kernel/LCM] hx83102p: id = 0x%08x\n", __func__, id);
#endif

	if (id == LCM_ID_NT35595)
		return 1;
	else
		return 0;
}

/*+maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
/*-maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	int sgm_level = 0;

	if (level > 255)
		level = 255;
	/*+maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	//sgm_level = backlight_i2c_map_hx[level];
	sgm_level = level*3200/255;
	printk("[Kernel/LCM] %s,hx83102p backlight: level = %d sgm_level = %d\n", __func__,level,sgm_level);
#if 0
	sgm37604a_write_byte(0x1A,(backlight_i2c_map_hx[level] & 0x0F));
	sgm37604a_write_byte(0x19,(backlight_i2c_map_hx[level] >> 4));
#endif
	bl_level[0].para_list[0]=((sgm_level>>8)&0xf);
	bl_level[0].para_list[1]=(sgm_level &0xff);
	push_table(bl_level, sizeof(bl_level)/sizeof(struct LCM_setting_table), 1);
	/*-maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	s_last_backlight_level = level;
	
}


static void hx83102p_backlight_recover()
{
	/*+maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
#if 0
	sgm37604a_write_byte(0x1A,(backlight_i2c_map_hx[s_last_backlight_level] & 0x0F));
	sgm37604a_write_byte(0x19,(backlight_i2c_map_hx[s_last_backlight_level] >> 4));
#endif
	int sgm_level;
	sgm_level = s_last_backlight_level*3200/255;
	bl_level[0].para_list[0]=((sgm_level>>8)&0xf);
	bl_level[0].para_list[1]=(sgm_level &0xff);
	push_table(bl_level, sizeof(bl_level)/sizeof(struct LCM_setting_table), 1);
	/*-maple-8397 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
}

unsigned int lcm_hx83102p_esd_recovery(void){
	printk("[Kernel/LCM] hx83102p lcm_recovery() enter\n");
	lcm_resume_power();
	lcm_resume();
	printk("[Kernel/LCM] set backlight_recover\n");
	hx83102p_backlight_recover();
	printk("[Kernel/LCM] hx83102p lcm_recovery() End\n");
	return 0;
}

struct LCM_DRIVER hx83102p_wxga_vdo_incell_boe_lcm_drv = {
	.name               = "hx83102p_wxga_vdo_incell_boe",
	.set_util_funcs     = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.init               = lcm_init_lcm,
	.resume             = lcm_resume,
	.suspend            = lcm_suspend,
	.init_power         = lcm_init_power,
	.resume_power       = lcm_resume_power,
	.suspend_power      = lcm_suspend_power,
	.compare_id         = lcm_compare_id,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.esd_recover = lcm_hx83102p_esd_recovery,
};

