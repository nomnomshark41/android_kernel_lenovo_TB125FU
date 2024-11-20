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
#include "backlight_map.h"
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
		.compatible = "mediatek,lcd_gpio3",
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
		.name = "nt36532_apcfwxga_vdo_incell_novatek",
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

static unsigned short s_last_backlight_level = 0;
//Gesture flag
extern volatile bool novatek_gesture_flag;

extern int nvt_lcm_ts_resume_func(void);

static void lcm_setbacklight_cmdq(void *handle, unsigned int level);

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
/* Disable command lock function */
	{0xF0, 1, {0xAA} },
	{0xF1, 1, {0x55} },
	{0xF2, 1, {0x99} },

	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0xB9, 1, {0x02} },
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 10, {} },
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 100, {} },
};

static struct LCM_setting_table lcm_initinal_setting[] = {
/*optimized MIPI eye map*/
	{0xFF, 1, {0x2A}},
	{0xFB, 1, {0x01}},
	{0xF1, 1, {0x03}},
	{0xFF, 1, {0xD0}},
	{0xFB, 1, {0x01}},
	{0x00, 1, {0x11}},
/*esd detect,the 0x0A register exception sets TE to low*/
	{0xFF, 1, {0x27}},
        {0xFB, 1, {0x01}},
        {0xD0, 1, {0x71}},
        {0xD1, 1, {0x84}},
        {0xD2, 1, {0x30}},
        {0xD4, 1, {0x10}},
        {0xDE, 1, {0x84}},//set TE to low if the screen is flowered

	{0xFF, 1, {0x26}},
        {0xFB, 1, {0x01}},
        {0x00, 1, {0x81}},
        {0x01, 1, {0x70}},
        {0x40, 1, {0x75}},
        {0x41, 1, {0x75}},
        {0x42, 1, {0x75}},
        {0x45, 1, {0x02}},
        {0x46, 1, {0x75}},
        {0x47, 1, {0x02}},
        {0x48, 1, {0x85}},
        {0x4D, 1, {0x4E}},
        {0x4E, 1, {0x1B}},
        {0x51, 1, {0x00}},
        {0x52, 1, {0x6A}},
        {0x56, 1, {0x00}},
        {0x58, 1, {0x6A}},
        {0x5B, 1, {0x00}},
        {0x5C, 1, {0x6A}},
        {0x64, 1, {0x00}},
        {0x65, 1, {0x6A}},
        {0x8C, 1, {0x7C}},
        {0x92, 1, {0x50}},
        {0x93, 1, {0x10}},
        {0x94, 1, {0x10}},
        {0x96, 1, {0x10}},
        {0xFF, 1, {0x25}},       
        {0xFB, 1, {0x01}},
        {0x15, 1, {0x02}},
        {0x16, 1, {0xF3}},
        {0xFF, 1, {0x20}},
        {0xFB, 1, {0x01}}, 
        {0x58, 1, {0x40}},
        {0x75, 1, {0xC4}},
        {0x8B, 1, {0x23}},
        {0x8C, 1, {0x23}},
        {0xFF, 1, {0x2A}},
        {0xFB, 1, {0x01}},
        //+ADD
        {0x01, 1, {0x01}},
        {0x02, 1, {0x11}},
        {0x05, 1, {0x01}},
        {0x06, 1, {0x75}},
        {0x08, 1, {0x01}},
        {0x09, 1, {0x75}},
        //+ADD
        {0x99, 1, {0x95}},
        {0x9A, 1, {0x03}},

	{0xFF, 1, {0xE0}},
        {0xFB, 1, {0x01}},
        {0x14, 1, {0x60}},
        {0x16, 1, {0xC0}},
        {0xFF, 1, {0xF0}},
        {0xFB, 1, {0x01}},
        {0x3A, 1, {0x08}},
        {0xFF, 1, {0x10}},
        {0xFB, 1, {0x01}},
        {0xB9, 1, {0x01}},
        {0xFF, 1, {0x20}},
        {0xFB, 1, {0x01}},
        {0x18, 1, {0x40}},
//        {0xFF, 1, {0x27}},
//        {0xFB, 1, {0x01}},   
//        {0xD1, 1, {0x94}},
//        {0xD2, 1, {0xA0}},  

        {0xFF, 1, {0x10}},
        {0xFB, 1, {0x01}},
        {0xB9, 1, {0x02}},
        {0xBB, 1, {0x13}},
        {0x3B, 5, {0x03,0x5F,0x1A,0x04,0x04}},
	/*+maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	/*pwm*/
		{0xFF, 1, {0x23}},
		{0xFB, 1, {0x01}},
		{0x00, 1, {0x80}},	 // pwm  12bit
		{0x07, 1, {0x00}},	 // PWM 23KHZ
		{0x08, 1, {0x01}},
		{0x09, 1, {0x00}},
	/*dimming*/
		{0xFF, 1, {0x10}},
		{0xFB, 1, {0x01}},
		{0x51, 2, {0x00,0x00}},
		{0x53, 1, {0x2c}},
		{0x68, 2, {0x03,0x01}},//maple-10242 liuxueyou.wt add 20221025 backlight blink when adjust seek bar
	/*-maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
        {0x35, 1, {0x00}}, 
        {0x11, 0, {0x00}},
        {REGFLAG_DELAY, 100, {}},
        {0x29, 0, {0x00}},
        {REGFLAG_DELAY, 10, {}},
/* Disable command lock function */
        {0xF0, 0, {0x55}},
        {0xF1, 0, {0xAA}},
        {0xF2, 0, {0x66}},
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

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode                    = CMD_MODE;
#else
	params->dsi.mode                    = SYNC_PULSE_VDO_MODE;
#endif

	params->dsi.LANE_NUM                = LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size             = 256;

	params->dsi.PS                      = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active    = 2;
	params->dsi.vertical_backporch      = 93;
	params->dsi.vertical_frontporch     = 26;
	params->dsi.vertical_active_line    = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active  = 6;
	params->dsi.horizontal_backporch    = 40;
	params->dsi.horizontal_frontporch   = 33;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_range = 1;
	params->dsi.ssc_disable             = 0;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK               = 500;
#else
	params->dsi.PLL_CLOCK               = 521;
#endif

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
}

static void lcm_init_power(void)
{
	printk("[Kernel/LCM] lcm_init_power()\n");
}

static void lcm_init_lcm(void)
{
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(15);
	nvt_lcm_ts_resume_func();

	push_table(lcm_initinal_setting,sizeof(lcm_initinal_setting) / sizeof(struct LCM_setting_table),1);
	MDELAY(40);
	/*+maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	sgm37604a_write_byte(0x11,0x25);
	//MDELAY(15);
	//lcm_setbacklight_cmdq(NULL,s_last_backlight_level);
#if 0
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	sgm37604a_write_byte(0x11,0x05);
	printk("[KERNEL/LCM] nt36532w lcm_init_lcm enter\n");
#endif
	/*-maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
}

static void lcm_suspend(void)
{
	printk("[Kernel/LCM] 3333nt36532w lcm_suspend() enter\n");
	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);//灭背光
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),1);
	MDELAY(15);
}

static void lcm_suspend_power(void)
{
	if(!novatek_gesture_flag) {
	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ZERO);
	MDELAY(8);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ZERO);

	printk("[Kernel/LCM] lcm_suspend_power()\n");
	}
}
static void lcm_resume_power(void)
{
	printk("[Kernel/LCM] lcm_resume_power()\n");
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);

	if(strstr(saved_command_line, "AW37503")) {
		ocp2138_write_byte(0x03,0x43);//modify BAIS IC 2st VSN from 80mA to 150mA
	}
	ocp2138_write_byte(0x00,0x11);//AVDD +5.7V
	ocp2138_write_byte(0x01,0x11);//AVEE -5.7V
	MDELAY(5);

	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ONE);
	MDELAY(8);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ONE);
	MDELAY(20);

	lcm_init_power();
}

static void lcm_resume(void)
{
	lcm_init_lcm();
	printk("[Kernel/LCM] nt36532w lcm_resume() enter\n");
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
	dprintf(0, "%s, [LK/LCM] nt36532w: id = 0x%08x\n", __func__, id);
#else
	printk("%s, [Kernel/LCM] nt36532w: id = 0x%08x\n", __func__, id);
#endif

	if (id == LCM_ID_NT35595)
		return 1;
	else
		return 0;
}

/*+maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
/*-maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	int sgm_level = 0;

	if (level > 255)
		level = 255;
	/*+maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	if(level < 4 && level > 0)
		level = 4;
	//sgm_level = backlight_i2c_map[level];
	sgm_level = level*2200/255;
	printk("[Kernel/LCM] %s,nt36532w backlight: level = %d sgm_level = %d\n", __func__,level,sgm_level);
#if 0
	sgm37604a_write_byte(0x1A,(backlight_map[level] & 0x0F));
	sgm37604a_write_byte(0x19,(backlight_map[level] >> 4));
#endif
	bl_level[0].para_list[0]=((sgm_level>>8)&0xf);
	bl_level[0].para_list[1]=(sgm_level &0xff);
	push_table(bl_level, sizeof(bl_level)/sizeof(struct LCM_setting_table), 1);
	/*-maple-10244 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
	s_last_backlight_level = level;
}

static void nt36532_backlight_recover()
{
	//lcm_setbacklight_cmdq(NULL,s_last_backlight_level);
	/*+maple-8401 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
#if 0
	sgm37604a_write_byte(0x1A,(backlight_i2c_map[s_last_backlight_level] & 0x0F));
	sgm37604a_write_byte(0x19,(backlight_i2c_map[s_last_backlight_level] >> 4));
#endif
	int sgm_level;
	sgm_level = s_last_backlight_level*3200/255;
	bl_level[0].para_list[0]=((sgm_level>>8)&0xf);
	bl_level[0].para_list[1]=(sgm_level &0xff);
	push_table(bl_level, sizeof(bl_level)/sizeof(struct LCM_setting_table), 1);
	/*-maple-8401 liuxueyou.wt add 20220921 change i2c mode to pwm mode to control backlight*/
}

extern int nvt_ts_suspend_func(void);
extern int nvt_ts_resume_func(void);

static unsigned int lcm_nt36532_esd_recovery(void){
	nvt_ts_suspend_func();
	lcm_resume_power();
	lcm_resume();
	nt36532_backlight_recover();
	nvt_ts_resume_func();
	printk("[Kernel/LCM] nt36532 lcm_recovery() End\n");

	return 0;
}

struct LCM_DRIVER nt36532w_apcf_wxga_vdo_incell_novatek_lcm_drv = {
	.name               = "nt36532w_apcf_wxga_vdo_incell_novatek",
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
	.esd_recover = lcm_nt36532_esd_recovery,
};

