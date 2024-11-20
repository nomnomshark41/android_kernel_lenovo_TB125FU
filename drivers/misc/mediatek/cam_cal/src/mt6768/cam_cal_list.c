/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

extern unsigned int zte_s5k4h7_read_region(struct i2c_client* client, unsigned int addr, unsigned char *data, unsigned int size);
extern unsigned int zte_s5k4h7_sub_read_region(struct i2c_client* client, unsigned int addr, unsigned char *data, unsigned int size);
extern unsigned int gc08a3_main_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size);
extern unsigned int gc08a3_sub_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size);

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	{GC08A3MIPI_SENSOR_ID, 0x22, gc08a3_main_read_region},
	{GC08A3SUBMIPI_SENSOR_ID, 0x22, gc08a3_sub_read_region},
	//+bug[MAPLE-506] zhouhongxiao.wt,add,2021/11/11,ov13b10 otp porting
	{OV13B10_SENSOR_ID , 0xb0, Common_read_region},
	{OV13B10TXD_SENSOR_ID, 0xb0, Common_read_region},
	//-bug[MAPLE-506] zhouhongxiao.wt,add,2021/11/11,ov13b10 otp porting
	{S5K4H7_SENSOR_ID, 0xA0, zte_s5k4h7_read_region},
	{S5K4H7SUB_SENSOR_ID, 0xA0, zte_s5k4h7_sub_read_region},
#if 0
	/*Below is commom sensor */
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*B+B. No Cal data for main2 OV8856*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
#ifdef SUPPORT_S5K4H7
	{S5K4H7_SENSOR_ID, 0xA0, zte_s5k4h7_read_region},
	{S5K4H7SUB_SENSOR_ID, 0xA0, zte_s5k4h7_sub_read_region},
#endif
#endif
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


