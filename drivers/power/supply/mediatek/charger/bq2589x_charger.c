/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <mt-plat/v1/charger_class.h>
#include <mt-plat/v1/mtk_charger.h>
#include <linux/delay.h>
#include "mtk_charger_intf.h"
#include "bq2589x_reg.h"
#include <linux/hardware_info.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_boot_common.h>
enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE = 0,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP, /*CDP for bq25890, Adapter for bq25892*/
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no {
	BQ25890 = 0x03,
	BQ25892 = 0x00,
	BQ25895 = 0x07,
};

#define BQ2589X_STATUS_PLUGIN			0x0001
#define BQ2589X_STATUS_PG				0x0002
#define	BQ2589X_STATUS_CHARGE_ENABLE	0x0004
#define BQ2589X_STATUS_FAULT			0x0008
#define BQ2589X_STATUS_VINDPM			0x00010
#define BQ2589X_STATUS_IINDPM			0x00020
#define BQ2589X_STATUS_EXIST			0x0100

#define BQ2589X_BOOST_CURRENT_LIMIT_UA	1200000

#ifdef WT_COMPILE_FACTORY_VERSION
#define DEFAULT_PRE_CURR 2500
#else
#define DEFAULT_PRE_CURR 800
#endif

struct bq2589x_config {
	int		default_target_vol;
	int		charge_voltage;
	int		term_current;
};


struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
	enum   bq2589x_part_no part_no;
	int    revision;

	unsigned int	status;
	int		vbus_type;
	int		pd_hub_active;

	struct wakeup_source* pe_tune_wakelock;
	struct wakeup_source* chg_curr_tune_wakelock;

	bool	enabled;
	bool	charge_enabled;	
	bool	suspend_flag;

	int		max_pe_vol;
	int		vbus_volt;
	int		vbat_volt;
	unsigned int pre_curr;
	unsigned int target_curr;
	unsigned int init_curr;

	int dpdm_sel;
	int shut_down;

	int		rsoc;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	const char *chg_dev_name;
	struct	bq2589x_config	cfg;
	struct work_struct irq_work;
	struct work_struct adapter_in_work;
	struct work_struct adapter_out_work;
	struct delayed_work monitor_work;
	struct delayed_work ico_work;
	struct delayed_work pe_volt_tune_work;
	struct delayed_work check_pe_tuneup_work;
	struct delayed_work chg_curr_tune_work;
};

struct pe_ctrl {
	bool tune_done;
	bool tune_rework;
	int  tune_count;
	int  target_volt;
};


static struct bq2589x *g_bq;
static struct pe_ctrl pe;

static DEFINE_MUTEX(bq2589x_i2c_lock);
static DEFINE_MUTEX(bq2589x_type_det_lock);

/* for mtk ops */
static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data);
static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data);
static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg);
static int bq2589x_plug_in(struct charger_device *chg_dev);
static int bq2589x_plug_out(struct charger_device *chg_dev);
static int bq2589x_dump_register(struct charger_device *chg_dev);
static int bq2589x_is_charging_enable(struct charger_device *chg_dev, bool *en);
static int bq2589x_get_ichg(struct charger_device *chg_dev, u32 *curr);
static int bq2589x_set_ichg(struct charger_device *chg_dev, u32 curr);
static int bq2589x_get_icl(struct charger_device *chg_dev, u32 *curr);
//static int bq2589x_set_icl(struct charger_device *chg_dev, u32 curr);
static int bq2589x_get_vchg(struct charger_device *chg_dev, u32 *volt);
static int bq2589x_set_vchg(struct charger_device *chg_dev, u32 volt);
static int bq2589x_kick_wdt(struct charger_device *chg_dev);
static int bq2589x_set_ivl(struct charger_device *chg_dev, u32 volt);
static int bq2589x_is_charging_done(struct charger_device *chg_dev, bool *done);
static int bq2589x_get_min_ichg(struct charger_device *chg_dev, u32 *curr);
static int bq2589x_set_safety_timer(struct charger_device *chg_dev, bool en);
static int bq2589x_is_safety_timer_enabled(struct charger_device *chg_dev, bool *en);
static int bq2589x_set_hz_mode(struct charger_device *chg_dev, bool en);
static int bq2589x_do_event(struct charger_device *chg_dev, u32 event, u32 args);

/* ops function */
static int bq2589x_enable_charging(struct charger_device *chg_dev, bool enable);
static void bq2589x_dump_regs(struct bq2589x *bq);
int bq2589x_adc_read_charge_current(struct bq2589x *bq);
int bq2589x_set_chargecurrent(struct bq2589x *bq, u32 curr);
int bq2589x_get_input_current_limit(struct bq2589x *bq, int *curr);
int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr);
int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt);
int bq2589x_reset_watchdog_timer(struct bq2589x *bq);
int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt);
static bool bq2589x_is_charge_done(struct bq2589x *bq);
int bq2589x_enter_hiz_mode(struct bq2589x *bq);
int bq2589x_exit_hiz_mode(struct bq2589x *bq);
static int bq2589x_enable_charger(struct bq2589x *bq);
static int bq2589x_disable_charger(struct bq2589x *bq);

static void bq2589x_reset_pe_param(void)
{
	pe.tune_done = true;
	pe.tune_rework = false;
	pe.tune_count = 0;
	pe.target_volt = g_bq->cfg.default_target_vol;
}

int bq2589x_irq;
static int bq2589x_plug_in(struct charger_device *chg_dev)
{
	int ret;
	ret = bq2589x_enable_charging(chg_dev, true);
	if(ret < 0) 
		pr_err("Failed to enable charging:%d\n", ret);
	return ret;
}

static int bq2589x_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = bq2589x_set_hz_mode(chg_dev, false);
	if(ret < 0) {
		pr_err("Failed to disable hz mode ret:%d\n", ret);
		return ret;
	}
	ret = bq2589x_enable_charging(chg_dev, false);
	if(ret < 0)
		pr_err("Failed to disable charging:%d\n", ret);
	return ret;
}

static int bq2589x_dump_register(struct charger_device *chg_dev)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	bq2589x_dump_regs(bq);
	return 0;
}

static int bq2589x_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	
	*en = bq->charge_enabled;
	
	return 0;
}

static int bq2589x_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	
	*curr = bq2589x_adc_read_charge_current(bq) * 1000;
	
	return 0;
}

static int bq2589x_set_icl(struct charger_device *chg_dev, u32 curr)
{
	int ret;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	curr = curr/1000;
	ret = bq2589x_set_input_current_limit(bq, curr);
	
	if (ret < 0) 
		dev_err(bq->dev, "%s:Failed to set input current:%d\n", __func__, ret);
	return 0;
}

static int bq2589x_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	int ret;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	ret = bq2589x_get_input_current_limit(bq, curr);
	
	if (ret < 0) 
		dev_err(bq->dev, "%s:Failed to get input current limit:%d\n", __func__, ret);
	
	return 0;
}

static int bq2589x_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	int ret;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	curr = curr/1000;
	ret = bq2589x_set_chargecurrent(bq, curr);
	
	if (ret < 0) 
		dev_err(bq->dev, "%s:Failed to set charge current limit:%d\n", __func__, ret);
	return 0;
}

static int bq2589x_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;
	
	ret = bq2589x_read_byte(bq, &reg_val, BQ2589X_REG_06);
	if(!ret)
	{
		vchg = (reg_val & BQ2589X_VREG_MASK) >> BQ2589X_VREG_SHIFT;
		vchg = vchg * BQ2589X_VREG_LSB + BQ2589X_VREG_BASE;
		*volt = vchg * 1000;
	}
	return ret;
}
static int bq2589x_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	int ret;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	
	volt = volt/1000;
	ret = bq2589x_set_chargevoltage(bq, volt);
	if (ret < 0) 
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
	
	return ret;
}
static int bq2589x_kick_wdt(struct charger_device *chg_dev)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	return bq2589x_reset_watchdog_timer(bq);
}

static int bq2589x_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	
	pr_err("limit volt = %d\n", volt);
	
	return bq2589x_set_input_volt_limit(bq, volt/1000);
}

static int bq2589x_is_tune_done(struct charger_device *chg_dev, bool *tune_done)
{
	*tune_done = pe.tune_done;

	return 0;
}

static int bq2589x_current_tune_done(struct charger_device *chg_dev, bool *tune_done)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);

	*tune_done = (bq->pre_curr == bq->target_curr);

	return 0;
}

static int bq2589x_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	*done = bq2589x_is_charge_done(bq);
	return 0;
}

static int bq2589x_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	*curr  = 60 * 1000;
	return 0;
}
static int bq2589x_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;
	
	if(en)
	{
		val = BQ2589X_CHG_TIMER_ENABLE << BQ2589X_EN_TIMER_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TIMER_MASK, val);
	}
	else
	{
		val = BQ2589X_CHG_TIMER_DISABLE << BQ2589X_EN_TIMER_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TIMER_MASK, val);
	}
	return ret;
}

static int bq2589x_is_safety_timer_enabled(struct charger_device *chg_dev, bool *en)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;
	
	ret = bq2589x_read_byte(bq, &reg_val, BQ2589X_REG_07);
	
	if(!ret)
		*en = !!(reg_val & BQ2589X_EN_TIMER_MASK);
	
	return ret;
}
static int bq2589x_set_hz_mode(struct charger_device *chg_dev, bool en)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	if(en)
	{
		bq2589x_enter_hiz_mode(bq);
	}
	else
	{
		bq2589x_exit_hiz_mode(bq);
	}

	pr_err("%s: hz_mode: %d\n", __func__, en);
	return 0;
}

static int bq2589x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);

	switch(event)
	{
		case EVENT_EOC:
			charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
			break;
		case EVENT_RECHARGE:
			bq2589x_set_chargecurrent(bq, 0);
			charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
			break;
		default:
			break;
	}
	return 0;
}
static int bq2589x_enable_charging(struct charger_device *chg_dev, bool enable)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;
	
	if(enable) {
		ret = bq2589x_enable_charger(bq);
	} else {
		bq2589x_set_chargecurrent(bq, 0);
		bq2589x_set_input_volt_limit(bq, 4600);
		ret = bq2589x_disable_charger(bq);
	}
	
	pr_err("%s: %s charger %s \n", __func__,
		enable ? "enable":"disable" , !ret ? "successfully":"failed");
	
	if (!ret)
		bq->charge_enabled = !!(val & BQ2589X_CHG_CONFIG_MASK);
	return ret;
}

static void bq2589x_dump_regs(struct bq2589x *bq)
{
	int addr;
	u8 val;
	int ret;

	return; //disable dump regs	
	for(addr = 0x0; addr <= 0x14; addr++)
	{
		msleep(2);
		ret = bq2589x_read_byte(bq, &val, addr);
		if(!ret)
			pr_err("bq2589x_dump_regs Reg[%.2x] = 0x%.2x\n", addr, val);
	}
}

int bq2589x_get_input_current_limit(struct bq2589x *bq, int *curr)
{
	u8 reg_val;
	int icl;
	int ret;
	
	ret = bq2589x_read_byte(bq, &reg_val ,BQ2589X_REG_00);
	if(!ret)
	{
		icl = (reg_val & BQ2589X_IINLIM_MASK) >> BQ2589X_IINLIM_SHIFT;
		icl = icl * BQ2589X_IINLIM_LSB + BQ2589X_IINLIM_BASE;
		*curr = icl * 1000;
	}
	return ret;
}

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

	if (bq->suspend_flag)
		return -EBUSY;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2589x_i2c_lock);

	return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->suspend_flag)
		return -EBUSY;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq2589x_i2c_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2589x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, tmp);
}


static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}


static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
							   BQ2589X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt)
{
	u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);

static int bq2589x_set_otg_current(struct charger_device *chg_dev, unsigned int uA)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 temp;

	pr_err("set otg current %dmA\n", uA/1000);
	if (uA > BQ2589X_BOOST_CURRENT_LIMIT_UA) {
		uA = BQ2589X_BOOST_CURRENT_LIMIT_UA;
		pr_err("limit otg current to %dmA\n", uA/1000);
	}
	if (uA <= 500000)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (uA <= 750000)
		temp = BQ2589X_BOOST_LIM_750MA;
	else if (uA <= 1200000)
		temp = BQ2589X_BOOST_LIM_1200MA;
	else if (uA <= 1650000)
		temp = BQ2589X_BOOST_LIM_1650MA;
	else if (uA <= 1875000)
		temp = BQ2589X_BOOST_LIM_1875MA;
	else if (uA <= 2150000)
		temp = BQ2589X_BOOST_LIM_2150MA;
	else
		temp = BQ2589X_BOOST_LIM_2450MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);

static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status |= BQ2589X_STATUS_CHARGE_ENABLE;
	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status &= ~BQ2589X_STATUS_CHARGE_ENABLE;
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);


/* interfaces that can be called by other module */
int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,  BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);

int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);


int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
	uint8_t val;
	int temp;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return ret;
	} else{
		temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		return temp;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_set_chargecurrent(struct bq2589x *bq, u32 curr)
{
	bq->target_curr = curr;
	cancel_delayed_work_sync(&bq->chg_curr_tune_work);
	if (bq->pre_curr != bq->target_curr) {
		if (!bq->chg_curr_tune_wakelock->active)
			__pm_stay_awake(bq->chg_curr_tune_wakelock);
		schedule_delayed_work(&bq->chg_curr_tune_work, 0);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);


int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;
	static int pre_cv = -1;

	if (pre_cv == volt) {
		dev_info(bq->dev, "[%s] same cv set(%dmV)\n", __func__, volt);
		return 0;
	}

	if (bq->vbat_volt>4080 && volt<=4200) {
		dev_err(bq->dev, "[%s] vbat(%dmV) too high to decrease cv(%dmV)\n", __func__,
				bq->vbat_volt, volt);
		return 0;
	}
	if (pre_cv < volt) {
		bq2589x_set_chargecurrent(bq, 0);
	}
	pre_cv = volt;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);


int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;
	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	if (curr < BQ2589X_IINLIM_BASE)
		curr = BQ2589X_IINLIM_BASE;
	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

int bq2589x_set_battery_comp(struct bq2589x *bq, int mohm)//ir
{
	u8 val;

	val = (mohm - BQ2589X_BAT_COMP_BASE)/BQ2589X_BAT_COMP_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_08, BQ2589X_BAT_COMP_MASK, val << BQ2589X_BAT_COMP_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_battery_comp);

int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	val = (offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

int bq2589x_set_otg(struct charger_device *chg_dev, bool enable)
{
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	pr_err("set otg %d\n", enable);
	if (enable) {
		ret = bq2589x_enable_otg(bq);
		if (ret < 0) {
			dev_err(bq->dev, "%s:Failed to enable otg-%d\n", __func__, ret);
			return ret;
		}
	} else{
		ret = bq2589x_disable_otg(bq);
		if (ret < 0){
			dev_err(bq->dev, "%s:Failed to disable otg-%d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);

int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, (u8)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret, timeout = 50;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if (ret)
		return ret;

	while (timeout >= 0) {
		ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
		if (ret)
			return ret;
		if (!(val&BQ2589X_FORCE_DPDM_MASK))
			break;
		msleep(20);
		timeout--;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);

int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	bq2589x_set_ichg(bq->chg_dev, 0);

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);


int bq2589x_pumpx_enable(struct bq2589x *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_force_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(bq2589x_check_force_ico_done);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_use_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_use_absolute_vindpm);

static int bq2589x_enable_ico(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ico);


static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

static bool bq2589x_is_charge_done(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);

static int bq2589x_enable_shipping_mode(struct charger_device *chg_dev, bool val)
{
	int ret;
	u8 temp;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);

	dev_err(bq->dev, "%s: enable(%d) shipping mode,dump registers\n", __func__, val);
	bq2589x_dump_regs(bq);
	if(val){
		temp = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;
		ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, temp);
	}
	return ret;
}

static int bq2589x_get_non_standard_type(struct bq2589x *bq)
{
	int ret;
	u8 reg_val;

	ret = bq2589x_read_byte(bq, &reg_val, BQ2589X_REG_00);
	if (ret)
		return CHARGER_NS_1A;
	switch ((reg_val & BQ2589X_IINLIM_MASK)>>BQ2589X_IINLIM_SHIFT) {
		case 18://1A
			ret = CHARGER_NS_1A;
			break;
		case 38://2A
			ret = CHARGER_NS_2A;
			break;
		case 40://2.1A
			ret = CHARGER_NS_2_1A;
			break;
		case 46://2.4A
			ret = CHARGER_NS_2_4A;
			break;
		default:
			ret = CHARGER_NS_1A;
			break;
	}

	return ret;
}

void usb_device_set_pd_hub_state(int enable)
{
	if (g_bq !=NULL) {
		g_bq->pd_hub_active = enable;
	}
}

extern int battery_get_boot_mode(void);
static int bq2589x_update_chg_type(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct bq2589x *bq = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval propval;
	struct power_supply *chrdet_psy;
	enum charger_type chg_type = CHARGER_UNKNOWN;
	int wait_cdp_cnt = 250, wait_plugin_cnt = 20;
	int wait_hub_cnt = 0;
	static bool first_connect = true;
	int boot_mode = battery_get_boot_mode();

	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_notice("[%s]: get power supply failed\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&bq2589x_type_det_lock);
	if (first_connect
			&& boot_mode != KERNEL_POWER_OFF_CHARGING_BOOT
			&& boot_mode != LOW_POWER_OFF_CHARGING_BOOT) {
		while (wait_cdp_cnt >= 0) {
			if (is_usb_rdy()) {
				pr_info("CDP, PASS\n");
				break;
			}
			pr_info("CDP, block\n");
			msleep(100);
			wait_cdp_cnt--;
		}
		if (wait_cdp_cnt <= 0) {
			pr_info("CDP, timeout\n");
		} else {
			pr_info("CDP, free\n");
		}
		first_connect = false;
	}
	if (en) {
		Charger_Detect_Init();
		gpio_direction_output(bq->dpdm_sel, 1);
		while (wait_plugin_cnt>0
			&& (bq->status&BQ2589X_STATUS_PLUGIN)==0) {
			pr_info("[%s] power not stable yet, waiting[%dms...]\n",
				__func__, (20-wait_plugin_cnt)*100);
			msleep(100);
			wait_plugin_cnt--;
		}
		msleep(200);//wait ic stable
		bq2589x_force_dpdm(bq);
		bq->vbus_type = bq2589x_get_vbus_type(bq);
		switch ((int)bq->vbus_type) {
			case BQ2589X_VBUS_USB_SDP:
				chg_type =STANDARD_HOST ;
				break;
			case BQ2589X_VBUS_USB_CDP:
				chg_type = CHARGING_HOST;
				break;
			case BQ2589X_VBUS_USB_DCP:
				chg_type = STANDARD_CHARGER;
				break;
			case BQ2589X_VBUS_NONSTAND:
				chg_type = bq2589x_get_non_standard_type(bq);
				break;
			case BQ2589X_VBUS_UNKNOWN:
				chg_type = CHARGER_FLOATING;
				break;
			case BQ2589X_VBUS_MAXC:
			case BQ2589X_VBUS_OTG:
			default:
				chg_type = NONSTANDARD_CHARGER;
				break;
		}

		gpio_direction_output(bq->dpdm_sel, 0);
		Charger_Detect_Release();
		if (chg_type == STANDARD_CHARGER) {
			if (!bq->pe_tune_wakelock->active)
				__pm_stay_awake(bq->pe_tune_wakelock);
			schedule_delayed_work(&bq->check_pe_tuneup_work, 0);
		}

		pr_info("[%s] en:%d vbus_type:%d chg_type:%d, hub_active:%d\n", __func__, en,
				bq->vbus_type, chg_type, bq->pd_hub_active);
		ret = power_supply_get_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0) {
			pr_err("[%s]: get charge type failed, ret = %d\n", __func__, ret);
			mutex_unlock(&bq2589x_type_det_lock);
			return ret;
		}
	} else {
		propval.intval = CHARGER_UNKNOWN;
		pr_info("[%s] en:%d\n", __func__, en);
	}
	if (!en || (propval.intval!=CHARGER_PD_5V
		&& propval.intval!=CHARGER_PD_9V
		&& propval.intval!=CHARGER_PE_12V
		&& propval.intval!=CHARGER_PE_9V
		&& propval.intval!=CHARGER_PE_7V
		&& propval.intval!=CHARGER_PE_5V)) {
		if (chg_type == CHARGER_FLOATING) {
			while (wait_hub_cnt++ < 10) {
				if (bq->pd_hub_active == true) {
					pr_info("[%s]:force change charge type: CHARGER_PD_5V\n", __func__);
					chg_type = CHARGER_PD_5V;
					break;
				}
				pr_info("[%s] pd_hub not ready, waiting[%dms...]\n",
							__func__, wait_hub_cnt * 50);
				msleep(50);
			}
		}

		propval.intval = chg_type;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0) {
			pr_err("[%s]: set charge type failed, ret = %d\n", __func__, ret);
			mutex_unlock(&bq2589x_type_det_lock);
			return ret;
		}
	}
	mutex_unlock(&bq2589x_type_det_lock);
	return 0;
}

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	ret = bq2589x_disable_watchdog_timer(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable wdt:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_enable_auto_dpdm(bq, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable auto dpdm:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_enable_term(bq, true);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable term:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_enable_ico(bq, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable ico:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_use_absolute_vindpm(bq, true);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable absolute vindpm:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_set_safety_timer(bq->chg_dev, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to disable safety timer:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK,
			10<<BQ2589X_BOOSTV_SHIFT);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set boost voltage:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_set_otg_current(bq->chg_dev, 1200000);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set boost current:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_adc_start(bq, false);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to start adc conv:%d\n", __func__, ret);
		return ret;
	}
	ret = bq2589x_pumpx_enable(bq, 1);
	if (ret) {
		dev_err(bq->dev, "%s:Failed to enable pumpx:%d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	if(of_property_read_string(np, "charger_name", &bq->chg_dev_name) < 0){
		bq->chg_dev_name = "primary_chg";
		dev_err(bq->dev,"%s:failed to read charger name\n",__func__);
	}
	ret = of_property_read_u32(np, "ti,bq2589x,default-target-vol",
			&bq->cfg.default_target_vol);
	if (ret) {
		dev_err(bq->dev, "%s:Fail to get 'ti,bq2589x,default-target-vol', default 5000\n",
				__func__, ret);
		bq->cfg.default_target_vol = 5000;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",&bq->cfg.charge_voltage);
	if (ret) {
		dev_err(bq->dev, "%s:Fail to get 'ti,bq2589x,charge-voltage', default 4432\n",
				__func__, ret);
		bq->cfg.charge_voltage = 4432;
	}
	ret = of_property_read_u32(np, "ti,bq2589x,term-current",&bq->cfg.term_current);
	if (ret) {
		dev_err(bq->dev, "%s:Fail to get 'ti,bq2589x,term-current', default 256\n",
				__func__, ret);
		bq->cfg.term_current = 256;
	}

	return 0;
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
		if (bq->part_no == 1) {
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_INFO, "SY6970");
		} else if (bq->part_no == 3) {
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_INFO, "BQ25890");
		} else {
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_INFO, "UNKNOWN");
		}
	}

	return ret;
}

static int bq2589x_read_batt_curr(struct bq2589x *bq)
{
	struct power_supply *batt_psy;
	union power_supply_propval propval = {0,};
	int ret;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("[%s]: get power supply failed\n", __func__);
		return -EINVAL;
	}

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &propval);
	if (ret) {
		pr_err("[%s]: get current_avg failed\n", __func__);
		return ret;
	}
	return propval.intval;
}

static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	struct power_supply *batt_psy;
	union power_supply_propval propval = {0,};
	int ret;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("[%s]: get power supply failed\n", __func__);
		return -EINVAL;
	}

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &propval);
	if (ret) {
		pr_err("[%s]: get capacity failed\n", __func__);
		return ret;
	}
	return propval.intval;
}

static void bq2589x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_in_work);

	schedule_delayed_work(&bq->monitor_work, 0);
}

static void bq2589x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_out_work);
	int ret;

	ret = bq2589x_set_input_volt_limit(bq, 4400);
	if (ret < 0)
		dev_err(bq->dev,"%s:reset vindpm threshold to 4400 failed:%d\n",__func__,ret);
	else
		dev_info(bq->dev,"%s:reset vindpm threshold to 4400 successfully\n",__func__);

	if (bq->shut_down == true) {
		return;
	}

	usb_device_set_pd_hub_state(false);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	bq2589x_reset_pe_param();
	bq->max_pe_vol = 5000;
	__pm_relax(bq->pe_tune_wakelock);
}

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	int ret;
	int idpm;
	u8 status;
	static bool ico_issued;

	if (!ico_issued) {
		ret = bq2589x_force_ico(bq);
		if (ret < 0) {
			schedule_delayed_work(&bq->ico_work, HZ); /* retry 1 second later*/
			dev_info(bq->dev, "%s:ICO command issued failed:%d\n", __func__, ret);
		} else {
			ico_issued = true;
			schedule_delayed_work(&bq->ico_work, 3 * HZ);
			dev_info(bq->dev, "%s:ICO command issued successfully\n", __func__);
		}
	} else {
		ico_issued = false;
		ret = bq2589x_check_force_ico_done(bq);
		if (ret) {/*ico done*/
			ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
			if (ret == 0) {
				idpm = ((status & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
				dev_err(bq->dev, "%s:ICO done, result is:%d mA\n", __func__, idpm);
			}
		}
	}
}

static void bq2589x_check_pe_tuneup_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, check_pe_tuneup_work.work);

	if (!pe.tune_done) {//make sure pe check work stopped and adapter release
		pe.tune_rework = true;
		msleep(10*1000);
	}
	bq2589x_reset_pe_param();
	pe.tune_done = false;

	bq->init_curr = 800; //set init ibat 800mA
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	bq->rsoc = bq2589x_read_batt_rsoc(bq); 

	dev_info(bq->dev, "%s:vbat = %d, bq->rsoc = %d\n", __func__, bq->vbat_volt,
			bq->rsoc);
	schedule_delayed_work(&bq->pe_volt_tune_work, 1*HZ);
}

static void bq2589x_report_fchg_type(struct bq2589x *bq)
{
	struct power_supply *chrdet_psy;
	union power_supply_propval propval;
	int ret;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (bq->vbus_volt < 4000) {
		dev_err(bq->dev, "[%s] vbus %dmV plugg out!\n", __func__, bq->vbus_volt);
		pe.tune_done = true;
		__pm_relax(bq->pe_tune_wakelock);
		return;
	}
//+Bug9450, yangpingao.wt, modify, add PE 7V support
#if 0
	else if (bq->vbus_volt>=6500 && bq->vbus_volt<8100) {//6.5~8.1=>PE 7V
		dev_err(bq->dev, "[%s] vbus %dmV reset to 5V!\n", __func__, bq->vbus_volt);
		bq2589x_reset_pe_param();
		pe.tune_done = false;
		pe.target_volt = 5000;
		if (bq->max_pe_vol < 7000)
			bq->max_pe_vol = 7000;
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
		return;
	}
#endif
//+Bug9450, yangpingao.wt, modify, add PE 7V support
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy){
		dev_err(bq->dev, "[%s] fail to get psy 'charger'!\n", __func__);
		pe.tune_done = true;
		__pm_relax(bq->pe_tune_wakelock);
		return;
	}

	ret = power_supply_get_property(chrdet_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret) {
		dev_err(bq->dev, "[%s] fail to get chg_type from 'charger'!\n", __func__);
		pe.tune_done = true;
		__pm_relax(bq->pe_tune_wakelock);
		return;
	}
	dev_info(bq->dev, "%s:charge_type:%d, vbus:%d\n", __func__,
			propval.intval, bq->vbus_volt);

	if (bq->vbus_volt<6100 && propval.intval!=CHARGER_PD_5V && bq->max_pe_vol > 5000) {
		propval.intval = CHARGER_PE_5V;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	} else if (bq->vbus_volt >= 6100 && bq->vbus_volt < 8100) {
			propval.intval = CHARGER_PE_7V;
			if (bq->max_pe_vol < 7000)
				bq->max_pe_vol = 7000;
			ret = power_supply_set_property(chrdet_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	} else if (bq->vbus_volt>=8100 && bq->vbus_volt<10500) {
		if (propval.intval != CHARGER_PD_9V) {
			propval.intval = CHARGER_PE_9V;
			if (bq->max_pe_vol < 9000)
				bq->max_pe_vol = 9000;
			ret = power_supply_set_property(chrdet_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		}
	} else if (bq->vbus_volt>=10500) {
		propval.intval = CHARGER_PE_12V;
		if (bq->max_pe_vol < 12000)
			bq->max_pe_vol = 12000;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	}

	pe.tune_done = true;
	__pm_relax(bq->pe_tune_wakelock);
    return;
}

static void bq2589x_tune_volt_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pe_volt_tune_work.work);
	int timeout;
	int vbus_before_volt;
	int vbus_voltage;

	bq2589x_set_input_current_limit(bq, 100);
	msleep(300);
	bq2589x_set_input_current_limit(bq, 700);
	msleep(700);

	while ((pe.tune_count <= 10) && (bq->shut_down != true)) {
		if (!(bq->status&BQ2589X_STATUS_PLUGIN)) {
			dev_info(bq->dev, "%s:giveup, plugout!\n", __func__);
			__pm_relax(bq->pe_tune_wakelock);
			return;
		}

		if (pe.tune_rework) {
			dev_info(bq->dev, "%s:giveup, rework!\n", __func__);
			return;
		}

		//timeout = 2200;
		vbus_before_volt = bq2589x_adc_read_vbus_volt(bq);
		dev_info(bq->dev, "%s:count:%d, vbus:%d,target:%d\n", __func__, pe.tune_count,
				vbus_before_volt, pe.target_volt);

		if (vbus_before_volt < pe.target_volt - 500) {
			dev_err(bq->dev, "%s:pumpx_increase_volt\n", __func__);
			bq2589x_pumpx_increase_volt(bq);
		#if 0
			while (timeout > 0) {
				timeout -= 20;
				msleep(20);
				if (!bq2589x_pumpx_increase_volt_done(bq))
					break;
			}
		#endif
		} else if (vbus_before_volt > pe.target_volt + 500) {
			dev_err(bq->dev, "%s:pumpx_decrease_volt\n", __func__);
			bq2589x_pumpx_decrease_volt(bq);
		#if 0
			while (timeout > 0) {
				timeout -= 20;
				msleep(20);
				if (!bq2589x_pumpx_decrease_volt_done(bq))
					break;
			}
		#endif
		} else {
			dev_err(bq->dev, "%s:voltage tune successfully\n", __func__);
			bq2589x_report_fchg_type(bq);
			return;
		}

		msleep(3850);
		timeout = 20;
		while (timeout > 0) {
			msleep(5);
			timeout -= 5;
			vbus_voltage = bq2589x_adc_read_vbus_volt(bq);
			if (abs(vbus_voltage - vbus_before_volt) > 1500) {
				if (abs(vbus_voltage - 7000) <= 1000) {
					bq->init_curr = 1200; //set init ibat 1000mA
					bq2589x_set_chargecurrent(bq, 1200);
				} else if (abs(vbus_voltage - 9000) <= 1000) {
					bq->init_curr = 1600; //set init ibat 1400mA
					bq2589x_set_chargecurrent(bq, 1600);
				}
				break;
			}
		}

		bq->vbus_volt = vbus_before_volt;
		dev_info(bq->dev, "%s:after tune, vbus:%dmv=>%dmv, timeout: %d\n", __func__,
				vbus_before_volt, vbus_voltage, timeout);

		if (pe.tune_count >= 10) {
			dev_info(bq->dev, "%s:voltage tune failed,reach max retry count\n", __func__);
			bq2589x_report_fchg_type(bq);
			return;
		}

		pe.tune_count++;
	}
}

static void bq2589x_chg_curr_tune_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, chg_curr_tune_work.work);
	unsigned char ichg;
	int ret;
	unsigned int curr;

	if (!pe.tune_done) {
		curr = bq->init_curr;
		if (bq->target_curr < bq->init_curr)
			curr = bq->target_curr;
	} else {
		if (bq->pre_curr < bq->target_curr) {
			if (bq->pre_curr < bq->init_curr && bq->target_curr >= bq->init_curr) {
				curr = bq->init_curr;
			} else {
				curr = bq->pre_curr + 200;
				if (curr > bq->target_curr)
					curr = bq->target_curr;
			}
		} else if (bq->pre_curr > bq->target_curr) {
			curr = bq->target_curr;
		} else {
			__pm_relax(bq->chg_curr_tune_wakelock);
			return;
		}
	}
	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);
	if (!ret) {
		bq->pre_curr = curr;
	}
	pr_err("[%s] pre:%dmA target:%dmA vol_tune:%d\n", __func__, bq->pre_curr,
			bq->target_curr, pe.tune_done);
	if (bq->pre_curr != bq->target_curr) {
		schedule_delayed_work(&bq->chg_curr_tune_work, 5*HZ);
	} else {
		__pm_relax(bq->chg_curr_tune_wakelock);
	}
}

static void bq2589x_monitor_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, monitor_work.work);
	u8 status = 0;
	int ret;
	unsigned char hiz_en;
	int chg_current, bat_current, chg_type;
	union power_supply_propval propval;
	struct power_supply *chrdet_psy;

	if ((bq->status & BQ2589X_STATUS_PLUGIN) == 0)
		return;

	if (bq->suspend_flag) {
		pr_notice("[%s]: suspend, bypass\n", __func__);
		goto out;
	}

	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		pr_notice("[%s]: get power supply failed\n", __func__);
		goto out;
	}

	ret = power_supply_get_property(chrdet_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0) {
		pr_err("[%s]: get charge type failed, ret = %d\n", __func__, ret);
		goto out;
	}
	chg_type = propval.intval;
	bq->rsoc = bq2589x_read_batt_rsoc(bq);
	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	chg_current = bq2589x_adc_read_charge_current(bq);
	bat_current = bq2589x_read_batt_curr(bq)/1000;
	bq2589x_get_hiz_mode(bq, &hiz_en);

	dev_info(bq->dev, "%s:vbus volt:%d,vbat volt:%d,charge current:%d "
			"battery current:%d rsoc:%d chg_type:%d hiz:%d\n", __func__,
			bq->vbus_volt, bq->vbat_volt, chg_current, bat_current,
			bq->rsoc, chg_type, hiz_en);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
	if (!ret) {
		if (status & BQ2589X_VDPM_STAT_MASK) {
			bq->status |= BQ2589X_STATUS_VINDPM;
			dev_info(bq->dev, "%s:VINDPM occurred\n", __func__);
		} else {
			bq->status &= ~BQ2589X_STATUS_VINDPM;
		}
		if (status & BQ2589X_IDPM_STAT_MASK) {
			bq->status |= BQ2589X_STATUS_IINDPM;
			dev_info(bq->dev, "%s:IINDPM occurred\n", __func__);
		} else {
			bq->status &= ~BQ2589X_STATUS_IINDPM;
		}
	}

	if (!pe.tune_done || (bq->pre_curr!=bq->target_curr) || bq->vbus_volt<0)
		goto out;
	if ((bq->vbus_volt<5500 || (bq->rsoc>85 && bat_current<1000))
			&& (chg_type==CHARGER_PE_7V || chg_type==CHARGER_PE_9V
					|| chg_type==CHARGER_PE_12V)) {
		bq2589x_reset_pe_param();
		pe.tune_done = false;
		pe.target_volt = 5000;
		if (!bq->pe_tune_wakelock->active)
			__pm_stay_awake(bq->pe_tune_wakelock);
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	} else if (bq->vbus_volt>14000 && chg_type==CHARGER_PE_12V) {
		bq2589x_reset_pe_param();
		pe.tune_done = false;
		pe.target_volt = 9000;
		if (!bq->pe_tune_wakelock->active)
			__pm_stay_awake(bq->pe_tune_wakelock);
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	} else if (bat_current>1500 && chg_type==CHARGER_PE_5V && bq->max_pe_vol>7000) {
		bq2589x_reset_pe_param();
		pe.tune_done = false;
		if (!bq->pe_tune_wakelock->active)
			__pm_stay_awake(bq->pe_tune_wakelock);
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	}

out:
	schedule_delayed_work(&bq->monitor_work, 10 * HZ);
}

static void bq2589x_charger_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	u8 charge_status = 0;
	u8 temp = 0;
	int ret;

	mdelay(100);

	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret)
		return;

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret)
		return;

	ret = bq2589x_read_byte(bq, &temp, BQ2589X_REG_11);
	if (ret)
		return;

	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;

	bq2589x_dump_regs(bq);
	dev_info(bq->dev, "%s:bq status = %.2x, bq->vbus_type = %.2x\n", __func__, bq->status, bq->vbus_type);
	if (!(temp & BQ2589X_VBUS_GD_MASK) && (bq->status & BQ2589X_STATUS_PLUGIN)) {
		dev_info(bq->dev, "%s:adapter removed\n", __func__);
		bq->status &= ~BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_out_work);
	} else if ((temp & BQ2589X_VBUS_GD_MASK) && !(bq->status & BQ2589X_STATUS_PLUGIN)) {
		dev_info(bq->dev, "%s:adapter plugged in\n", __func__);
		bq->status |= BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_in_work);
	}

	if ((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG))
		bq->status |= BQ2589X_STATUS_PG;
	else if (!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG))
		bq->status &= ~BQ2589X_STATUS_PG;

	if (fault && !(bq->status & BQ2589X_STATUS_FAULT))
		bq->status |= BQ2589X_STATUS_FAULT;
	else if (!fault && (bq->status & BQ2589X_STATUS_FAULT))
		bq->status &= ~BQ2589X_STATUS_FAULT;

	charge_status = (status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	if (charge_status == BQ2589X_CHRG_STAT_IDLE)
		dev_info(bq->dev, "%s:not charging\n", __func__);
	else if (charge_status == BQ2589X_CHRG_STAT_PRECHG)
		dev_info(bq->dev, "%s:precharging\n", __func__);
	else if (charge_status == BQ2589X_CHRG_STAT_FASTCHG)
		dev_info(bq->dev, "%s:fast charging\n", __func__);
	else if (charge_status == BQ2589X_CHRG_STAT_CHGDONE)
		dev_info(bq->dev, "%s:charge done!\n", __func__);
	
	if (fault)
		dev_info(bq->dev, "%s:charge fault:%02x\n", __func__,fault);
}


static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	dev_err(bq->dev, "bq2589x_charger_interrupt\n");
	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

static struct charger_ops bq2589x_chg_ops = {
	/*normal charging*/
	.plug_in = bq2589x_plug_in,
	.plug_out = bq2589x_plug_out,
	.dump_registers = bq2589x_dump_register,
	.enable = bq2589x_enable_charging,
	.is_enabled = bq2589x_is_charging_enable,
	.get_charging_current = bq2589x_get_ichg,
	.set_charging_current = bq2589x_set_ichg,
	.get_input_current = bq2589x_get_icl,
	.set_input_current = bq2589x_set_icl,
	.get_constant_voltage = bq2589x_get_vchg,
	.set_constant_voltage = bq2589x_set_vchg,
	.kick_wdt = bq2589x_kick_wdt,
	.set_mivr = bq2589x_set_ivl,
	.is_charging_done = bq2589x_is_charging_done,
	.get_min_charging_current = bq2589x_get_min_ichg,
	.enable_chg_type_det = bq2589x_update_chg_type,
	.is_tune_done = bq2589x_is_tune_done,
	.current_tune_done = bq2589x_current_tune_done,

	/* Safety timer */
	.enable_safety_timer = bq2589x_set_safety_timer,
	.is_safety_timer_enabled = bq2589x_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = bq2589x_set_otg,
	.set_boost_current_limit = bq2589x_set_otg_current,
	.enable_discharge = NULL,

	/* PE+/PE+20 */ 
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,
	.enable_hz = bq2589x_set_hz_mode,
	.event = bq2589x_do_event,

	/* Shipping mode */
	.enable_shipping_mode = bq2589x_enable_shipping_mode,
};

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;
	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	bq->suspend_flag = false;
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	if (!ret) {
		bq->status |= BQ2589X_STATUS_EXIST;
		dev_info(bq->dev, "%s: charger device bq25890 detected, revision:%d\n", __func__, bq->revision);
	} else {
		dev_info(bq->dev, "%s: no bq25890 charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	g_bq = bq;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);
		
	/*Register charger device*/
	bq->chg_props.alias_name = "bq2589x",
	bq->chg_dev = charger_device_register(bq->chg_dev_name, 
	&client->dev, bq, &bq2589x_chg_ops, &bq->chg_props);
	if(IS_ERR_OR_NULL(bq->chg_dev)){
		dev_info(bq->dev, "%s: Register charger device failed\n", __func__);
		ret = PTR_ERR(bq->chg_dev);
		return ret;
	}
	
	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}
	bq2589x_irq = of_get_named_gpio(client->dev.of_node, "bq2589x_irq", 0);
	if(bq2589x_irq < 0){
		dev_err(bq->dev, "%s: %d get gpio failed\n", __func__, bq2589x_irq);
		ret = -EINVAL;
		goto err_0;
	}
	ret = gpio_request(bq2589x_irq, "bq2589x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, bq2589x_irq);
		goto err_0;
	}
	gpio_direction_input(bq2589x_irq);

	irqn = gpio_to_irq(bq2589x_irq);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;
	bq->dpdm_sel = of_get_named_gpio(client->dev.of_node, "bq2589x_dpdm_sel", 0);
	if(bq->dpdm_sel < 0){
		dev_err(bq->dev, "%s: %d get gpio failed\n", __func__, bq->dpdm_sel);
		ret = -EINVAL;
		goto err_1;
	}
	ret = gpio_request(bq->dpdm_sel, "bq2589x dpdm_sel pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, bq->dpdm_sel);
		goto err_1;
	}
	gpio_direction_output(bq->dpdm_sel, 0);
	if (ret)
		goto err_2;

	bq2589x_reset_pe_param();
	bq->max_pe_vol = 5000;
	bq->pe_tune_wakelock = wakeup_source_register(NULL, "bq25890 voltage tune wakelock");
	bq->chg_curr_tune_wakelock = wakeup_source_register(NULL, "bq25890 chg_curr tune wakelock");
	INIT_WORK(&bq->irq_work, bq2589x_charger_irq_workfunc);
	INIT_WORK(&bq->adapter_in_work, bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2589x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->pe_volt_tune_work, bq2589x_tune_volt_workfunc);
	INIT_DELAYED_WORK(&bq->check_pe_tuneup_work, bq2589x_check_pe_tuneup_workfunc);
	INIT_DELAYED_WORK(&bq->chg_curr_tune_work, bq2589x_chg_curr_tune_workfunc);

	ret = request_irq(client->irq, bq2589x_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2589x_charger1_irq", bq);
	if (ret) {
		dev_err(bq->dev, "%s:Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		dev_info(bq->dev, "%s:irq = %d\n", __func__, client->irq);
	}

	bq->pd_hub_active = false;
	bq->shut_down = false;
	bq->pre_curr = 0;
	//bq2589x_set_chargecurrent(bq, 0);
	schedule_work(&bq->irq_work);/*in case of adapter has been in when power off*/
	dev_info(bq->dev, "%s done\n", __func__, client->irq);
	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->chg_curr_tune_work);
err_2:
	gpio_free(bq->dpdm_sel);
err_1:
	gpio_free(bq2589x_irq);
err_0:
	g_bq = NULL;

return ret;


}

static int bq2589x_sw_reset(struct bq2589x *bq)
{
	int ret, timeout=2000;
	u8 val;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK,
			1<<BQ2589X_RESET_SHIFT);
	if (ret)
		return ret;
	while (timeout > 0) {
		timeout -= 100;
		msleep(100);
		ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
		if (ret)
			return ret;
		if ((val & BQ2589X_RESET_MASK) == 0)
			break;
	}
	return bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	dev_info(bq->dev, "%s: shutdown\n", __func__);
	bq->shut_down = true;

	cancel_work_sync(&bq->irq_work);
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->chg_curr_tune_work);

	bq2589x_sw_reset(g_bq);
	bq2589x_set_chargecurrent(g_bq, 0);
	g_bq = NULL;
}

#ifdef CONFIG_PM_SLEEP
static int bq2589x_charger_suspend(struct device *dev)
{
	struct bq2589x *bq = dev_get_drvdata(dev);

	pr_notice("[%s]\n", __func__);
	bq->suspend_flag = true;
	return 0;
}

static int bq2589x_charger_resume(struct device *dev)
{
	struct bq2589x *bq = dev_get_drvdata(dev);

	pr_notice("[%s]\n", __func__);
	bq->suspend_flag = false;
	return 0;
}

static SIMPLE_DEV_PM_OPS(bq2589x_charger_pm_ops, bq2589x_charger_suspend,
	bq2589x_charger_resume);
#endif

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq2589x-1",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x-1", BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq2589x-1",
		.of_match_table = bq2589x_charger_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &bq2589x_charger_pm_ops,
#endif
	},
	.id_table	= bq2589x_charger_id,

	.probe		= bq2589x_charger_probe,
	.shutdown   = bq2589x_charger_shutdown,
};

module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
