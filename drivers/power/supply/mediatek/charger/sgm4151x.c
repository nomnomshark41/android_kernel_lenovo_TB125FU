/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/kernel.h>
#include <linux/hardware_info.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mt-plat/upmu_common.h>

#include <mt-plat/v1/charger_class.h>
#include <mt-plat/v1/mtk_charger.h>
#include "sgm4151x.h"

#if 0
static unsigned int sgm41513_prechg_current_table[SGM41513_PRECHG_CURR_SIZE] = {
	5000,
	10000,
	15000,
	20000,
	30000,
	40000,
	50000,
	60000,
	80000,
	100000,
	120000,
	140000,
	160000,
	180000,
	200000,
	240000
};
#endif

static unsigned int sgm41513_iterm_current_table[SGM41513_ITERM_CURR_SIZE] = {
	5000,
	10000,
	15000,
	20000,
	30000,
	40000,
	50000,
	60000,
	80000,
	100000,
	120000,
	140000,
	160000,
	180000,
	200000,
	240000
};

static int __sgm4151x_read_byte(struct sgm4151x_device *sgm, u8 reg, u8 *data)
{
	s32 ret;

	if (sgm->suspend_flag)
		return -EBUSY;
	ret = i2c_smbus_read_byte_data(sgm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __sgm4151x_write_byte(struct sgm4151x_device *sgm, int reg, u8 val)
{
	s32 ret;

	if (sgm->suspend_flag)
		return -EBUSY;
	ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int sgm4151x_read_reg(struct sgm4151x_device *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4151x_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

static int sgm4151x_update_bits(struct sgm4151x_device *sgm, u8 reg,
		u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4151x_read_byte(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	ret = __sgm4151x_write_byte(sgm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

static int sgm4151x_check_pn(struct sgm4151x_device *sgm)
{
	u8 reg_val;
	int ret, pn;

	ret = sgm4151x_read_reg(sgm, SGM4151x_CHRG_CTRL_B, &reg_val);
	if (ret)
		return -1;
	pn = (reg_val>>3)&0x0f;
	switch (pn) {
		case 0x0:
			sgm->sgm41513_info = true;
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_SLAVE_INFO, "SGM41513");
			dev_info(sgm->dev, "[%s] pn:0000 ic:SGM41513\n", __func__);
			break;
		case 0x2:
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_SLAVE_INFO, "SGM41511");
			dev_info(sgm->dev, "[%s] pn:0010 ic:SGM41511\n", __func__);
			break;
		case 0x9:
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_SLAVE_INFO, "SY6974");
			dev_info(sgm->dev, "[%s] pn:1001 ic:SY6974\n", __func__);
			break;
		default:
			hardwareinfo_set_prop(HARDWARE_CHARGER_IC_SLAVE_INFO, "UNKNOWN");
			dev_info(sgm->dev, "[%s] pn:0x%02x ic:UNKNOWN\n", __func__, pn);
			return -ENODEV;
	}
	return 0;
}

static int sgm4151x_sw_reset(struct charger_device *chg_dev)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);
	int ret, timeout=2000;
	unsigned char reg_val;

	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_B, 0x80, 0x80);
	if (ret) {
		dev_err(sgm->dev, "failed to set REG_RST! ret:%d\n", ret);
		return ret;
	}
	while(timeout > 0) {
		timeout -= 100;
		msleep(100);
		ret = sgm4151x_read_reg(sgm, SGM4151x_CHRG_CTRL_B, &reg_val);
		if (ret) {
			dev_err(sgm->dev, "failed to get REG_RST! ret:%d\n", ret);
			return ret;
		}
		if ((reg_val & 0x80) == 0) {
			dev_info(sgm->dev, "finish to reset chip, cost %dms\n", 2000-timeout);
			break;
		}
	}
	return ret;
}

static int sgm4151x_set_term_curr(struct charger_device *chg_dev, int term_current)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);
	int reg_val;
	int temp;

	if (sgm->sgm41513_info == true) {
		for (temp = 0; temp < SGM41513_ITERM_CURR_SIZE; temp++) {
			if (term_current <= sgm41513_iterm_current_table[temp])
				break;
		}
		reg_val = temp;
	} else {
		if (term_current < SGM4151x_TERMCHRG_I_MIN_uA)
			term_current = SGM4151x_TERMCHRG_I_MIN_uA;
		else if (term_current > SGM4151x_TERMCHRG_I_MAX_uA)
			term_current = SGM4151x_TERMCHRG_I_MAX_uA;

		reg_val = term_current / SGM4151x_TERMCHRG_CURRENT_STEP_uA;
	}

	return sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_3,
			SGM4151x_TERMCHRG_CUR_MASK, reg_val);
}

static unsigned char sgm41513_get_ichrg_curr(unsigned int chrg_curr)
{
	unsigned char ichg;

	if (chrg_curr <= 40)
		ichg = chrg_curr / 5;
	else if (chrg_curr < 50)
		ichg = 0x08;
	else if (chrg_curr <= 110)
		ichg = 0x08 + (chrg_curr - 40) / 10;
	else if (chrg_curr < 130)
		ichg = 0x0F;
	else if (chrg_curr <= 270)
		ichg = 0x0F + (chrg_curr - 110) / 20;
	else if (chrg_curr < 300)
		ichg = 0x17;
	else if (chrg_curr <= 540)
		ichg = 0x17 + (chrg_curr - 270) / 30;
	else if (chrg_curr < 600)
		ichg = 0x20;
	else if (chrg_curr <= 1500)
		ichg = 0x20 + (chrg_curr - 540) / 60;
	else if (chrg_curr < 1620)
		ichg = 0x30;
	else if (chrg_curr <= 2940)
		ichg = 0x30 + (chrg_curr - 1500) / 120;
	else
		ichg = 0x3d;

	pr_err("[%s] chrg_curr:%dmA, ichg_reg:0x%x\n", __func__, chrg_curr, ichg);

	return ichg;
}

static void sgm4151x_chg_curr_tune_workfunc(struct work_struct *work)
{
	struct sgm4151x_device *sgm = container_of(work, struct sgm4151x_device,
			chg_curr_tune_work.work);
	unsigned char ichg;
	int ret;
	unsigned int curr;
	unsigned int sgm41513_curr;

	if (sgm->pre_curr < sgm->target_curr) {
		curr = sgm->pre_curr + 100000;
		if (curr > sgm->target_curr)
			curr = sgm->target_curr;
	} else if (sgm->pre_curr > sgm->target_curr) {
		curr = sgm->target_curr;
	} else {
		__pm_relax(sgm->chg_curr_tune_wakelock);
		return;
	}

	if (sgm->sgm41513_info == true) {
		sgm41513_curr = curr / 1000;
		ichg = sgm41513_get_ichrg_curr(sgm41513_curr);
	} else {
		ichg = curr / SGM4151x_ICHRG_CURRENT_STEP_uA;
	}

	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_2,
			SGM4151x_ICHRG_CUR_MASK, ichg);
	if (!ret) {
		sgm->pre_curr = curr;
	}
	pr_err("[%s] pre:%dmA target:%dmA\n", __func__, sgm->pre_curr/1000,
			sgm->target_curr/1000);
	if (sgm->pre_curr != sgm->target_curr) {
		schedule_delayed_work(&sgm->chg_curr_tune_work, 2*HZ);
	} else {
		__pm_relax(sgm->chg_curr_tune_wakelock);
	}
}

static int sgm4151x_set_ichrg_curr(struct charger_device *chg_dev, unsigned int chrg_curr)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	dev_err(sgm->dev, "set_ichrg_curr = %d\n", chrg_curr);
	if (chrg_curr < SGM4151x_ICHRG_I_MIN_uA)
		chrg_curr = SGM4151x_ICHRG_I_MIN_uA;

	sgm->target_curr = chrg_curr;
	if (!sgm->chg_curr_tune_wakelock->active)
		__pm_stay_awake(sgm->chg_curr_tune_wakelock);
	cancel_delayed_work_sync(&sgm->chg_curr_tune_work);
	schedule_delayed_work(&sgm->chg_curr_tune_work, 0);

	return 0;
}

static int sgm4151x_set_chrg_volt(struct charger_device *chg_dev, unsigned int chrg_volt)
{
	static int pre_cv = 4208000;
	int reg_val, ret;
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	if (chrg_volt < SGM4151x_VREG_V_MIN_uV)
		chrg_volt = SGM4151x_VREG_V_MIN_uV;

	if (pre_cv < chrg_volt) {
		sgm4151x_set_ichrg_curr(sgm->chg_dev, 0);
	}
	pre_cv = chrg_volt;

	reg_val = (chrg_volt-SGM4151x_VREG_V_MIN_uV) / SGM4151x_VREG_V_STEP_uV;
	reg_val = reg_val<<3;
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_4,
			SGM4151x_VREG_V_MASK, reg_val);

	return ret;
}

static int sgm4151x_set_input_volt_lim(struct charger_device *chg_dev, unsigned int vindpm)
{
	int ret;
	int offset;
	u8 vlim;	
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	if (vindpm < SGM4151x_VINDPM_V_MIN_uV)
		vindpm = SGM4151x_VINDPM_V_MIN_uV;
	if (vindpm > SGM4151x_VINDPM_V_MAX_uV)
		vindpm = SGM4151x_VINDPM_V_MAX_uV;

	offset = SGM4151x_VINDPM_V_MIN_uV; //uv	
	vlim = (vindpm - offset) / SGM4151x_VINDPM_STEP_uV;
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_6,
			SGM4151x_VINDPM_V_MASK, vlim); 

	return ret;
}

static int sgm4151x_set_input_curr_lim(struct charger_device *chg_dev, unsigned int iindpm)
{
	int ret;
	u8 reg_val;
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	if (iindpm < SGM4151x_IINDPM_I_MIN_uA)
		iindpm = SGM4151x_IINDPM_I_MIN_uA;
	if (iindpm > SGM4151x_IINDPM_I_MAX_uA)
		iindpm = SGM4151x_IINDPM_I_MAX_uA;

	reg_val = (iindpm-SGM4151x_IINDPM_I_MIN_uA) / SGM4151x_IINDPM_STEP_uA;	
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_0,
			SGM4151x_IINDPM_I_MASK, reg_val);
	return ret;
}

static int sgm4151x_set_hiz_en(struct charger_device *chg_dev, bool hiz_en)
{
	int reg_val;
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM4151x_HIZ_EN : 0;

	if (hiz_en) {
		sgm4151x_set_ichrg_curr(sgm->chg_dev, 0);
	}

	return sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_0,
			SGM4151x_HIZ_EN, reg_val);
}

static int sgm4151x_enable_shipping_mode(struct charger_device *chg_dev, bool en)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);
	int ret;

	if (en) {
		dev_warn(sgm->dev, "%s:%d", __func__, en);
		ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_7, 0x04, 0);
		if (ret) {
			dev_warn(sgm->dev, "[%s] set batfet_dly failed:%d", __func__, ret);
			return ret;
		}
		ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_7, 0x20, 0x20);
		if (ret) {
			dev_warn(sgm->dev, "[%s] set batfet_dis failed:%d", __func__, ret);
			return ret;
		}
	} else {
		/* do nothing, should never happen */
	}
	return 0;
}

static int sgm4151x_plug_out(struct charger_device *chg_dev)
{
	sgm4151x_set_input_volt_lim(chg_dev, 12000000);
	return sgm4151x_set_hiz_en(chg_dev, true);
}

static int sgm4151x_enable(struct charger_device *chg_dev, bool enable)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	if (!enable) {
		sgm4151x_set_ichrg_curr(sgm->chg_dev, 0);
	}
	sgm->enable = enable;
	gpio_direction_output(sgm->chg_en_gpio, !enable);
	gpio_direction_output(sgm->new_chg_en_gpio, !enable);
	return sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_1, 0x10, (!!enable)<<4);
}

static int sgm4151x_is_enabled(struct charger_device *chg_dev, bool *enable)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	*enable = sgm->enable;
	return 0;
}

static int sgm4151x_current_tune_done(struct charger_device *chg_dev, bool *tune_done)
{
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	*tune_done = (sgm->pre_curr == sgm->target_curr);
	return 0;
}

static int sgm4151x_dump_register(struct charger_device *chg_dev)
{
	int i = 0;
	u8 reg = 0;
	int sgm_reg = 0;
	struct sgm4151x_device *sgm = charger_get_data(chg_dev);

	return 0; //disable dump regs

	if (sgm->suspend_flag)
		return -EBUSY;

	if (sgm->sgm41513_info == true) {
		sgm_reg = SGM41513_CHRG_CTRL_F;
	} else {
		sgm_reg = SGM4151x_CHRG_CTRL_B;
	}

	for(i=0; i <= sgm_reg; i++) {
		sgm4151x_read_reg(sgm, i, &reg);
		pr_err("%s REG%02x  %02x\n", __func__, i, reg);
	}
	return 0;
}

static int sgm4151x_hw_init(struct sgm4151x_device *sgm)
{
	int ret;

	/* set hz */
	ret = sgm4151x_set_hiz_en(sgm->chg_dev, true);
	if (ret)
		return ret;
	/* disable wdt */
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_5,
			SGM4151x_WDT_TIMER_MASK, SGM4151x_WDT_TIMER_DISABLE);
	if (ret)
		return ret;
	/* disable safe timer*/
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_5,
			SGM4151x_SAFETY_TIMER_MASK, SGM4151x_SAFETY_TIMER_DISABLE);
	if (ret)
		return ret;
	/* set ovp 14V */
	ret = sgm4151x_update_bits(sgm, SGM4151x_CHRG_CTRL_6,
			SGM4151x_VAC_OVP_MASK, 0xC0);
	if (ret)
		return ret;
	/* set Iterm 200mA */
	ret = sgm4151x_set_term_curr(sgm->chg_dev, 200000);
	if (ret)
		return ret;
	/* set Ichg 0mA */
	ret = sgm4151x_set_ichrg_curr(sgm->chg_dev, 0);
	if (ret)
		return ret;
	/* set CV 4.43V */
	ret = sgm4151x_set_chrg_volt(sgm->chg_dev, 4432000);
	if (ret)
		return ret;
	/* set Vindpm 12V */
	ret = sgm4151x_set_input_volt_lim(sgm->chg_dev, 12000000);
	if (ret)
		return ret;

	return ret;
}

static int sgm4151x_parse_dt(struct sgm4151x_device *sgm)
{
	int ret;
	int irq_gpio = 0, irqn = 0;	

	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	ret = gpio_request(irq_gpio, "sgm4151x irq pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	sgm->client->irq = irqn;

	sgm->chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (!gpio_is_valid(sgm->chg_en_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, sgm->chg_en_gpio);
		return -EINVAL;
	}
	ret = gpio_request(sgm->chg_en_gpio, "sgm chg en pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, sgm->chg_en_gpio);
		return ret;
	}

	sgm->new_chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,new-chg-en-gpio", 0);
	if (!gpio_is_valid(sgm->new_chg_en_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, sgm->new_chg_en_gpio);
		return -EINVAL;
	}
	ret = gpio_request(sgm->new_chg_en_gpio, "new sgm chg en pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, sgm->new_chg_en_gpio);
		return ret;
	}
	gpio_direction_output(sgm->new_chg_en_gpio, 1);//default disable charge

	/* en gpio power domain 2.76V */
	pmic_set_register_value(PMIC_RG_LDO_VSIM1_EN, 1);
	mdelay(3);
	pmic_set_register_value(PMIC_RG_VSIM1_VOSEL, 8);//3/4/8/11/12=>1.7/1.8/2.7/3.0/3.1V
	gpio_direction_output(sgm->chg_en_gpio, 1);//default disable charge
	sgm->enable = false;

	return 0;
}

static struct charger_ops sgm4151x_chg_ops = {
	.enable_hz = sgm4151x_set_hiz_en,
	.enable_shipping_mode = sgm4151x_enable_shipping_mode,

	/* Normal charging */
	.enable = sgm4151x_enable,
	.is_enabled = sgm4151x_is_enabled,
	.dump_registers = sgm4151x_dump_register,
	.set_charging_current = sgm4151x_set_ichrg_curr,
	.set_input_current = sgm4151x_set_input_curr_lim,
	.set_constant_voltage = sgm4151x_set_chrg_volt,
	.set_mivr = sgm4151x_set_input_volt_lim,
	.plug_out = sgm4151x_plug_out,
	.current_tune_done = sgm4151x_current_tune_done,
};

static int sgm4151x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct sgm4151x_device *sgm;

	printk("sgm4151x start probe\n");
	sgm = devm_kzalloc(&client->dev, sizeof(struct sgm4151x_device), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	mutex_init(&sgm->i2c_rw_lock);

	sgm->client = client;
	sgm->dev = &client->dev;
	sgm->suspend_flag = false;
	i2c_set_clientdata(client, sgm);

	sgm->sgm41513_info = false;
	ret = sgm4151x_check_pn(sgm);
	if (ret)
		return -ENODEV;
	// Customer customization
	ret = sgm4151x_parse_dt(sgm);
	if (ret) {
		dev_err(sgm->dev, "Failed to read device tree properties<errno:%d>\n", ret);
		return ret;
	}

	sgm->chg_curr_tune_wakelock = wakeup_source_register(NULL, "sub_charger chg_curr tune wakelock");
	INIT_DELAYED_WORK(&sgm->chg_curr_tune_work, sgm4151x_chg_curr_tune_workfunc);

	/* Register charger device */
	sgm->sgm4151x_chg_props.alias_name = "sgm4151x";
	sgm->chg_dev = charger_device_register("primary_divider_chg",
			sgm->dev, sgm, &sgm4151x_chg_ops, &sgm->sgm4151x_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}

	ret = sgm4151x_hw_init(sgm);
	if (ret) {
		dev_err(sgm->dev, "Cannot initialize the chip.\n");
		return ret;
	}

	sgm4151x_dump_register(sgm->chg_dev);
	return ret;
}

static int sgm4151x_charger_remove(struct i2c_client *client)
{
	struct sgm4151x_device *sgm = i2c_get_clientdata(client);

	mutex_destroy(&sgm->i2c_rw_lock);

	return 0;
}

static void sgm4151x_charger_shutdown(struct i2c_client *client)
{
	struct sgm4151x_device *sgm = i2c_get_clientdata(client);
	int ret;

	ret = sgm4151x_sw_reset(sgm->chg_dev);
	ret = sgm4151x_set_hiz_en(sgm->chg_dev, true);
	ret = sgm4151x_enable_shipping_mode(sgm->chg_dev, true);
}

#ifdef CONFIG_PM_SLEEP
static int sgm4151x_charger_suspend(struct device *dev)
{
	struct sgm4151x_device *sgm = dev_get_drvdata(dev);

	dev_info(dev, "[%s]\n", __func__);
	sgm->suspend_flag = true;
	return 0;
}

static int sgm4151x_charger_resume(struct device *dev)
{
	struct sgm4151x_device *sgm = dev_get_drvdata(dev);

	dev_info(dev, "[%s]\n", __func__);
	sgm->suspend_flag = false;
	return 0;
}

static SIMPLE_DEV_PM_OPS(sgm4151x_charger_pm_ops, sgm4151x_charger_suspend,
	sgm4151x_charger_resume);
#endif

static const struct i2c_device_id sgm4151x_i2c_ids[] = {
	{ "sgm41511", 0 },
	{ "sgm41512", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4151x_i2c_ids);

static const struct of_device_id sgm4151x_of_match[] = {
	{.compatible = "sgm,sgm41511",},
	{},
};
MODULE_DEVICE_TABLE(of, sgm4151x_of_match);

static struct i2c_driver sgm4151x_driver = {
	.driver = {
		.name = "sgm4151x-charger",
		.of_match_table = sgm4151x_of_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &sgm4151x_charger_pm_ops,
#endif
	},
	.probe = sgm4151x_probe,
	.remove = sgm4151x_charger_remove,
	.shutdown = sgm4151x_charger_shutdown,
	.id_table = sgm4151x_i2c_ids,
};
module_i2c_driver(sgm4151x_driver);

MODULE_AUTHOR(" qhq <allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4151x charger driver");
MODULE_LICENSE("GPL v2");
