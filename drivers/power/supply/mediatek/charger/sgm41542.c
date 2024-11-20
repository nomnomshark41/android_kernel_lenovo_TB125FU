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
#include <linux/mutex.h>
#include <mt-plat/upmu_common.h>
#include <linux/hardware_info.h>
#include <mt-plat/mtk_boot_common.h>

#include "mtk_charger_intf.h"
//#include "mtk_charger.h"

#define SGM41542_REG_00 0x00
#define SGM41542_EN_HIZ_MASK            0x80
#define SGM41542_EN_HIZ_SHIFT           7
#define SGM41542_EN_ICHG_MON_MASK       0x60
#define SGM41542_EN_ICHG_MON_SHIFT      5
#define SGM41542_IINDPM_MASK            0x1F
#define SGM41542_IINDPM_SHIFT           0

#define SGM41542_REG_01 0x01
#define SGM41542_PFM_DIS_MASK           0x80
#define SGM41542_PFM_DIS_SHIFT          7
#define SGM41542_WD_RST_MASK            0x40
#define SGM41542_WD_RST_SHIFT           6
#define SGM41542_OTG_CONFIG_MASK        0x20
#define SGM41542_OTG_CONFIG_SHIFT       5
#define SGM41542_CHG_CONFIG_MASK        0x10
#define SGM41542_CHG_CONFIG_SHIFT       4
#define SGM41542_SYS_MIN_MASK           0x0E
#define SGM41542_SYS_MIN_SHIFT          1
#define SGM41542_MIN_BAT_SEL_MASK       0x01
#define SGM41542_MIN_BAT_SEL_SHIFT      0

#define SGM41542_REG_02 0x02
#define SGM41542_BOOST_LIM_MASK         0x80
#define SGM41542_BOOST_LIM_SHIFT        7
#define SGM41542_Q1_FULLON_MASK         0x40
#define SGM41542_Q1_FULLON_SHIFT        6
#define SGM41542_ICHG_MASK              0x3F
#define SGM41542_ICHG_SHIFT             0

#define SGM41542_REG_03 0x03
#define SGM41542_IPRECHG_MASK           0xF0
#define SGM41542_IPRECHG_SHIFT          4
#define SGM41542_ITERM_MASK             0x0F
#define SGM41542_ITERM_SHIFT            0

#define SGM41542_REG_04 0x04
#define SGM41542_VREG_MASK              0xF8
#define SGM41542_VREG_SHIFT             3
#define SGM41542_TOPOFF_TIMER_MASK      0x06
#define SGM41542_TOPOFF_TIMER_SHIFT     1
#define SGM41542_VRECHG_MASK            0x01
#define SGM41542_VRECHG_SHIFT           0

#define SGM41542_REG_05 0x05
#define SGM41542_EN_TERM_MASK           0x80
#define SGM41542_EN_TERM_SHIFT          7
#define SGM41542_ARDCEN_ITS_MASK        0x40
#define SGM41542_ARDCEN_ITS_SHIFT       6
#define SGM41542_WATCHDOG_MASK          0x30
#define SGM41542_WATCHDOG_SHIFT         4
#define SGM41542_EN_TIMER_MASK          0x08
#define SGM41542_EN_TIMER_SHIFT         3
#define SGM41542_CHG_TIMER_MASK         0x04
#define SGM41542_CHG_TIMER_SHIFT        2
#define SGM41542_TREG_MASK              0x02
#define SGM41542_TREG_SHIFT             1
#define SGM41542_JEITA_ISET_L_MASK      0x01
#define SGM41542_JEITA_ISET_L_SHIFT     0

#define SGM41542_REG_06 0x06
#define SGM41542_OVP_MASK               0xC0
#define SGM41542_OVP_SHIFT              6
#define SGM41542_BOOSTV_MASK            0x30
#define SGM41542_BOOSTV_SHIFT           4
#define SGM41542_VINDPM_MASK            0x0F
#define SGM41542_VINDPM_SHIFT           0

#define SGM41542_REG_07 0x07
#define SGM41542_IINDET_EN_MASK         0x80
#define SGM41542_IINDET_EN_SHIFT        7
#define SGM41542_TMR2X_EN_MASK          0x40
#define SGM41542_TMR2X_EN_SHIFT         6
#define SGM41542_BATFET_DIS_MASK        0x20
#define SGM41542_BATFET_DIS_SHIFT       5
#define SGM41542_JEITA_VSET_H_MASK      0x10
#define SGM41542_JEITA_VSET_H_SHIFT     4
#define SGM41542_BATFET_DLY_MASK        0x08
#define SGM41542_BATFET_DLY_SHIFT       3
#define SGM41542_BATFET_RST_EN_MASK     0x04
#define SGM41542_BATFET_RST_EN_SHIFT    2
#define SGM41542_VDPM_BAT_TRACK_MASK    0x03
#define SGM41542_VDPM_BAT_TRACK_SHIFT   0

#define SGM41542_REG_08 0x08
#define SGM41542_VBUS_STAT_MASK         0xE0
#define SGM41542_VBUS_STAT_SHIFT        5
#define SGM41542_CHRG_STAT_MASK         0x18
#define SGM41542_CHRG_STAT_SHIFT        3
#define SGM41542_PG_STAT_MASK           0x04
#define SGM41542_PG_STAT_SHIFT          2
#define SGM41542_THERM_STAT_MASK        0x02
#define SGM41542_THERM_STAT_SHIFT       1
#define SGM41542_VSYS_STAT_MASK         0x01
#define SGM41542_VSYS_STAT_SHIFT        0

#define SGM41542_REG_09 0x09
#define SGM41542_WATCHDOG_FAULT_MASK    0x80
#define SGM41542_WATCHDOG_FAULT_SHIFT   7
#define SGM41542_BOOST_FAULT_MASK       0x40
#define SGM41542_BOOST_FAULT_SHIFT      6
#define SGM41542_CHRG_FAULT_MASK        0x30
#define SGM41542_CHRG_FAULT_SHIFT       4
#define SGM41542_BAT_FAULT_MASK         0x08
#define SGM41542_BAT_FAULT_SHIFT        3
#define SGM41542_NTC_FAULT_MASK         0x07
#define SGM41542_NTC_FAULT_SHIFT        0

#define SGM41542_REG_0A 0x0A
#define SGM41542_VBUS_GD_MASK           0x80
#define SGM41542_VBUS_GD_SHIFT          7
#define SGM41542_VINDPM_STAT_MASK       0x40
#define SGM41542_VINDPM_STAT_SHIFT      6
#define SGM41542_IINDPM_STAT_MASK       0x20
#define SGM41542_IINDPM_STAT_SHIFT      5
#define SGM41542_CV_STAT_MASK           0x10
#define SGM41542_CV_STAT_SHIFT          4
#define SGM41542_TOPOFF_ACTIVE_MASK     0x08
#define SGM41542_TOPOFF_ACTIVE_SHIFT    3
#define SGM41542_ACOV_STAT_MASK         0x04
#define SGM41542_ACOV_STAT_SHIFT        2
#define SGM41542_VINDPM_INT_MASK_MASK   0x02
#define SGM41542_VINDPM_INT_MASK_SHIFT  1
#define SGM41542_IINDPM_INT_MASK_MASK   0x01
#define SGM41542_IINDPM_INT_MASK_SHIFT  0

#define SGM41542_REG_0B 0x0B
#define SGM41542_REG_RST_MASK           0x80
#define SGM41542_REG_RST_SHIFT          4
#define SGM41542_PN_MASK                0x78
#define SGM41542_PN_SHIFT               3
#define SGM41542_SGMPART_MASK           0x04
#define SGM41542_SGMPART_SHIFT          2
#define SGM41542_DEV_REV_MASK           0x03
#define SGM41542_DEV_REV_SHIFT          0

#define SGM41542_REG_0C 0x0C
#define SGM41542_JEITA_VSET_L_MASK      0x80
#define SGM41542_JEITA_VSET_L_SHIFT     7
#define SGM41542_JEITA_ISET_L_EN_MASK   0x40
#define SGM41542_JEITA_ISET_L_EN_SHIFT  6
#define SGM41542_JEITA_ISET_H_MASK      0x30
#define SGM41542_JEITA_ISET_H_SHIFT     4
#define SGM41542_JEITA_VT2_MASK         0x0C
#define SGM41542_JEITA_VT2_SHIFT        2
#define SGM41542_JEITA_VT3_MASK         0x03
#define SGM41542_JEITA_VT3_SHIFT        0

#define SGM41542_REG_0D 0x0D
#define SGM41542_EN_PUMPX_MASK          0x80
#define SGM41542_EN_PUMPX_SHIFT         7
#define SGM41542_PUMPX_UP_MASK          0x40
#define SGM41542_PUMPX_UP_SHIFT         6
#define SGM41542_PUMPX_DN_MASK          0x20
#define SGM41542_PUMPX_DN_SHIFT         5
#define SGM41542_DP_VSET_MASK           0x18
#define SGM41542_DP_VSET_SHIFT          3
#define SGM41542_DM_VSET_MASK           0x06
#define SGM41542_DM_VSET_SHIFT          1
#define SGM41542_JEITA_EN_MASK          0x01
#define SGM41542_JEITA_EN_SHIFT         0

#define SGM41542_REG_0E 0x0E
#define SGM41542_INPUT_DET_DONE_MASK    0x80
#define SGM41542_INPUT_DET_DONE_SHIFT   7

#define SGM41542_REG_0F 0x0F
#define SGM41542_VREG_FT_MASK           0xC0
#define SGM41542_VREG_FT_SHIFT          6
#define SGM41542_DCEN_MASK              0x10
#define SGM41542_DCEN_SHIFT             4
#define SGM41542_STAT_SET_MASK          0x0C
#define SGM41542_STAT_SET_SHIFT         2
#define SGM41542_VINDPM_OS_MASK         0x03
#define SGM41542_VINDPM_OS_SHIFT        0

#define SGM41542_PN 0x0D

enum sgm41542_vbus_stat {
    SGM41542_VBUS_STAT_NONE = 0,
    SGM41542_VBUS_STAT_SDP,
    SGM41542_VBUS_STAT_CDP,
    SGM41542_VBUS_STAT_DCP,
    SGM41542_VBUS_STAT_UNKNOWN = 5,
    SGM41542_VBUS_STAT_NONSTANDARD,
    SGM41542_VBUS_STAT_OTG,
    SGM41542_VBUS_STAT_MAX = SGM41542_VBUS_STAT_OTG
};

struct pe_ctrl {
    bool tune_done;
    bool tune_rework;
    int tune_count;
    int target_vol;
};

struct sgm41542_charge_config {
    unsigned int input_current_limit;
    unsigned int charge_current_limit;
    unsigned int constant_charge_voltage;
    unsigned int terminate_charge_current;
};

struct sgm41542_status {
    unsigned int online: 1;//plugged
    unsigned int iindpm: 1;
    unsigned int vindpm: 1;
    unsigned int vbus: 14;//mv
    unsigned int vbus_stat:3;
    unsigned int reserved:12;
};

struct sgm41542_chip {
    struct device *dev;
    struct i2c_client *client;

    struct charger_device *chg_dev;
    struct charger_properties chg_props;

    bool suspend_flag;

    int max_pe_vol;
    int sgm41542_irq;
    int sgm41542_dpdm_sel;
    int default_target_vol;
    struct sgm41542_status status;
    struct pe_ctrl pe;
    struct sgm41542_charge_config chg_cfg;
    struct work_struct irq_work;
    struct work_struct in_work;
    struct work_struct out_work;
    struct work_struct pe_tune_work;
    struct work_struct check_pe_work;
    struct delayed_work monitor_work;

    struct wakeup_source* pe_tune_wakelock;

    struct mutex sgm41542_i2c_lock;
    struct mutex sgm41542_update_ct_lock;
};

//i2c ops
static int sgm41542_read_byte(struct sgm41542_chip *chip, unsigned char *data,
                                unsigned char reg)
{
    int ret;

    if (chip->suspend_flag)
        return -EBUSY;

    mutex_lock(&chip->sgm41542_i2c_lock);
    ret = i2c_smbus_read_byte_data(chip->client, reg);
    if (ret >= 0)
        *data = (unsigned char)ret;
    mutex_unlock(&chip->sgm41542_i2c_lock);
    return ret<0?ret:0;
}

static int sgm41542_write_byte(struct sgm41542_chip *chip, unsigned char reg,
                                unsigned char data)
{
    int ret;

    if (chip->suspend_flag)
        return -EBUSY;

    mutex_lock(&chip->sgm41542_i2c_lock);
    ret = i2c_smbus_write_byte_data(chip->client, reg, data);
    mutex_unlock(&chip->sgm41542_i2c_lock);
    return ret;
}

static int sgm41542_update_bits(struct sgm41542_chip *chip, unsigned char reg,
                                unsigned char mask, unsigned char data)
{
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, reg);
    if (ret)
        return ret;

    reg_val &= ~mask;
    reg_val |= data & mask;

    return sgm41542_write_byte(chip, reg, reg_val);
}

//charger ops
static int sgm41542_dump_register(struct charger_device *chg_dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret, reg;
    unsigned char reg_val;

    dev_info(chip->dev, "[%s] charger status:0x%.8X\n", __func__, chip->status);
    for (reg=SGM41542_REG_00;reg<=SGM41542_REG_0F;reg++) {
        ret = sgm41542_read_byte(chip, &reg_val, reg);
        if (ret) {
            if (!chip->suspend_flag)
                dev_err(chip->dev, "[%s] failed read reg[%.2X]!(ret:%d)\n",
                    __func__, reg, ret);
            break;
        }
        dev_info(chip->dev, "[%s] reg[%.2X] = 0x%.2X\n", __func__, reg, reg_val);
    }
    return ret<0?ret:0;
}

static int sgm41542_sw_reset(struct charger_device *chg_dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret, timeout=2000;
    unsigned char reg_val;

    reg_val = (1<<SGM41542_REG_RST_SHIFT) & SGM41542_REG_RST_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_0B, SGM41542_REG_RST_MASK, reg_val);
    if (ret) {
        dev_err(chip->dev, "[%s] failed set reg[%.2X]!(ret:%d)\n", __func__, 0x0B, ret);
        return ret;
    }
    while (timeout>0) {
        timeout -= 100;
        msleep(100);
        ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_0B);
        if (ret) {
            dev_err(chip->dev, "[%s] failed read reg[%.2X]!(ret:%d)\n", __func__, 0x0B, ret);
            return ret;
        }
        if ((reg_val & SGM41542_REG_RST_MASK)>>SGM41542_REG_RST_SHIFT == 0) {
            dev_info(chip->dev, "[%s] reset chip finish, cost %dms!\n", __func__, 2000-timeout);
            break;
        }
    }
    return ret;
}

static int sgm41542_enable_charging(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    reg_val = ((en?1:0)<<SGM41542_CHG_CONFIG_SHIFT) & SGM41542_CHG_CONFIG_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_01, SGM41542_CHG_CONFIG_MASK, reg_val);
    return ret;
}

static int sgm41542_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_01);
    if (ret)
        return ret;

    *en = (((reg_val & SGM41542_CHG_CONFIG_MASK)>>SGM41542_CHG_CONFIG_SHIFT) == 1);
    return ret;
}

static int sgm41542_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_02);
    if (ret)
        return ret;

    *uA = ((reg_val & SGM41542_ICHG_MASK)>>SGM41542_ICHG_SHIFT)*60000;
    return ret;
}

static int sgm41542_set_ichg(struct charger_device *chg_dev, u32 uA)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    reg_val = ((uA / 60000)<<SGM41542_ICHG_SHIFT) & SGM41542_ICHG_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_02, SGM41542_ICHG_MASK, reg_val);

    return ret;
}

static int sgm41542_get_icl(struct charger_device *chg_dev, u32 *uA)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_00);
    if (ret)
        return ret;

    *uA = ((reg_val & SGM41542_IINDPM_MASK)>>SGM41542_IINDPM_SHIFT)*100000+100000;
    if (*uA == 3200000)
        *uA = 3800000;
    return ret;
}

static int sgm41542_set_icl(struct charger_device *chg_dev, u32 uA)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    if (uA>3100000 && uA!=3800000)
        return -EINVAL;

    if (uA < 100000)
        uA = 100000;

    if (uA == 3800000)
        reg_val = 31;
    else
        reg_val = (((uA-100000)/100000)<<SGM41542_IINDPM_SHIFT) & SGM41542_IINDPM_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_00, SGM41542_IINDPM_MASK, reg_val);

    return ret;
}

static int sgm41542_get_vchg(struct charger_device *chg_dev, u32 *uV)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_04);
    if (ret)
        return ret;

    *uV = ((reg_val & SGM41542_VREG_MASK)>>SGM41542_VREG_SHIFT)*32000+3856000;
    if (*uV == 4336000)
        *uV = 4352000;
    return ret;
}

static int sgm41542_set_vchg(struct charger_device *chg_dev, u32 uV)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;
    static unsigned int pre_cv = 0;

    if (uV>=4336000 && uV<4352000)
        uV = 4320000;

    if (pre_cv == uV) {
        dev_info(chip->dev, "[%s] same cv set(%dmV)\n", __func__, uV/1000);
        return 0;
    }

    if (battery_get_bat_voltage()>4080 && uV<=4200000) {
        dev_err(chip->dev, "[%s] vbat(%dmV) too high to decrease cv(%dmV)\n", __func__,
                battery_get_bat_voltage(), uV/1000);
        return 0;
    }

    reg_val = (((uV-3856000)/32000)<<SGM41542_VREG_SHIFT) & SGM41542_VREG_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_04, SGM41542_VREG_MASK, reg_val);

    return ret;
}

static int sgm41542_kick_wdt(struct charger_device *chg_dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;

    ret = sgm41542_update_bits(chip, SGM41542_REG_01, SGM41542_WD_RST_MASK,
            1<<SGM41542_WD_RST_SHIFT);

    return ret;
}

static int sgm41542_set_mivr(struct charger_device *chg_dev, u32 uV)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val_vindpm, reg_val_vindpm_os;

    if (uV>=3900000 && uV<=5400000) {
        reg_val_vindpm_os = (0<<SGM41542_VINDPM_OS_SHIFT) & SGM41542_VINDPM_OS_MASK;
        reg_val_vindpm = (((uV-3900000)/100000)<<SGM41542_VINDPM_SHIFT)
                            & SGM41542_VINDPM_MASK;
    } else if (uV>=5900000 && uV<=7400000) {
        reg_val_vindpm_os = (1<<SGM41542_VINDPM_OS_SHIFT) & SGM41542_VINDPM_OS_MASK;
        reg_val_vindpm = (((uV-5900000)/100000)<<SGM41542_VINDPM_SHIFT)
                            & SGM41542_VINDPM_MASK;
    } else if (uV>=7500000 && uV<=9000000) {
        reg_val_vindpm_os = (2<<SGM41542_VINDPM_OS_SHIFT) & SGM41542_VINDPM_OS_MASK;
        reg_val_vindpm = (((uV-7500000)/100000)<<SGM41542_VINDPM_SHIFT)
                            & SGM41542_VINDPM_MASK;
    } else if (uV>=10500000 && uV<=12000000) {
        reg_val_vindpm_os = (3<<SGM41542_VINDPM_OS_SHIFT) & SGM41542_VINDPM_OS_MASK;
        reg_val_vindpm = (((uV-10500000)/100000)<<SGM41542_VINDPM_SHIFT)
                            & SGM41542_VINDPM_MASK;
    } else {
        return -EINVAL;
    }

    ret = sgm41542_update_bits(chip, SGM41542_REG_0F, SGM41542_VINDPM_OS_MASK,
            reg_val_vindpm_os);
    if (ret)
        return ret;

    ret = sgm41542_update_bits(chip, SGM41542_REG_06, SGM41542_VINDPM_MASK,
            reg_val_vindpm);

    return ret;
}

static int sgm41542_is_tune_done(struct charger_device *chg_dev, bool *tune_done)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);

    *tune_done = chip->pe.tune_done;

    return 0;
}

static int sgm41542_current_tune_done(struct charger_device *chg_dev, bool *tune_done)
{
    *tune_done = true;
    return 0;
}

static int sgm41542_is_charging_done(struct charger_device *chg_dev, bool *done)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_08);
    if (ret)
        return ret;

    *done = (((reg_val & SGM41542_CHRG_STAT_MASK)>>SGM41542_CHRG_STAT_SHIFT)
                == 3);
    return ret;
}

static int sgm41542_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
    *uA = 0;

    return 0;
}

static int sgm41542_force_dpdm(struct sgm41542_chip *chip)
{
    int ret, timeout=600;
    unsigned char reg_val;

    ret = sgm41542_update_bits(chip, SGM41542_REG_07, SGM41542_IINDET_EN_MASK,
            1<<SGM41542_IINDET_EN_SHIFT);
    if (ret)
        return ret;

    while (timeout > 0) {
        msleep(20);
        timeout -= 20;
        ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_07);
        if (ret)
            return ret;

        if ((reg_val & SGM41542_IINDET_EN_MASK)>>SGM41542_IINDET_EN_SHIFT == 0)
            break;
    }

    return ret;
}

static int sgm41542_update_vbus_type(struct sgm41542_chip *chip)
{
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_08);
    if (ret)
        return ret;

    chip->status.vbus_stat = 
        (reg_val & SGM41542_VBUS_STAT_MASK)>>SGM41542_VBUS_STAT_SHIFT;

    return ret;
}

static int sgm41542_get_plugged(struct sgm41542_chip *chip)
{
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_08);
    if (ret) {
        dev_err(chip->dev, "[%s] failed to read PG_STAT!(ret:%d)\n",
                __func__, ret);
        return 0;
    }

    chip->status.online == (reg_val & SGM41542_PG_STAT_MASK)>>SGM41542_PG_STAT_SHIFT;
    return chip->status.online;
}

static int sgm41542_get_non_standard_type(struct sgm41542_chip *chip)
{
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_00);
    if (ret) {
        dev_err(chip->dev, "[%s] failed to read IINDPM!(ret:%d)\n",
                __func__, ret);
        return CHARGER_NS_1A;
    }
    switch ((reg_val & SGM41542_IINDPM_MASK)>>SGM41542_IINDPM_SHIFT) {
        case 9://1A
            ret = CHARGER_NS_1A;
            break;
        case 19://2A
            ret = CHARGER_NS_2A;
            break;
        case 20://2.1A
            ret = CHARGER_NS_2_1A;
            break;
        case 23://2.4A
            ret = CHARGER_NS_2_4A;
            break;
        default:
            ret = CHARGER_NS_1A;
            break;
    }

    return ret;
}

extern int battery_get_boot_mode(void);
static int sgm41542_update_chg_type(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    union power_supply_propval propval;
    struct power_supply *chrdet_psy;
    enum charger_type chg_type;
    int ret, wait_cdp_timeout=25000;
    static first_connect = true;
    int boot_mode = battery_get_boot_mode();

    chrdet_psy = power_supply_get_by_name("charger");
    if (!chrdet_psy) {
        dev_err(chip->dev, "[%s] failed to get power supply 'charger'!\n",
                __func__, ret);
        return -EINVAL;
    }

    mutex_lock(&chip->sgm41542_update_ct_lock);
    if (first_connect
            && boot_mode != KERNEL_POWER_OFF_CHARGING_BOOT
            && boot_mode != LOW_POWER_OFF_CHARGING_BOOT) {
        while (wait_cdp_timeout > 0) {
            if (is_usb_rdy()) {
                dev_info(chip->dev, "[%s] CDP, PASS\n", __func__);
                break;
            }
            dev_info(chip->dev, "[%s] CDP, block\n", __func__);
            wait_cdp_timeout -= 100;
            msleep(100);
        }
        if (wait_cdp_timeout > 0) {
            dev_info(chip->dev, "[%s] CDP, timeout\n", __func__);
        } else {
            dev_info(chip->dev, "[%s] CDP, free\n", __func__);
        }
        first_connect = false;
    }
    if (en) {
        Charger_Detect_Init();
        gpio_direction_output(chip->sgm41542_dpdm_sel, 1);

        msleep(600);//wait ic stable
        ret = sgm41542_force_dpdm(chip);
        if (ret) {
            dev_err(chip->dev, "[%s] force_dpdm fail(%d)\n", __func__, ret);
            mutex_unlock(&chip->sgm41542_update_ct_lock);
            return ret;
        }
        ret = sgm41542_update_vbus_type(chip);
        if (ret) {
            dev_err(chip->dev, "[%s] update vbus_type fail(%d)\n", __func__, ret);
            mutex_unlock(&chip->sgm41542_update_ct_lock);
            return ret;
        }
        switch (chip->status.vbus_stat) {
            case SGM41542_VBUS_STAT_SDP:
                chg_type = STANDARD_HOST;
                break;
            case SGM41542_VBUS_STAT_CDP:
                chg_type = CHARGING_HOST;
                break;
            case SGM41542_VBUS_STAT_DCP:
                chg_type = STANDARD_CHARGER;
                break;
            case SGM41542_VBUS_STAT_NONSTANDARD:
                chg_type = sgm41542_get_non_standard_type(chip);
                break;
            case SGM41542_VBUS_STAT_UNKNOWN:
                chg_type = CHARGER_FLOATING;
                break;
            case SGM41542_VBUS_STAT_OTG:
            default:
                chg_type = NONSTANDARD_CHARGER;
                break;
        }

        gpio_direction_output(chip->sgm41542_dpdm_sel, 0);
        Charger_Detect_Release();
        if (chg_type == STANDARD_CHARGER) {
            if (!chip->pe_tune_wakelock->active)
                __pm_stay_awake(chip->pe_tune_wakelock);
            schedule_work(&chip->pe_tune_work);
        }

        dev_info(chip->dev, "[%s] en:%d vbus_stat:%d chg_type:%d\n", __func__, en,
                chip->status.vbus_stat, chg_type);

        ret = power_supply_get_property(chrdet_psy,
                POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
        if (ret < 0) {
            pr_err("[%s]: get charge type failed, ret = %d\n", __func__, ret);
            mutex_unlock(&chip->sgm41542_update_ct_lock);
            return ret;
        }
    } else {
        propval.intval = CHARGER_UNKNOWN;
        dev_info(chip->dev, "[%s] en:%d\n", __func__, en);
    }
    if (!en || (propval.intval!=CHARGER_PD_9V && propval.intval!=CHARGER_PD_5V
            && propval.intval!=CHARGER_PE_12V && propval.intval!=CHARGER_PE_9V
            && propval.intval!=CHARGER_PE_5V)) {
        propval.intval = chg_type;
        ret = power_supply_set_property(chrdet_psy,
                POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
        if (ret < 0) {
            pr_err("[%s]: set charge type failed, ret = %d\n", __func__, ret);
            mutex_unlock(&chip->sgm41542_update_ct_lock);
            return ret;
        }
    }

    mutex_unlock(&chip->sgm41542_update_ct_lock);
    return ret;
}

static int sgm41542_get_hz_mode(struct charger_device *chg_dev, bool *en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_00);
    if (ret)
        return ret;
    *en = ((reg_val & SGM41542_EN_HIZ_MASK)>>SGM41542_EN_HIZ_SHIFT);
    return ret;
}

static int sgm41542_set_hz_mode(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    if (en) {
        sgm41542_set_ichg(chip->chg_dev, 0);
    }
    reg_val = ((en?1:0)<<SGM41542_EN_HIZ_SHIFT) & SGM41542_EN_HIZ_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_00, SGM41542_EN_HIZ_MASK, reg_val);
    return ret;
}

static int sgm41542_set_safety_timer(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    reg_val = ((en?1:0)<<SGM41542_EN_TIMER_SHIFT) & SGM41542_EN_TIMER_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_05, SGM41542_EN_TIMER_MASK, reg_val);

    return ret;
}

static int sgm41542_is_safety_timer_enabled(struct charger_device *chg_dev, bool *en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_05);
    if (ret)
        return ret;

    *en = (((reg_val & SGM41542_EN_TIMER_MASK)>>SGM41542_EN_TIMER_SHIFT) == 1);
    return ret<0?ret:0;
}

static int sgm41542_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
    switch (event) {
        case EVENT_EOC:
            charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
            break;
        case EVENT_RECHARGE:
            charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
            break;
        default:
            break;
    }

    return 0;
}

static int sgm41542_enable_otg(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    reg_val = ((en?1:0)<<SGM41542_OTG_CONFIG_SHIFT) & SGM41542_OTG_CONFIG_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_01, SGM41542_OTG_CONFIG_MASK, reg_val);
    return ret;
}

static int sgm41542_set_otg_current(struct charger_device *chg_dev, u32 uA)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    uA = 1200000;
    reg_val = ((uA>1200000?1:0)<<SGM41542_BOOST_LIM_SHIFT) & SGM41542_BOOST_LIM_MASK;
    ret = sgm41542_update_bits(chip, SGM41542_REG_02, SGM41542_BOOST_LIM_MASK, reg_val);

    return ret;
}

static int sgm41542_plug_in(struct charger_device *chg_dev)
{
    int ret;

    ret = sgm41542_enable_otg(chg_dev, false);
    if (ret)
        return ret;

    ret = sgm41542_enable_charging(chg_dev, true);
    return ret;
}

static int sgm41542_plug_out(struct charger_device *chg_dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;

    chip->max_pe_vol = 5000;
    ret = sgm41542_set_hz_mode(chg_dev, false);
    if (ret)
        return ret;
    ret = sgm41542_enable_charging(chg_dev, false);

    return ret;
}

static int sgm41542_enable_shipping_mode(struct charger_device *chg_dev, bool en)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&chg_dev->dev);
    int ret;
    unsigned char reg_val;

    reg_val = ((en?1:0)<<SGM41542_BATFET_DIS_SHIFT) & SGM41542_BATFET_DIS_MASK;

    ret = sgm41542_update_bits(chip, SGM41542_REG_07, SGM41542_BATFET_DIS_MASK, reg_val);
    dev_err(chip->dev, "[%s] enter shipping mode\n", __func__);

    return ret;
}

static struct charger_ops sgm41542_chg_ops = {
    /* Charging */
    .dump_registers = sgm41542_dump_register,
    .enable = sgm41542_enable_charging,
    .is_enabled = sgm41542_is_charging_enable,
    .get_charging_current = sgm41542_get_ichg,
    .set_charging_current = sgm41542_set_ichg,
    .get_input_current = sgm41542_get_icl,
    .set_input_current = sgm41542_set_icl,
    .get_constant_voltage = sgm41542_get_vchg,
    .set_constant_voltage = sgm41542_set_vchg,
    .kick_wdt = sgm41542_kick_wdt,
    .set_mivr = sgm41542_set_mivr,
    .is_tune_done = sgm41542_is_tune_done,
    .current_tune_done = sgm41542_current_tune_done,
    .is_charging_done = sgm41542_is_charging_done,
    .get_min_charging_current = sgm41542_get_min_ichg,
    .enable_chg_type_det = sgm41542_update_chg_type,
    .enable_hz = sgm41542_set_hz_mode,
    .enable_safety_timer = sgm41542_set_safety_timer,
    .is_safety_timer_enabled = sgm41542_is_safety_timer_enabled,
    .plug_in = sgm41542_plug_in,
    .plug_out = sgm41542_plug_out,
    .event = sgm41542_do_event,

    /* OTG */
    .enable_otg = sgm41542_enable_otg,
    .set_boost_current_limit = sgm41542_set_otg_current,

    /* Shipping mode */
    .enable_shipping_mode = sgm41542_enable_shipping_mode,
};

static int sgm41542_check_device(struct sgm41542_chip *chip)
{
    int ret;
    unsigned char reg_val;

    ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_0B);
    if (!ret && (reg_val & SGM41542_PN_MASK)>>SGM41542_PN_SHIFT==SGM41542_PN)
        return 0;

    dev_err(chip->dev, "[%s] failed to get chip PN!(ret:%d)\n",
            __func__, ret);

    return -ENODEV;
}

static void sgm41542_charger_reset_pe_setting(struct sgm41542_chip *chip)
{
    chip->pe.tune_done = true;
    chip->pe.tune_rework = false;
    chip->pe.tune_count = 0;
    chip->pe.target_vol = chip->default_target_vol;
}

static void sgm41542_charger_irq_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(work, struct sgm41542_chip, irq_work);
    int ret;

    dev_info(chip->dev, "[%s] irq handle!\n", __func__);

    ret = sgm41542_dump_register(chip->chg_dev);
    if (ret) {
        dev_warn(chip->dev, "[%s] failed to dump regs!\n", __func__);
    }

    if (sgm41542_get_plugged(chip)) {
        schedule_work(&chip->in_work);
    } else {
        schedule_work(&chip->out_work);
    }
}

static void sgm41542_charger_in_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(work, struct sgm41542_chip, in_work);

    dev_info(chip->dev, "[%s] handle!\n", __func__);
}

static void sgm41542_charger_out_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(work, struct sgm41542_chip, out_work);

    dev_info(chip->dev, "[%s] handle!\n", __func__);
}

static void sgm41542_pe_tune_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(work, struct sgm41542_chip, pe_tune_work);

    if (!chip->pe.tune_done) {//make sure pe check work stopped and adapter release
        chip->pe.tune_rework = true;
        msleep(10*1000);
    }
    sgm41542_charger_reset_pe_setting(chip);
    chip->pe.tune_done = false;
    schedule_work(&chip->check_pe_work);
}

static int sgm41542_handle_pe_tune_result(struct sgm41542_chip *chip)
{
    int ret, vbus;
    union power_supply_propval propval;
    struct power_supply *chrdet_psy;

    vbus = battery_get_vbus();//mV

    if (vbus < 4000) {
        dev_err(chip->dev, "[%s] vbus %dmV plugg out!\n", __func__, vbus);
        chip->pe.target_vol = 5000;
        __pm_relax(chip->pe_tune_wakelock);
        return -2;
    } else if (vbus>=6500 && vbus<8100) {//6.5~8.1=>PE 7V
        dev_err(chip->dev, "[%s] vbus %dmV reset to 5V!\n", __func__, vbus);
        sgm41542_charger_reset_pe_setting(chip);
        chip->pe.target_vol = 5000;
        chip->pe.tune_done = false;
        if (chip->max_pe_vol < 7000)
            chip->max_pe_vol = 7000;
        schedule_work(&chip->check_pe_work);
        return -1;
    }

    chrdet_psy = power_supply_get_by_name("charger");
    if (!chrdet_psy) {
        dev_err(chip->dev, "[%s] fail to get psy 'charger'!\n", __func__);
        chip->pe.tune_done = true;
        __pm_relax(chip->pe_tune_wakelock);
        return -EINVAL;
    }

    ret = power_supply_get_property(chrdet_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
    if (ret) {
        dev_err(chip->dev,
                "[%s] fail to get charge type from psy 'charger'!\n", __func__);
        chip->pe.tune_done = true;
        __pm_relax(chip->pe_tune_wakelock);
        return ret;
    }

    dev_info(chip->dev, "[%s] charge_type:%d, vbus:%d\n", __func__, propval.intval, vbus);

    if (vbus<6500 && propval.intval!=CHARGER_PD_5V && chip->max_pe_vol>5000) {
        propval.intval = CHARGER_PE_5V;
        ret = power_supply_set_property(chrdet_psy,
                POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
    } else if (vbus>8100 && vbus<10500) {
        if (propval.intval != CHARGER_PD_9V) {
            propval.intval = CHARGER_PE_9V;
            if (chip->max_pe_vol < 9000)
                chip->max_pe_vol = 9000;
            ret = power_supply_set_property(chrdet_psy,
                    POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
        }
    } else if (vbus>=10500) {
        propval.intval = CHARGER_PE_12V;
        if (chip->max_pe_vol < 12000)
            chip->max_pe_vol = 12000;
        ret = power_supply_set_property(chrdet_psy,
                POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
    }

    chip->pe.tune_done = true;
    __pm_relax(chip->pe_tune_wakelock);
    return ret;
}

static void sgm41542_pe_up_once(struct sgm41542_chip *chip)
{
    int ret, timeout=500;
    unsigned char reg_val;

    ret = sgm41542_update_bits(chip, SGM41542_REG_0D, SGM41542_PUMPX_UP_MASK,
            1<<SGM41542_PUMPX_UP_SHIFT);
    if (ret)
        return;

    while (timeout > 0) {
        timeout -= 20;
        msleep(20);
        ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_0D);
        if (ret)
            return;

        if ((reg_val & SGM41542_PUMPX_UP_MASK)>>SGM41542_PUMPX_UP_SHIFT == 0)
            break;
    }

    return;
}

static void sgm41542_pe_down_once(struct sgm41542_chip *chip)
{
    int ret, timeout=500;
    unsigned char reg_val;

    ret = sgm41542_update_bits(chip, SGM41542_REG_0D, SGM41542_PUMPX_DN_MASK,
            1<<SGM41542_PUMPX_DN_SHIFT);
    if (ret)
        return;

    while (timeout > 0) {
        timeout -= 20;
        msleep(20);
        ret = sgm41542_read_byte(chip, &reg_val, SGM41542_REG_0D);
        if (ret)
            return;

        if ((reg_val & SGM41542_PUMPX_DN_MASK)>>SGM41542_PUMPX_DN_SHIFT == 0)
            break;
    }

    return;
}

static void sgm41542_check_pe_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(work, struct sgm41542_chip, check_pe_work);
    int vbus, timeout = 4000;

    if (chip->pe.tune_rework) {
        dev_info(chip->dev, "[%s] giveup, rework!\n", __func__);
        return;
    }

    vbus = battery_get_vbus();//mV
    if (vbus<chip->pe.target_vol-500) {//tune up
        sgm41542_pe_up_once(chip);
    } else if (vbus > chip->pe.target_vol+500) {//tune down
        sgm41542_pe_down_once(chip);
    } else {//done
        dev_info(chip->dev, "[%s] pe vbus tune successed!\n", __func__);
        sgm41542_handle_pe_tune_result(chip);
        return;
    }

    while (timeout > 0) {
        msleep(500);
        timeout -= 500;
        if (abs(battery_get_vbus()-vbus) > 1500)
            break;
    }
    msleep(500);
    dev_info(chip->dev, "[%s] after tune, vbus:%dmv=>%dmv\n", __func__, vbus,
            battery_get_vbus());

    if (chip->pe.tune_count >= 10) {//maybe fail
        dev_warn(chip->dev, "[%s] reach max pe tune times!\n", __func__);
        sgm41542_handle_pe_tune_result(chip);
        return;
    }

    chip->pe.tune_count++;
    schedule_work(&chip->check_pe_work);
}

static void sgm41542_monitor_workfunc(struct work_struct *work)
{
    struct sgm41542_chip *chip = container_of(to_delayed_work(work),
            struct sgm41542_chip, monitor_work);
    int vbus, bat_current, soc, chg_type, ret;
    union power_supply_propval propval;
    struct power_supply *chrdet_psy, *batt_psy;
    bool hiz_en;

    if (chip->suspend_flag) {
        dev_info(chip->dev, "[%s] suspend, bypass\n", __func__);
        goto out;
    }

    chrdet_psy = power_supply_get_by_name("charger");
    if (!chrdet_psy) {
        dev_err(chip->dev, "[%s] get chr power supply failed\n", __func__);
        goto out;
    }

    ret = power_supply_get_property(chrdet_psy,
            POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
    if (ret) {
        dev_err(chip->dev, "[%s] get charge type failed\n", __func__);
        goto out;
    }
    chg_type = propval.intval;
    if (chg_type == CHARGER_UNKNOWN) {
        dev_info(chip->dev, "[%s] no charger, bypass\n", __func__);
        sgm41542_dump_register(chip->chg_dev);
        charger_dev_dump_registers(get_charger_by_name("primary_divider_chg"));
        goto out;
    }

    batt_psy = power_supply_get_by_name("battery");
    if (!batt_psy) {
        dev_err(chip->dev, "[%s] get bat power supply failed\n", __func__);
        goto out;
    }

    ret = power_supply_get_property(batt_psy,
            POWER_SUPPLY_PROP_CURRENT_AVG, &propval);
    if (ret) {
        dev_err(chip->dev, "[%s] get bat curr_avg failed\n", __func__);
        goto out;
    }
    bat_current = propval.intval/1000;

    ret = power_supply_get_property(batt_psy,
            POWER_SUPPLY_PROP_CAPACITY, &propval);
    if (ret) {
        dev_err(chip->dev, "[%s] get bat capacity failed\n", __func__);
        goto out;
    }
    soc = propval.intval;

    sgm41542_get_hz_mode(chip->chg_dev, &hiz_en);
    vbus = battery_get_vbus();
    dev_err(chip->dev, "[%s] vbus:%dmv ibat:%dma soc:%d chg_type:%d tune_done:%d "
            "hiz:%d\n", __func__, vbus, bat_current, soc, chg_type, chip->pe.tune_done,
            hiz_en);

    if(!chip->pe.tune_done)
        goto out;
    if ((vbus<5500 || (soc>85 && bat_current<1000))
            && (chg_type==CHARGER_PE_9V || chg_type==CHARGER_PE_12V)) {
        sgm41542_charger_reset_pe_setting(chip);
        chip->pe.tune_done = false;
        chip->pe.target_vol = 5000;
        if (!chip->pe_tune_wakelock->active)
            __pm_stay_awake(chip->pe_tune_wakelock);
        schedule_work(&chip->check_pe_work);
    } else if (vbus>14000 && chg_type==CHARGER_PE_12V) {
        sgm41542_charger_reset_pe_setting(chip);
        chip->pe.tune_done = false;
        chip->pe.target_vol = 9000;
        if (!chip->pe_tune_wakelock->active)
            __pm_stay_awake(chip->pe_tune_wakelock);
        schedule_work(&chip->check_pe_work);
    } else if (bat_current>1500 && chg_type==CHARGER_PE_5V && chip->max_pe_vol>7000) {
        sgm41542_charger_reset_pe_setting(chip);
        chip->pe.tune_done = false;
        if (!chip->pe_tune_wakelock->active)
            __pm_stay_awake(chip->pe_tune_wakelock);
        schedule_work(&chip->check_pe_work);
    }

out:
    schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(10*1000));
}

/*
static irqreturn_t sgm41542_charger_interrupt(int irq, void *data)
{
    struct sgm41542_chip *chip = data;

    schedule_work(&chip->irq_work);
    return IRQ_HANDLED;
}
*/
static int sgm41542_parse_dt(struct device *dev, struct sgm41542_chip *chip)
{
    int ret;

    ret = of_property_read_u32(dev->of_node, "default_pe_target_vbus", &chip->default_target_vol);
    if (ret) {
        dev_err(chip->dev, "[%s] failed to read 'default_pe_target_vbus'!\n", __func__);
        chip->default_target_vol = 5000;
    }

    chip->sgm41542_dpdm_sel = of_get_named_gpio(chip->dev->of_node, "sgm41542_dpdm_sel", 0);
    if (chip->sgm41542_dpdm_sel < 0) {
        dev_err(chip->dev,
                "[%s] failed to request gpio%d!\n", __func__, chip->sgm41542_dpdm_sel);
        ret = -EINVAL;
        goto out;
    }

    ret = gpio_request(chip->sgm41542_dpdm_sel, "sgm41542_dpdm_sel");
    if (ret) {
        dev_err(chip->dev,
                "[%s] failed to request gpio%d!\n", __func__, chip->sgm41542_dpdm_sel);
        ret = -EINVAL;
        goto out;
    }
    gpio_direction_output(chip->sgm41542_dpdm_sel, 0);

    chip->sgm41542_irq = of_get_named_gpio(chip->dev->of_node, "sgm41542_irq", 0);
    if (chip->sgm41542_irq < 0) {
        dev_err(chip->dev,
                "[%s] failed to request gpio%d!\n", __func__, chip->sgm41542_irq);
        ret = -EINVAL;
        goto free_dpdm;
    }

    ret = gpio_request(chip->sgm41542_irq, "sgm41542_irq");
    if (ret) {
        dev_err(chip->dev,
                "[%s] failed to request gpio%d!\n", __func__, chip->sgm41542_irq);
        ret = -EINVAL;
        goto free_dpdm;
    }

    gpio_direction_input(chip->sgm41542_irq);
    ret = gpio_to_irq(chip->sgm41542_irq);
    if (ret < 0) {
        dev_err(chip->dev, "[%s] gpio_to_irq failed ret:%d!\n", __func__, ret);
        ret = -EINVAL;
        goto free_irq;
    }
    chip->client->irq = ret;

    return 0;

free_irq:
    gpio_free(chip->sgm41542_irq);
free_dpdm:
    gpio_free(chip->sgm41542_dpdm_sel);
out:
    return ret;
}

static int sgm41542_init_device(struct sgm41542_chip *chip)
{
    int ret;
    /* term current 300mA */
    ret = sgm41542_update_bits(chip, SGM41542_REG_03, SGM41542_ITERM_MASK,
            ((300-60)/60)<<SGM41542_ITERM_SHIFT);
    if (ret)
        return ret;
    /* disable wdt */
    ret = sgm41542_update_bits(chip, SGM41542_REG_05, SGM41542_WATCHDOG_MASK,
            0<<SGM41542_WATCHDOG_SHIFT);
    if (ret)
        return ret;
    /* disable safe timer(use sw safe timer 7x24x3600s) */
    ret = sgm41542_update_bits(chip, SGM41542_REG_05, SGM41542_EN_TIMER_MASK,
            0<<SGM41542_EN_TIMER_SHIFT);
    if (ret)
        return ret;
    /* enable pumpx */
    ret = sgm41542_update_bits(chip, SGM41542_REG_0D, SGM41542_EN_PUMPX_MASK,
            1<<SGM41542_EN_PUMPX_SHIFT);
    if (ret)
        return ret;

    ret = sgm41542_update_bits(chip, SGM41542_REG_07, SGM41542_BATFET_DLY_MASK,
            1<<SGM41542_BATFET_DLY_SHIFT);
    if (ret)
        return ret;
    dev_info(chip->dev, "[%s] finish!\n", __func__);

    return ret;
}

static int sgm41542_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct sgm41542_chip *chip;
    int ret;

    dev_info(&client->dev, "[%s] start!\n", __func__);
    chip = devm_kzalloc(&client->dev, sizeof(struct sgm41542_chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev, "[%s] alloc memery failed, may OOM!\n", __func__);
        ret = -ENOMEM;
        goto out;
    }

    chip->dev = &client->dev;
    chip->client = client;
    chip->suspend_flag = false;
    i2c_set_clientdata(client, chip);

    mutex_init(&chip->sgm41542_i2c_lock);
    mutex_init(&chip->sgm41542_update_ct_lock);

    ret = sgm41542_check_device(chip);
    if (ret) {
        dev_err(chip->dev, "[%s] check device sgm41542 failed!\n", __func__);
        goto out;
    }
    hardwareinfo_set_prop(HARDWARE_CHARGER_IC_INFO, "SGM41542");

    ret = sgm41542_init_device(chip);
    if (ret) {
        dev_err(chip->dev, "[%s] init device sgm41542 failed!\n", __func__);
        goto out;
    }

    ret = sgm41542_parse_dt(chip->dev, chip);
    if (ret) {
        dev_err(chip->dev, "[%s] parse device-tree failed!\n", __func__);
        goto out;
    }

    chip->chg_props.alias_name = "sgm41542";
    chip->chg_dev = charger_device_register("primary_chg", chip->dev, chip,
                        &sgm41542_chg_ops, &chip->chg_props);
    if (IS_ERR_OR_NULL(chip->chg_dev)) {
        ret = PTR_ERR(chip->chg_dev);
        dev_err(chip->dev, "[%s] failed to register charger device!ret:%d\n", __func__, ret);
        goto free_gpio;
    }

    ret = sgm41542_dump_register(chip->chg_dev);
    if (ret) {
        dev_err(chip->dev, "[%s] failed to dump regs!ret:%d\n", __func__, ret);
        goto free_gpio;
    }

    sgm41542_charger_reset_pe_setting(chip);
    chip->max_pe_vol = 5000;
	chip->pe_tune_wakelock = wakeup_source_register(NULL, "sgm41542 pe tune");

    INIT_WORK(&chip->irq_work, sgm41542_charger_irq_workfunc);
    INIT_WORK(&chip->in_work, sgm41542_charger_in_workfunc);
    INIT_WORK(&chip->out_work, sgm41542_charger_out_workfunc);
    INIT_WORK(&chip->pe_tune_work, sgm41542_pe_tune_workfunc);
    INIT_WORK(&chip->check_pe_work, sgm41542_check_pe_workfunc);
    INIT_DELAYED_WORK(&chip->monitor_work, sgm41542_monitor_workfunc);

    schedule_delayed_work(&chip->monitor_work, msecs_to_jiffies(0));
/*
    ret = request_irq(chip->client->irq, sgm41542_charger_interrupt,
            IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "sgm41542_irq", chip);
    if (ret) {
        dev_err(chip->dev, "[%s] failed to request_irq ret:%d!\n", __func__, ret);
        ret = -EINVAL;
        goto free_works;
    }
*/
    schedule_work(&chip->irq_work);
    dev_info(chip->dev, "[%s] finish!\n", __func__);

    return ret;
/*free_works:
    cancel_work_sync(&chip->irq_work);
    cancel_work_sync(&chip->in_work);
    cancel_work_sync(&chip->out_work);
    cancel_work_sync(&chip->pe_tune_work);
    cancel_work_sync(&chip->check_pe_work);*/
free_gpio:
    gpio_free(chip->sgm41542_irq);
out:
    return -ENODEV;
}

static void sgm41542_charger_shutdown(struct i2c_client *client)
{
    struct sgm41542_chip *chip = dev_get_drvdata(&client->dev);
    int ret;

    //ret = sgm41542_plug_out(chip->chg_dev);
    ret = sgm41542_sw_reset(chip->chg_dev);
    ret = sgm41542_set_ichg(chip->chg_dev, 0);
}

#ifdef CONFIG_PM_SLEEP
static int sgm41542_charger_suspend(struct device *dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "[%s]\n", __func__);
    chip->suspend_flag = true;
    return 0;
}

static int sgm41542_charger_resume(struct device *dev)
{
    struct sgm41542_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "[%s]\n", __func__);
    chip->suspend_flag = false;
    return 0;
}
static SIMPLE_DEV_PM_OPS(sgm41542_charger_pm_ops, sgm41542_charger_suspend,
    sgm41542_charger_resume);
#endif

static struct of_device_id sgm41542_charger_match_table[] = {
    {.compatible = "sgm,sgm41542",},
    {},
};

static const struct i2c_device_id sgm41542_charger_id[] = {
    {"sgm41542", 0x00},
    {},
};
MODULE_DEVICE_TABLE(i2c, sgm41542_charger_id);

static struct i2c_driver sgm41542_charger_driver = {
    .driver = {
        .name = "sgm41542",
        .of_match_table = sgm41542_charger_match_table,
#ifdef CONFIG_PM_SLEEP
        .pm = &sgm41542_charger_pm_ops,
#endif
    },
    .id_table = sgm41542_charger_id,
    .probe = sgm41542_charger_probe,
    .shutdown = sgm41542_charger_shutdown,
};
module_i2c_driver(sgm41542_charger_driver);

MODULE_DESCRIPTION("SGM sgm41542 Charger Driver");
MODULE_LICENSE("GPL");
