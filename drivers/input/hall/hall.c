/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef CONFIG_PM_SLEEP
//#include <linux/pm_wakeup.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "linux/hardware_info.h"
#define HALL_NAME	"hall"

struct input_dev *hall_input_dev;

#define MTK_GPIO_PIN_BASE  69

#define KEY_HALL_DOWN  252
#define KEY_HALL_UP  253
extern char hallname[HARDWARE_MAX_ITEM_LONGTH];
static int hall_pdrv_probe(struct platform_device *pdev);
static int hall_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int hall_pdrv_resume(struct platform_device *pdev);
static struct platform_driver hall_pdrv;

struct delayed_work hall_delayed_work;

void hall_delayed_work_func(struct work_struct *work)
{
    
}

struct hall_priv {
	struct work_struct eint_work;
	struct device_node *irq_node;
	int irq;
        int gpio;
};

static struct hall_priv *hall_obj;


static irqreturn_t hall_irq_handler(int irq, void *desc)
{
        printk(KERN_ERR"%s\n", __func__);
	disable_irq_nosync(hall_obj->irq);

	schedule_work(&hall_obj->eint_work);

	return IRQ_HANDLED;
}

static void hall_eint_work(struct work_struct *work)
{
    int val;
    val = gpio_get_value(hall_obj->gpio + MTK_GPIO_PIN_BASE);
    printk(KERN_ERR"%s, val=%d\n", __func__, val);

    if(val != 0)
    {
        input_report_key(hall_input_dev, KEY_HALL_UP, 1);
        input_sync(hall_input_dev);
        input_report_key(hall_input_dev, KEY_HALL_UP, 0);
        input_sync(hall_input_dev);

	irq_set_irq_type(hall_obj->irq, IRQF_TRIGGER_FALLING);
    }
    else
    {
        input_report_key(hall_input_dev, KEY_HALL_DOWN, 1);
        input_sync(hall_input_dev);
        input_report_key(hall_input_dev, KEY_HALL_DOWN, 0);
        input_sync(hall_input_dev);

	irq_set_irq_type(hall_obj->irq, IRQF_TRIGGER_RISING);
    }

    enable_irq(hall_obj->irq);
}



static ssize_t hall_status_show(struct device_driver *ddri, char *buf)
{
    int val;
    val = gpio_get_value(hall_obj->gpio + MTK_GPIO_PIN_BASE);
    printk(KERN_ERR"%s, val=%d\n", __func__, val);
    return sprintf(buf, "%d\n", val);
}
//static DRIVER_ATTR(hall_status, S_IRUGO, hall_show_hall_status, NULL);
static DRIVER_ATTR_RO(hall_status);

static struct driver_attribute *hall_attr_list[] = {
   &driver_attr_hall_status,
};

static int hall_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = sizeof(hall_attr_list) / sizeof(hall_attr_list[0]);
    
    printk(KERN_ERR"%s\n", __func__);
    if(driver == NULL)
    {
        return -EINVAL;    
    }
    for (idx = 0; idx < num; idx ++)
    {
        err = driver_create_file(driver, hall_attr_list[idx]);
        printk(KERN_ERR"driver_create_file(%s) success\n", hall_attr_list[idx]->attr.name);
        if (err < 0)
        {
            printk(KERN_ERR"driver_create_file(%s) fail\n", hall_attr_list[idx]->attr.name);
            break;
        }
    }
    return 0;
}


static int of_get_hall_dt(struct device *dev)
{
    struct device_node *node = dev->of_node;
    u32 gpio_config[2] = {0, 0};
    int debounce = 0;
    
    printk(KERN_ERR"%s\n", __func__);

    if(node)
    {  
        of_property_read_u32_array(node, "interrupts", gpio_config, ARRAY_SIZE(gpio_config));
        hall_obj->gpio = gpio_config[0];
        printk(KERN_ERR"%s,hall gpio: %d\n", __func__, hall_obj->gpio); 

	hall_obj->irq = irq_of_parse_and_map(node, 0);
	if (!hall_obj->irq) {
		printk("hall_obj irq failed\n");
		return -ENODEV;
	}
	printk(KERN_ERR"%s,hall irq: %d\n", __func__, hall_obj->irq);
    }
    return 0;
}

static int hall_pdrv_probe(struct platform_device *pdev)
{
        struct hall_priv *obj;
	u32 i;
	int32_t err = 0;
        int val = 0;

        printk(KERN_ERR"%s in\n", __func__);
	obj = devm_kzalloc(&pdev->dev, sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	hall_obj = obj;

	if (!pdev->dev.of_node) {
		printk(KERN_ERR"no hall dev node\n");
		return -ENODEV;
	}
        
        of_get_hall_dt(&pdev->dev);

	hall_input_dev = devm_input_allocate_device(&pdev->dev);
	if (!hall_input_dev) {
		printk(KERN_ERR"input allocate device fail.\n");
		return -ENOMEM;
	}

	hall_input_dev->name = HALL_NAME;
         /*add hall hardwareinfo*/
        strcpy(hallname,"AS1800WRN-W");//MAPLE-450,zhanghao3.wt,mod,20211102,for hall sensor hardware info

	__set_bit(EV_KEY, hall_input_dev->evbit);
        input_set_capability(hall_input_dev, EV_KEY, KEY_HALL_DOWN);
        input_set_capability(hall_input_dev, EV_KEY, KEY_HALL_UP);

	err = input_register_device(hall_input_dev);
	if (err) {
		printk(KERN_ERR"register input device failed (%d)\n", err);
		return err;
	}

        INIT_WORK(&obj->eint_work, hall_eint_work);
        hall_create_attr(&(hall_pdrv.driver));
        
	err = request_irq(hall_obj->irq, hall_irq_handler, IRQF_TRIGGER_FALLING,
			"hall_irq", NULL);
	if (err) {
		printk(KERN_ERR"register IRQ failed (%d)\n", err);
		input_unregister_device(hall_input_dev);
		return err;
	}

	if (enable_irq_wake(hall_obj->irq) < 0)
		printk(KERN_ERR"hall irq %d enable irq wake fail\n", hall_obj->irq);

	printk(KERN_ERR"%s OK.\n", __func__);
exit:
	return err;
}

static int hall_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int hall_pdrv_resume(struct platform_device *pdev)
{
	return 0;
}
static int hall_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id hall_of_match[] = {
	{.compatible = "mediatek,hall_1"},
	{},
};

static struct platform_driver hall_pdrv = {
	.probe = hall_pdrv_probe,
	.suspend = hall_pdrv_suspend,
	.resume = hall_pdrv_resume,
        .remove = hall_pdrv_remove,
	.driver = {
		   .name = HALL_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = hall_of_match,
		   },
};

static int __init hall_init(void)
{
     printk(KERN_ERR"%s\n", __func__);
     platform_driver_register(&hall_pdrv);
     return 0;
}
static void __exit hall_exit(void)
{
     printk(KERN_ERR"%s\n", __func__);
     platform_driver_unregister(&hall_pdrv);
}

module_init(hall_init);
module_exit(hall_exit);
//module_platform_driver(hall_pdrv);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("Hall Driver");
MODULE_LICENSE("GPL");
