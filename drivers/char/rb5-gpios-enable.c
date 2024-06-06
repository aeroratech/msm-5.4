// SPDX-License-Identifier: GPL-2.0-only

/**
 * Driver for enable gpio 140/145,116/117 on QRB5165.
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/kernel.h>

#define DEV_NAME "rb5_gpios_enable"

/* Active devices */
static DEFINE_MUTEX(rb5_gpios_devnode_lock);
static DECLARE_BITMAP(rb5_gpios_devnode_nums, 256);

static int rb5_gpios_enable_major;
static struct class *rb5_gpios_enable_class;

static const struct file_operations rb5_gpios_enable_fops = {
	.owner		= THIS_MODULE,
};

static int rb5_gpios_enable_probe(struct platform_device *pdev)
{
	struct device *dev = NULL;
	struct device_node *np = pdev->dev.of_node;
	const char* dname = dev_name(&pdev->dev);
	int minor;
	u32 enable_gpio;

	pr_debug("probe %s\n", dname);

	mutex_lock(&rb5_gpios_devnode_lock);
	minor = find_next_zero_bit(rb5_gpios_devnode_nums, 256, 0);
	if (minor == 256) {
		mutex_unlock(&rb5_gpios_devnode_lock);
		pr_err(DEV_NAME "could not get a free minor\n");
		return -ENFILE;
	}
	set_bit(minor, rb5_gpios_devnode_nums);
	mutex_unlock(&rb5_gpios_devnode_lock);

	dev = device_create(rb5_gpios_enable_class, NULL, MKDEV(rb5_gpios_enable_major, minor), NULL, dname);
	if (IS_ERR(dev)) {
		pr_err(DEV_NAME ": failed to create device %s\n", dname);
		mutex_lock(&rb5_gpios_devnode_lock);
		clear_bit(minor, rb5_gpios_devnode_nums);
		mutex_unlock(&rb5_gpios_devnode_lock);
		return PTR_ERR(dev);
	}

	enable_gpio = of_get_named_gpio(np, "qcom,enable-gpio", 0);
	if (!gpio_is_valid(enable_gpio)) {
		pr_err("%s qcom,enable-gpio not specified\n", __func__);
		goto error;
	}
	if (gpio_request(enable_gpio, "qcom,enable-gpio")) {
		pr_err("qcom,enable-gpio request failed\n");
		goto error;
	}
	gpio_direction_output(enable_gpio, 1);
	gpio_set_value(enable_gpio, 0);
	pr_debug("%s gpio:%d set to low\n", __func__);
	return 0;

error:
	gpio_free(enable_gpio);

	mutex_lock(&rb5_gpios_devnode_lock);
	clear_bit(minor, rb5_gpios_devnode_nums);
	mutex_unlock(&rb5_gpios_devnode_lock);

	return -EINVAL;
}

static int rb5_gpios_enable_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char* dname = dev_name(&pdev->dev);
	u32 enable_gpio = of_get_named_gpio(np, "qcom,enable-gpio", 0);
	pr_debug("remove %s\n", dname);
	gpio_free(enable_gpio);
	device_destroy(rb5_gpios_enable_class, MKDEV(rb5_gpios_enable_major, 0));
	class_destroy(rb5_gpios_enable_class);
	unregister_chrdev(rb5_gpios_enable_major, dname);
	return 0;
}

static const struct of_device_id of_rb5_gpios_enable_dt_match[] = {
	{.compatible	= "qcom,rb5_gpios_enable"},
	{},
};

MODULE_DEVICE_TABLE(of, of_rb5_gpios_enable_dt_match);

static struct platform_driver rb5_gpios_enable_driver = {
	.probe	= rb5_gpios_enable_probe,
	.remove	= rb5_gpios_enable_remove,
	.driver	= {
		.name	= "rb5_gpios_enable",
		.of_match_table	= of_rb5_gpios_enable_dt_match,
	},
};

static int __init rb5_gpios_enable_init(void)
{
	int ret;

	pr_debug(DEV_NAME " init\n");

	rb5_gpios_enable_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(rb5_gpios_enable_class))
		return PTR_ERR(rb5_gpios_enable_class);

	rb5_gpios_enable_major = register_chrdev(0, DEV_NAME, &rb5_gpios_enable_fops);
	if (rb5_gpios_enable_major < 0) {
		class_destroy(rb5_gpios_enable_class);
		return -EIO;
	}

	ret = platform_driver_register(&rb5_gpios_enable_driver);
	if (ret) {
		unregister_chrdev(rb5_gpios_enable_major, DEV_NAME);
		class_destroy(rb5_gpios_enable_class);
	}

	return ret;
}

static void __exit rb5_gpios_enable_exit(void)
{
	pr_debug(DEV_NAME " exit\n");
	platform_driver_unregister(&rb5_gpios_enable_driver);
	unregister_chrdev(rb5_gpios_enable_major, DEV_NAME);
	class_destroy(rb5_gpios_enable_class);
}

subsys_initcall(rb5_gpios_enable_init);
module_exit(rb5_gpios_enable_exit);

MODULE_DESCRIPTION("Driver to enable rb5 gpio 140/145,116/117");
MODULE_LICENSE("GPL v2");
