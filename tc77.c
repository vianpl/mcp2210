/*
 * An hwmon driver for the Microchip TC77
 *
 * Copyright 2018 Nicolas Saenz Julienne <nicolassaenzj@gmail.com>
 *
 * Based on tc74.c
 *	Copyright 2015 Maciej Szmigiero <mail@maciej.szmigiero.name>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>

#define TC77_CONVERSION_TIME_MS		400
#define TC77_TEMPERATURE_DELTA		62 /* in m°C */

struct tc77 {
	struct spi_device *spi;
	struct device *hwmon;
	struct mutex lock;

	unsigned long next_update;
	int temp;
};

static int tc77_update_device(struct tc77 *tc77)
{
	int ret;

	ret = mutex_lock_interruptible(&tc77->lock);
	if (ret)
		return ret;

	/* As per the datasheet the temperature conversion time is ~400ms */
	if (time_after(jiffies, tc77->next_update)) {

		ret = spi_read(tc77->spi, (u8 *)&tc77->temp, sizeof(s16));
		if (ret < 0) {
			dev_err(&tc77->spi->dev, "SPI read error, %d\n", ret);
			goto ret_unlock;
		}
		dev_info(&tc77->spi->dev, "SPI READ 0x%x", tc77->temp);

		/*
		 * Temperature is a 13 bit signed number, we get rid of the 3
		 * LSB. Each increment accounts for 0.0625°C, yet we also know
		 * there is a ~1°C error per mesurement.
		 */
		tc77->temp >>= 3;
		tc77->temp = be16_to_cpu(tc77->temp);
		tc77->temp *= TC77_TEMPERATURE_DELTA;

		tc77->next_update =
			jiffies + msecs_to_jiffies(TC77_CONVERSION_TIME_MS);
	}

ret_unlock:
	mutex_unlock(&tc77->lock);

	return ret;
}

static ssize_t show_temp_input(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct tc77 *tc77 = dev_get_drvdata(dev);
	int ret;

	ret = tc77_update_device(tc77);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", tc77->temp * 1000);
}
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp_input, NULL, 0);

static struct attribute *tc77_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(tc77);

static int tc77_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tc77 *tc77;

	tc77 = devm_kzalloc(dev, sizeof(struct tc77), GFP_KERNEL);
	if (!tc77)
		return -ENOMEM;

	tc77->spi = spi;
	mutex_init(&tc77->lock);

	tc77->hwmon = devm_hwmon_device_register_with_groups(dev, "tc77",
							     tc77, tc77_groups);
	return PTR_ERR_OR_ZERO(tc77->hwmon);
}

static const struct spi_device_id tc77_id[] = {
	{ "tc77", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, tc77_id);

static struct spi_driver tc77_driver = {
	.driver = {
		.name	= "tc77",
	},
	.probe	= tc77_probe,
	.id_table = tc77_id,
};

module_spi_driver(tc77_driver);

MODULE_AUTHOR("Nicolas Saenz Julienne <nicolassaenzj@gmail.com>");
MODULE_DESCRIPTION("TC77 driver");
MODULE_LICENSE("GPL v2");
