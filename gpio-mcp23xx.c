/*
 *  mcp23xx - Microchip's SPI GPIO expander series driver
 *
 *  Copyright (C) 2017 Nicolas Saenz Julienne  <nicolassaenzj@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/gpio/driver.h>

#define MCP23XX_REG_IODIR	0x0
#define MCP23XX_REG_IOCON	0x5
#define MCP23XX_REG_GPPU	0x6
#define MCP23XX_REG_GPIO	0x9
#define MCP23XX_REG_OLAT	0xA

struct mcp23xx {
	struct gpio_chip gc;
	struct regmap *regmap;
};

struct mcp23xx_chip_info {
	unsigned int ngpio;
};

enum ad7476_supported_device_ids {
	ID_MCP23S08,
};

static const struct mcp23xx_chip_info mcp23xx_spi_chip_info[] = {
	[ID_MCP23S08] = {
		.ngpio = 8,
	},
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,6,0)
#define gpiochip_get_data(_gc)		container_of(_gc, struct mcp23xx, gc)
#endif

static int mcp23xx_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	unsigned int val;

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_read(mcp23xx->regmap, MCP23XX_REG_GPIO, &val);

	return !!(val & offset);
}

static void mcp23xx_set_value(struct gpio_chip *gc,
			      unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_update_bits(mcp23xx->regmap, MCP23XX_REG_GPIO, 1 << offset, val);
}

static void mcp23xx_set_multiple(struct gpio_chip *gc, unsigned long *mask,
				 unsigned long *bits)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_update_bits(mcp23xx->regmap, MCP23XX_REG_GPIO, *mask, *bits);
}

static int mcp23xx_direction_output(struct gpio_chip *gc,
				    unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_update_bits(mcp23xx->regmap, MCP23XX_REG_IODIR, 1 << offset, 0);
	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_update_bits(mcp23xx->regmap, MCP23XX_REG_GPIO, 1 << offset, val);
	return 0;
}

static int mcp23xx_direction_input(struct gpio_chip *gc,
				   unsigned offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_update_bits(mcp23xx->regmap, MCP23XX_REG_IODIR, 1 << offset, 1);
	return 0;
}

static int mcp23xx_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	unsigned int val;

	pr_info("%s, %d\n", __func__, __LINE__);
	regmap_read(mcp23xx->regmap, MCP23XX_REG_IODIR, &val);

	return !!(val & (1 << offset));
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
static int mcp23xx_gpio_set_config(struct gpio_chip *gc, unsigned offset,
				   unsigned long config)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	enum pin_config_param param = pinconf_to_config_param(config);

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		reg_update_bits(mcp23xx->regmap, MCP23XX_REG_GPPU, 1 << offset, 0);
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		reg_update_bits(&mcp24xx->regmap, MCP23XX_REG_GPPU, 1 << offset, 1);
		break;
	default:
		return -ENOTSUPP;

	}
}
#endif

static bool mcp23xx_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MCP23XX_REG_GPIO:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config mcp23xx_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MCP23XX_REG_OLAT,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
	.cache_type = REGCACHE_GPIO,
#endif
	.volatile_reg = mcp23xx_volatile_reg,
};

static int mcp23xx_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct mcp23xx_chip_info *info =
				&mcp23xx_spi_chip_info[id->driver_data];
	struct mcp23xx *mcp23xx;
	int ret;

	mcp23xx = devm_kzalloc(&spi->dev, sizeof(*mcp23xx), GFP_KERNEL);
	if (!mcp23xx)
		return -ENOMEM;

	spi_set_drvdata(spi, mcp23xx);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
	mcp23xx->gc.parent = &spi->dev;
#endif
	mcp23xx->gc.owner = THIS_MODULE;
	mcp23xx->gc.get = mcp23xx_get_value;
	mcp23xx->gc.set = mcp23xx_set_value;
	mcp23xx->gc.direction_output = mcp23xx_direction_output;
	mcp23xx->gc.direction_input = mcp23xx_direction_input;
	mcp23xx->gc.get_direction = mcp23xx_get_direction;
	mcp23xx->gc.set_multiple = mcp23xx_set_multiple;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
	mcp23xx->gc.set_config = mcp23xx_gpio_set_config;
#endif
	mcp23xx->gc.can_sleep = true;
	mcp23xx->gc.ngpio = info->ngpio;
	mcp23xx->gc.base = -1;

	mcp23xx->regmap = devm_regmap_init_spi(spi, &mcp23xx_regmap_cfg);
	if (IS_ERR(mcp23xx->regmap)) {
		ret = PTR_ERR(mcp23xx->regmap);
		dev_err(&spi->dev, "Failed to init regmap\n");
		goto exit;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,6,0)
	ret = gpiochip_add(&mcp23xx->gc);
#else
	ret = gpiochip_add_data(&mcp23xx->gc, mcp23xx);
#endif
	if (ret)
		dev_err(&spi->dev, "Failed to register gpiochip\n");

exit:
	return ret;
}

static int mcp23xx_remove(struct spi_device *spi)
{
	struct mcp23xx *mcp23xx = spi_get_drvdata(spi);

	gpiochip_remove(&mcp23xx->gc);

	return 0;
}

static const struct spi_device_id mcp23xx_id[] = {
	{"mcp23s08", ID_MCP23S08},
	{}
};
MODULE_DEVICE_TABLE(spi, mcp23xx_id);

static struct spi_driver mcp23xx_driver = {
	.driver = {
		.name = "mcp23xx",
	},
	.probe = mcp23xx_probe,
	.remove	= mcp23xx_remove,
	.id_table = mcp23xx_id,
};
module_spi_driver(mcp23xx_driver);

MODULE_AUTHOR("Nicolas Saenz Julienne <nicolassaenzj@gmail.com");
MODULE_DESCRIPTION("Microchip's SPI GPIO expander series driver");
MODULE_LICENSE("GPL v2");
