/*
 *  mcp23xx - Microchip's SPI GPIO expander series driver
 *
 *  Copyright (C) 2017 Nicolas Saenz Julienne  <nicolassaenzj@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>

#define MCP23XX_REG_GPIO

struct mcp23xx {
	struct gpio_chip gpio_chip;
	struct regmap regmap
};

struct mcp23xx_chip_info {
	ngpio;
};

enum ad7476_supported_device_ids {
	ID_MCP23S08,
};

static const struct mcp23xx_chip_info mcp23xx_spi_chip_info[] = {
	[ID_MCP23S08] = {
		.ngpio = 8;
	},
};

static int mcp23xx_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
}

static void mcp23xx_set_value(struct gpio_mcp23xx *gc,
			      unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
}

static void mcp23xx_set_multiple(struct gpio_mcp23xx *gc, unsigned long *mask,
				 unsigned long *bits)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
}

static int mcp23xx_direction_output(struct gpio_mcp23xx *gc,
				    unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	return 0;
}

static int mcp23xx_direction_input(struct gpio_mcp23xx *gc,
				   unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	return 0;
}

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
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = mcp23xx_volatile_reg,
};

static int mcp23xx_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct mcp23xx_chip_info *info =
				&mcp23xx_spi_chip_info[id->driver_data];
	struct mcp23xx *mcp23xx;
	u32 nregs;
	int ret;

	mcp23xx = devm_kzalloc(&spi->dev, sizeof(*mcp23xx) + nregs, GFP_KERNEL);
	if (!mcp23xx)
		return -ENOMEM;

	mcp23xx->gpio_chip.parent = &spi->dev;
	mcp23xx->gpio_chip.owner = THIS_MODULE;
	mcp23xx->gpio_chip.get = mcp23xx_get_value;
	mcp23xx->gpio_chip.set = mcp23xx_set_value;
	mcp23xx->gpio_chip.direction_output = mcp23xx_direction_output;
	mcp23xx->gpio_chip.direction_input = mcp23xx_direction_input;
	mcp23xx->gpio_chip.get_direction = mcp23xx_get_direction;
	mcp23xx->gpio_chip.set_multiple = mcp23xx_set_multiple;
	mcp23xx->gpio_chip.can_sleep = true;
	mcp23xx->gpio_chip.ngpio = info->ngpio;
	mcp23xx->gpio_chip.base = -1;

	mcp23xx->regmap = devm_regmap_init_spi(spi, &mxp23xx_regmap_cfg);
	if (IS_ERR(mcp23xx->regmap)) {
		ret = PTR_ERR(mcp23xx->regmap);
		dev_err(&spi->dev, "Failed to init regmap\n");
		goto exit;
	}

	ret = gpiochip_add_data(&chip->gpio_chip, mcp23xx);
	if (ret)
		dev_err(&spi->dev, "Failed to register gpiochip\n");

exit:
	return ret;
}

static int mcp23xx_remove(struct spi_device *spi)
{
	struct mcp23xx *mcp23xx = spi_get_drvdata(spi);

	gpiomcp23xx_remove(&mcp23xx->gpio_mcp23xx);

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
