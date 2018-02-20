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
#include <linux/spi/spi.h>
#include <linux/gpio/driver.h>
#include "mcp23xx.h"

#define MCP23XX_SPI_MSG_LEN	3
#define MCP23XX_READ_BIT	0x1

#define MCP23XX_REG_IODIR	0x0
#define MCP23XX_REG_IOCON	0x5
#define MCP23XX_REG_GPPU	0x6
#define MCP23XX_REG_GPIO	0x9
#define MCP23XX_REG_OLAT	0xA

struct mcp23xx {
	struct spi_device *spi;
	struct gpio_chip gc;
	struct device *dev;
	u8 address;
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

static void mcp23xx_read_reg(struct mcp23xx *mcp23xx, unsigned int reg, u8 *val)
{
	u8 tx_buf[MCP23XX_SPI_MSG_LEN];
	u8 rx_buf[MCP23XX_SPI_MSG_LEN];
	struct spi_transfer xfer;
	int ret;

	tx_buf[0] = mcp23xx->address | MCP23XX_READ_BIT;
	tx_buf[1] = reg;
	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.len = MCP23XX_SPI_MSG_LEN;
	ret = spi_sync_transfer(mcp23xx->spi, &xfer, 1);
	if (ret)
		dev_err(mcp23xx->dev, "SPI read failed, %d\n", ret);
	*val = rx_buf[2];
}

static void mcp23xx_write_reg(struct mcp23xx *mcp23xx, unsigned int reg, u8 val)
{
	u8 tx_buf[MCP23XX_SPI_MSG_LEN];
	int ret;

	tx_buf[0] = mcp23xx->address;
	tx_buf[1] = reg;
	tx_buf[2] = val;
	ret = spi_write(mcp23xx->spi, tx_buf, sizeof(*tx_buf));
	if (ret)
		dev_err(mcp23xx->dev, "SPI write failed, %d\n", ret);
}

static void mcp23xx_update_bits(struct mcp23xx *mcp23xx, unsigned int reg,
			        u16 mask, u8 val)
{
	u8 tmp;

	mcp23xx_read_reg(mcp23xx, reg, &tmp);
	tmp &= ~mask;
	tmp |= val;
	mcp23xx_write_reg(mcp23xx, reg, tmp);
}

static int mcp23xx_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	u8 val;

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_read_reg(mcp23xx, MCP23XX_REG_GPIO, &val);

	return !!(val & offset);
}

static void mcp23xx_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_update_bits(mcp23xx, MCP23XX_REG_GPIO, 1 << offset, val);
}

static void mcp23xx_set_multiple(struct gpio_chip *gc, unsigned long *mask,
				 unsigned long *bits)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_update_bits(mcp23xx, MCP23XX_REG_GPIO, *mask, *bits);
}

static int mcp23xx_direction_output(struct gpio_chip *gc,
				    unsigned offset, int val)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_update_bits(mcp23xx, MCP23XX_REG_IODIR, 1 << offset, 0);
	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_update_bits(mcp23xx, MCP23XX_REG_GPIO, 1 << offset, val);
	return 0;
}

static int mcp23xx_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_update_bits(mcp23xx, MCP23XX_REG_IODIR, 1 << off, 1);
	return 0;
}

static int mcp23xx_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	u8 val;

	pr_info("%s, %d\n", __func__, __LINE__);
	mcp23xx_read_reg(mcp23xx, MCP23XX_REG_IODIR, &val);

	return !!(val & (1 << offset));
}

static int mcp23xx_gpio_set_config(struct gpio_chip *gc, unsigned offset,
				   unsigned long config)
{
	struct mcp23xx *mcp23xx = gpiochip_get_data(gc);
	enum pin_config_param param = pinconf_to_config_param(config);

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		mcp23xx_update_bits(mcp23xx, MCP23XX_REG_GPPU,
				    1 << offset, 0);
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		mcp23xx_update_bits(mcp23xx, MCP23XX_REG_GPPU,
				    1 << offset, 1);
		break;
	default:
		return -ENOTSUPP;

	}

	return 0;
}

static int mcp23xx_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct mcp23xx_chip_info *info =
				&mcp23xx_spi_chip_info[id->driver_data];
	struct mcp23xx_platform_data *pdata;
	struct mcp23xx *mcp23xx;
	int ret;

	pr_err("YEYYEYEYEYEYEYYE\n");
	mcp23xx = devm_kzalloc(&spi->dev, sizeof(*mcp23xx), GFP_KERNEL);
	if (!mcp23xx)
		return -ENOMEM;

	pdata = dev_get_platdata(&spi->dev);
	mcp23xx->address = pdata->address;
	dev_info(&spi->dev, "CHIP ADDRESS 0x%x\n", mcp23xx->address);
	mcp23xx->spi = spi;
	mcp23xx->dev = &spi->dev;
	mcp23xx->gc.parent = &spi->dev;
	mcp23xx->gc.get = mcp23xx_get_value;
	mcp23xx->gc.set = mcp23xx_set_value;
	mcp23xx->gc.direction_output = mcp23xx_direction_output;
	mcp23xx->gc.direction_input = mcp23xx_direction_input;
	mcp23xx->gc.get_direction = mcp23xx_get_direction;
	mcp23xx->gc.set_multiple = mcp23xx_set_multiple;
	mcp23xx->gc.set_config = mcp23xx_gpio_set_config;
	mcp23xx->gc.can_sleep = true;
	mcp23xx->gc.ngpio = info->ngpio;
	mcp23xx->gc.base = -1;
	spi_set_drvdata(spi, mcp23xx);

	ret = gpiochip_add_data(&mcp23xx->gc, mcp23xx);
	if (ret)
		dev_err(&spi->dev, "Failed to register gpiochip\n");


	dev_info(&spi->dev, "%s gpio chip expander registered\n", id->name);
	pr_err("%s gpio chip expander registered\n", id->name);
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
MODULE_LICENSE("GPL");
