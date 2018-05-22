/*
 *  mcp2210-eval.c - Driver to instantiate mcp2210's eval board SPI devices
 *
 *  Author: Nicolas Saenz Julienne <nicolassaenzj@gmail.com>
 *
 *  Copyright (C) 2017 Prodys S.L.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/spi/mcp23s08.h>
#include <linux/spi/eeprom.h>
#include <linux/sizes.h>

#define DRIVER_NAME			"mcp2210-eval"
#define MCP2210_EVAL_SPI_BUS_NUM	34

struct mcp2210_eval {
	struct platform_device *pdev;
	struct spi_master *spi_master;

	struct spi_device *gpio;
	struct spi_device *temp;
	struct spi_device *adc;
	struct spi_device *eeprom;
};

static struct spi_board_info tc77_temp = {
	.modalias = "tc77",
	.max_speed_hz = 5 * 1000 * 1000,
	.chip_select = 7,
	.mode = SPI_MODE_0,
};

static struct mcp23s08_platform_data mcp23xx_pdata = {
	.spi_present_mask = 0x1,
	.base = -1,
};

static struct spi_board_info mcp23s08_gpio_expander = {
	.modalias = "mcp23s08",
	.max_speed_hz = 1000 * 1000,
	.chip_select = 4,
	.mode = SPI_MODE_0,
	.platform_data = &mcp23xx_pdata,
};

static struct spi_board_info mcp3204_adc = {
	.modalias = "mcp3204",
	.max_speed_hz = 1000 * 1000,
	.chip_select = 1,
	.mode = SPI_MODE_0,
};

static struct spi_eeprom _25lc020_pdata = {
	.byte_len = SZ_2K / 8,
	.name = "25lc020",
	.page_size = 16,
	.flags = EE_ADDR1,
};

static struct spi_board_info _25lc020_eeprom = {
	.modalias = "at25",
	.max_speed_hz = 5 * 1000 * 1000,
	.chip_select = 0,
	.mode = SPI_MODE_0,
	.platform_data = &_25lc020_pdata,
};

static int mcp2210_eval_probe(struct platform_device *pdev)
{
	struct mcp2210_eval *mcp2210_eval;

	mcp2210_eval = devm_kzalloc(&pdev->dev, sizeof(*mcp2210_eval), GFP_KERNEL);
	if (!mcp2210_eval) {
		dev_err(&pdev->dev, "Failed to allocate mcp2210_eval device\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, mcp2210_eval);

	mcp2210_eval->spi_master = spi_busnum_to_master(MCP2210_EVAL_SPI_BUS_NUM);
	if (!mcp2210_eval->spi_master) {
		dev_info(&pdev->dev,
			 "unable to find SPI master, defering probe\n");
		return -EPROBE_DEFER;
	}

	mcp2210_eval->gpio = spi_new_device(mcp2210_eval->spi_master,
					        &mcp23s08_gpio_expander);
	if (!mcp2210_eval->gpio) {
		dev_err(&pdev->dev, "unable to create mcp23s08 SPI device\n");
		return -EINVAL;
	}

	mcp2210_eval->temp = spi_new_device(mcp2210_eval->spi_master,
					    &tc77_temp);
	if (!mcp2210_eval->temp) {
		dev_err(&pdev->dev, "unable to create tc77 SPI device\n");
		return -EINVAL;
	}

	mcp2210_eval->adc = spi_new_device(mcp2210_eval->spi_master,
					       &mcp3204_adc);
	if (!mcp2210_eval->adc) {
		dev_err(&pdev->dev, "unable to create mcp3204 SPI device\n");
		return -EINVAL;
	}

	mcp2210_eval->eeprom = spi_new_device(mcp2210_eval->spi_master,
					       &_25lc020_eeprom);
	if (!mcp2210_eval->eeprom) {
		dev_err(&pdev->dev, "unable to create 25lc020 SPI device\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "mcp2210 evaluation board driver loaded\n");

	return 0;
}

int mcp2210_eval_remove(struct platform_device *pdev)
{
	struct mcp2210_eval *mcp2210_eval = platform_get_drvdata(pdev);

	spi_unregister_device(mcp2210_eval->gpio);
	spi_unregister_device(mcp2210_eval->temp);
	spi_unregister_device(mcp2210_eval->adc);
	spi_unregister_device(mcp2210_eval->eeprom);

	return 0;
}

static struct platform_driver pdriv = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcp2210_eval_probe,
	.remove = mcp2210_eval_remove,
};

static struct platform_device *pdev;

static int mcp2210_eval_init(void)
{
	int ret;

	/*
	 * Here we should check or product specific info (read some eeprom or
	 * something similar) in order to select the proper configuration.
	 */
	ret = platform_driver_register(&pdriv);
	if (ret)
		return ret;

	pdev = platform_device_alloc(DRIVER_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto fail_platform_device1;
	}

	ret = platform_device_add(pdev);
	if (ret)
		goto fail_platform_device2;

	return 0;

fail_platform_device2:
	platform_device_put(pdev);
fail_platform_device1:
	platform_driver_unregister(&pdriv);
	return ret;
}

static void mcp2210_eval_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&pdriv);
}

module_init(mcp2210_eval_init);
module_exit(mcp2210_eval_exit);

MODULE_DESCRIPTION("mcp2210-eval driver");
MODULE_AUTHOR("Nicolas Saenz Julienne <nicolassaenzj@gmail.com>");
MODULE_LICENSE("GPL");
