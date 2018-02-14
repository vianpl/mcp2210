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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/mtd.h>
#include <linux/delay.h>

#define DRIVER_NAME				"mcp2210-eval"

struct mcp2210_eval {
	struct platform_device *pdev;
	struct spi_device *spi_device;
	struct spi_master *spi_master;
};

static struct spi_board_ifo mcp2210_eval_spi_board = {
	.modalias = "mcp23s08",
	.max_speed_hz = 1000 * 1000,
	.chip_select = 0,
	.mode = SPI_MODE_0,
};

static ssize_t fpga_mtd_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mcp2210_eval *mcp2210_eval = platform_get_drvdata(pdev);
	int ret;

	} else if (*buf == '0') {
		if (!mcp2210_eval->i2c_adapter)
			return -EINVAL;

		ret = fpga_set_mtd_access(mcp2210_eval, 0);
		if (ret) {
			return ret;
			dev_err(&pdev->dev, "Failed to disable spi access\n");
		}

		spi_unregister_device(mcp2210_eval->spi_device);
		i2c_unregister_device(mcp2210_eval->i2c_client);
		mcp2210_eval->i2c_adapter = NULL;
	} else {
		return -EINVAL;
	}

	return len;
}

static int mcp2210_eval_probe(struct platform_device *pdev)
{
	struct mcp2210_eval *mcp2210_eval;
	int ret;

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

	mcp2210_eval->spi_device = spi_new_device(mcp2210_eval->spi_master,
					      &mcp2210_eval_spi_board);
	if (!mcp2210_eval->spi_device) {
		dev_err(&pdev->dev, "unable to create SPI device\n");
		return -EINVAL;
	}

	return 0;
}

static struct platform_driver pdriv = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcp2210_eval_probe,
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

	pr_info("iksunet2-platform driver loaded\n");

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
