/*
 *  mcp2210 - Microchip's USB to SPI adapter driver
 *
 *  Copyright (C) 2017 Nicolas Saenz Julienne  <nicolassaenzj@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/bitops.h>
#include <linux/hid.h>
#include <linux/spi/spi.h>
#include <linux/gpio/driver.h>
#include <linux/delay.h>
#include "../linux/drivers/hid/hid-ids.h"

#define USB_DEVICE_ID_MCP2210			0x00de
#define MCP2210_REPORT_SIZE			64
#define MCP2210_SPI_MIN_SPEED			1500
#define MCP2210_SPI_MAX_SPEED			12000000
#define MCP2210_SPI_MAX_TRANSLEN		U16_MAX
#define MCP2210_SPI_MAX_XFERLEN			60
#define MCP2210_SPI_BUS_NUM			34
#define MCP2210_NUM_GPIOS			9

enum {
	MCP2210_SET_CHIP_SETTINGS = 0x21,
	MCP2210_GET_CHIP_SETTINGS = 0x20,
	MCP2210_SET_SPI_SETTINGS = 0x40,
	MCP2210_GET_SPI_SETTINGS = 0x41,
	MCP2210_SPI_TRANSFER = 0x42,
};

enum {
	MCP2210_STATUS_COMMAND_COMPLETED = 0x00,
	MCP2210_STATUS_BUS_NOT_AVAILABLE = 0xF7,
	MCP2210_STATUS_TX_IN_PROGRESS = 0xF8,
};

enum {
	MCP2210_PINCONF_GPIO,
	MCP2210_PINCONF_CS,
	MCP2210_PINCONF_DEDICATED,

};

enum {
	MCP2210_TX_STATUS_FINISHED = 0x10,
	MCP2210_TX_STATUS_STARTED = 0x20,
	MCP2210_TX_STATUS_NOT_FINISHED = 0x30,
};

struct mcp2210_response_report {
	u8 id;
	u8 status;
	u8 data_len;
	u8 spi_tx_status;
	u8 data[60];
} __packed;

struct mcp2210_spi_transfer_report {
	u8 id;
	u8 data_len;
	u16 reserved;
	u8 data[60];
} __packed;

struct mcp2210_gpio_settings_report {
	u8 id;
	u8 reserved[3];
	u8 pinconf[MCP2210_NUM_GPIOS];
	u8 default_gpio_output[2];
	u8 default_gpio_direction[2];
	u8 other_settings;
} __packed;

struct mcp2210_spi_settings_report {
	u8 id;
	u8 reserved[3];
	u32 bit_rate;
	u16 idle_cs_value;
	u16 active_cs_value;
	u16 cs_data_delay;
	u16 cs_deassert_delay;
	u16 data_delay;
	u16 bytes_to_tx;
	u8 mode;
} __packed;

struct mcp2210 {
	struct hid_device *hdev;
	struct spi_controller *spi_controller;
	struct mutex lock;
	struct completion report_completion;

	void *out_buf;
	void *in_buf;
	unsigned int report_size;
	unsigned int total_sent;
};

static int mcp2210_send_report(struct mcp2210 *mcp2210)
{
	int ret;

	print_hex_dump(KERN_INFO, "raw event data out: ", DUMP_PREFIX_OFFSET, 16, 1, mcp2210->out_buf, MCP2210_REPORT_SIZE, true);
	hid_hw_output_report(mcp2210->hdev, mcp2210->out_buf,
			     MCP2210_REPORT_SIZE);
	ret = wait_for_completion_timeout(&mcp2210->report_completion,
					  msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static int mcp2210_set_gpio_as_cs(struct mcp2210 *mcp2210, u8 cs)
{
	struct mcp2210_gpio_settings_report *rep = mcp2210->out_buf;
	int ret;

	rep->id = MCP2210_GET_CHIP_SETTINGS;
	ret = mcp2210_send_report(mcp2210);
	if (ret)
		return ret;

	memcpy(rep, mcp2210->in_buf, MCP2210_REPORT_SIZE);
	rep->id = MCP2210_SET_CHIP_SETTINGS;
	rep->pinconf[cs] = MCP2210_PINCONF_CS;
	ret = mcp2210_send_report(mcp2210);

	return ret;
}

static int mcp2210_spi_setup(struct spi_device *spi)
{
	struct mcp2210 *mcp2210 = spi_controller_get_devdata(spi->controller);
	u8 cs = spi->chip_select;
	int ret = 0;

	mutex_lock(&mcp2210->lock);
	ret = mcp2210_set_gpio_as_cs(mcp2210, cs);
	if (ret)
		hid_err(mcp2210->hdev, "failed to setup chip-select\n");
	mutex_unlock(&mcp2210->lock);

	return ret;
}

static int mcp2210_spi_prepare_message(struct spi_controller *controller,
				       struct spi_message *message)
{
	struct mcp2210 *mcp2210 = spi_controller_get_devdata(controller);
	struct mcp2210_spi_settings_report *rep = mcp2210->out_buf;
	struct spi_device *spi = message->spi;
	u16 bytes_to_tx = message->frame_length;
	u32 bit_rate = spi->max_speed_hz;
	u8 cs = spi->chip_select;
	u16 mode = spi->mode;
	int ret = 0;

	hid_info(mcp2210->hdev, "Message length %d\n", bytes_to_tx);

	if (bytes_to_tx > MCP2210_SPI_MAX_TRANSLEN) {
		dev_err(&spi->dev, "Message is too long (%d)\n",
			message->frame_length);
		return -EINVAL;
	}

	mutex_lock(&mcp2210->lock);
	memset(rep, 0, MCP2210_REPORT_SIZE);
	rep->id = MCP2210_SET_SPI_SETTINGS;
	rep->bit_rate = __cpu_to_le32(bit_rate);
	rep->idle_cs_value |= !(mode & SPI_CS_HIGH) << cs;
	rep->active_cs_value |= (mode & SPI_CS_HIGH) << cs;
	rep->bytes_to_tx = __cpu_to_le16(bytes_to_tx);
	rep->mode = mode & 0x3;
	ret = mcp2210_send_report(mcp2210);
	if (ret)
		hid_err(mcp2210->hdev, "Failed to configure for SPI message\n");
	mutex_unlock(&mcp2210->lock);

	return ret;
}

static int mcp2210_spi_transfer_one(struct spi_controller *controller,
				    struct spi_device *dev,
				    struct spi_transfer *t)
{
	struct mcp2210 *mcp2210 = spi_controller_get_devdata(controller);
	struct mcp2210_spi_transfer_report *out_rep = mcp2210->out_buf;
	struct mcp2210_response_report *in_rep = mcp2210->in_buf;

	hid_info(mcp2210->hdev, "transfer... len %d\n", t->len);

	mcp2210->report_size = 0;
	mcp2210->total_sent = 0;
	memset(in_rep, 0, sizeof(*in_rep));
	out_rep->id = MCP2210_SPI_TRANSFER;
	mutex_lock(&mcp2210->lock);
	while (true)
	{
		hid_info(mcp2210->hdev, "in_rep->len %d, mcp2210->total_sent %d, t->rx_buf %p, t->tx_buf %p\n", in_rep->data_len, mcp2210->total_sent, t->rx_buf, t->tx_buf);
		switch (in_rep->status) {
		case MCP2210_STATUS_COMMAND_COMPLETED:
			/* the data was processed send more if available */
			if (t->rx_buf && in_rep->data_len)
				memcpy(t->rx_buf + mcp2210->total_sent,
				       in_rep->data, in_rep->data_len);

			mcp2210->total_sent += in_rep->data_len;
			if (mcp2210->total_sent >= t->len) {
				hid_err(mcp2210->hdev, "done...\n");
				mutex_unlock(&mcp2210->lock);
				return 0;
			}

			/* calculate next report's size */
			out_rep->data_len = t->len - mcp2210->total_sent;
			if (out_rep->data_len > MCP2210_SPI_MAX_XFERLEN)
				out_rep->data_len = MCP2210_SPI_MAX_XFERLEN;

			/* populate hid report */
			if (t->tx_buf)
				memcpy(out_rep->data,
				       t->tx_buf + mcp2210->total_sent,
				       out_rep->data_len);
			break;
		case MCP2210_STATUS_TX_IN_PROGRESS:
			/* retry last command */
				break;
		case MCP2210_STATUS_BUS_NOT_AVAILABLE:
			/* bus not available, that's an error, we exit */
			hid_err(mcp2210->hdev, "SPI bus not available!\n");
			return -EINVAL;
			break;
		default:
			/* per datasheet should never happen, we exit */
			return -EINVAL;
			break;
		}

		hid_info(mcp2210->hdev, "ID 0x%x, num bits %d", out_rep->id, out_rep->data_len);
		mcp2210_send_report(mcp2210);
	}
}

static int mcp2210_probe(struct hid_device *hdev,
			 const struct hid_device_id *id)
{
	struct mcp2210 *mcp2210;
	int ret;

	mcp2210 = devm_kzalloc(&hdev->dev, sizeof(*mcp2210), GFP_KERNEL);
	if (!mcp2210)
		return -ENOMEM;

	mcp2210->in_buf = devm_kzalloc(&hdev->dev, MCP2210_REPORT_SIZE,
				       GFP_KERNEL);
	mcp2210->out_buf = devm_kzalloc(&hdev->dev, MCP2210_REPORT_SIZE,
					GFP_KERNEL);
	if (!mcp2210->in_buf || !mcp2210->out_buf)
		return -ENOMEM;

	mcp2210->spi_controller = spi_alloc_master(&hdev->dev, 0);
	if (!mcp2210->spi_controller)
		return -ENOMEM;

	mcp2210->hdev = hdev;
	hid_set_drvdata(hdev, mcp2210);
	init_completion(&mcp2210->report_completion);
	mutex_init(&mcp2210->lock);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw open failed\n");
		goto err_hid_stop;
	}

	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		goto err_hid_close;
	}

	mcp2210->spi_controller->num_chipselect = MCP2210_NUM_GPIOS;
	mcp2210->spi_controller->bus_num = MCP2210_SPI_BUS_NUM;
	mcp2210->spi_controller->mode_bits = SPI_CPOL | SPI_CPHA;
	mcp2210->spi_controller->max_speed_hz = MCP2210_SPI_MAX_SPEED;
	mcp2210->spi_controller->min_speed_hz = MCP2210_SPI_MIN_SPEED;
	mcp2210->spi_controller->setup = mcp2210_spi_setup;
	mcp2210->spi_controller->prepare_message = mcp2210_spi_prepare_message;
	mcp2210->spi_controller->transfer_one = mcp2210_spi_transfer_one;
	spi_controller_set_devdata(mcp2210->spi_controller, mcp2210);
	ret = devm_spi_register_controller(&hdev->dev, mcp2210->spi_controller);
	if (ret) {
		hid_err(hdev, "failed to register spi controller\n");
		goto err_hid_close;
	}

	hid_info(hdev, "MCP2210 USB/SPI driver registered");

	return 0;

err_hid_close:
	hid_hw_close(hdev);
err_hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void mcp2210_remove(struct hid_device *hdev)
{
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static int mcp2210_raw_event(struct hid_device *hdev, struct hid_report *report,
			     u8 *data, int size)
{
	struct mcp2210 *mcp2210 = hid_get_drvdata(hdev);

	print_hex_dump(KERN_INFO, "raw event data in: ", DUMP_PREFIX_OFFSET, 16, 1, data, size, true);
	memcpy(mcp2210->in_buf, data, size);
	complete(&mcp2210->report_completion);

	return 0;
}

static const struct hid_device_id mcp2210_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MCP2210) },
	{ }
};
MODULE_DEVICE_TABLE(hid, mcp2210_devices);

static struct hid_driver mcp2210_driver = {
	.name = "hid-mcp2210",
	.id_table = mcp2210_devices,
	.probe = mcp2210_probe,
	.remove	= mcp2210_remove,
	.raw_event = mcp2210_raw_event,
};

module_hid_driver(mcp2210_driver);
MODULE_DESCRIPTION("mcp2210 HID to SPI/GPIO driver");
MODULE_AUTHOR("Nicolas Saenz Julienne <nicolassaenzj@gmail.com>");
MODULE_LICENSE("GPL");
