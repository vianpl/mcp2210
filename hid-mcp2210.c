#include <linux/version.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/bitops.h>
#include <linux/hid.h>
#include <linux/spi/spi.h>
#include <linux/gpio/driver.h>
#include <linux/delay.h>
#include "linux-mainline/drivers/hid/hid-ids.h"

#define USB_DEVICE_ID_MCP2210			0x00de
#define MCP2210_REPORT_SIZE			64
#define MCP2210_SPI_MIN_SPEED			1500
#define MCP2210_SPI_MAX_SPEED			12000000
#define MCP2210_NUM_GPIOS			9
#define MCP2210_SPI_MAX_TRANSLEN		U16_MAX
#define MCP2210_SPI_MAX_XFERLEN			60

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
} __packed;

struct mcp2210_spi_transfer_report {
	u8 id;
	u8 len;
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
	struct spi_master *spi_master;
	struct gpio_chip gc;
	struct mutex lock;
	struct hid_report *report;
	s32 *out_buf;
	size_t out_buf_size;
	char in_buf[64];
	char *rx_buf;

	u16 requested_gpio_pins;
	struct completion report_completion;
	u16 tx_offset;
	u8 report_status;
	unsigned int tx_sent;
	unsigned int rx_offset;

	unsigned int last_len;
	bool spi_busy;
	bool write_enabled;
	bool message_bypass;
	unsigned int discount;
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,6,0)
#define gpiochip_get_data(__gc)		container_of(__gc, struct mcp2210, gc)
#endif

static int mcp2210_send_report(struct mcp2210 *mcp2210)
{
	int ret;

	print_hex_dump(KERN_INFO, "raw event data out: ", DUMP_PREFIX_OFFSET, 16, 1,
					 mcp2210->out_buf, MCP2210_REPORT_SIZE, true);

	hid_hw_output_report(mcp2210->hdev, mcp2210->out_buf, MCP2210_REPORT_SIZE);

	ret = wait_for_completion_timeout(&mcp2210->report_completion,
						msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static int mcp2210_set_gpio_as_cs(struct mcp2210 *mcp2210, u8 cs)
{
	struct mcp2210_gpio_settings_report *rep =
		(struct mcp2210_gpio_settings_report*)mcp2210->out_buf;
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
	struct mcp2210 *mcp2210 = spi_master_get_devdata(spi->master);
	u8 cs = spi->chip_select;
	int ret = 0;

	if (!spi->controller_state)
		spi->controller_state = mcp2210;

	mutex_lock(&mcp2210->lock);
	if (!(mcp2210->requested_gpio_pins & BIT(cs))) {
		ret = mcp2210_set_gpio_as_cs(mcp2210, cs);
		if (ret) {
			hid_err(mcp2210->hdev, "failed to setup chip-select\n");
			goto exit;
		}

		mcp2210->requested_gpio_pins |= BIT(cs);
	} else {
		hid_err(mcp2210->hdev, "chip-select already in use\n");
		ret = -EINVAL;
	}

exit:
	mutex_unlock(&mcp2210->lock);
	return ret;
}

static void mcp2210_spi_cleanup(struct spi_device *spi)
{
	struct mcp2210 *mcp2210 = spi_master_get_devdata(spi->master);

	mutex_lock(&mcp2210->lock);
	mcp2210->requested_gpio_pins &= ~BIT(spi->chip_select);
	mutex_unlock(&mcp2210->lock);
	spi->controller_state = NULL;
}

static int mcp2210_bypass_message(struct mcp2210 *mcp2210,
				  struct spi_message *message)
{
	struct spi_transfer *t =
		list_first_entry(&message->transfers, struct spi_transfer, transfer_list);

	/* SPI memory Write Enable bypass: no hace falta hacer un enable de write a
	 * la memoria cada vez que ecribimos un bloque siempre y cuando nadie haya
	 * desactivado el enable (que no lo hace nadie...)
	 */
	if (message->frame_length == 1 && t->tx_buf && ((char*)t->tx_buf)[0] == 0x06) {
		hid_info(mcp2210->hdev, "write enable!\n");
		if (mcp2210->write_enabled) {
			mcp2210->message_bypass = true;
			return 1;
		}

		mcp2210->write_enabled = true;
	/* SPI write check bypass: perdemos unos 8ms por escritura verificando que la
	 * memoria SPI ha copiado todo correctamente, la memoria va a 1MBs mientras
	 * que nosotros siempre iremos <20KBs por lo jodidamente lento que es el
	 * mcp2210 (una escritura SPI de 1 bit tarda 2ms), no hace falta esa
	 * verificación.
	 */
	} else if (message->frame_length == 2 && t->tx_buf && t->len == 1 &&
		   ((char*)t->tx_buf)[0] == 0x05) {
		hid_info(mcp2210->hdev, "write check bypass!!\n");
			mcp2210->message_bypass = true;
			return 1;
	}

	mcp2210->message_bypass = false;

	return 0;
}

static int mcp2210_spi_prepare_message(struct spi_master *master,
				       struct spi_message *message)
{
	struct mcp2210 *mcp2210 = spi_master_get_devdata(master);
	struct mcp2210_spi_settings_report *rep =
		(struct mcp2210_spi_settings_report*)mcp2210->out_buf;
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

	if (mcp2210_bypass_message(mcp2210, message))
		return 0;

	mutex_lock(&mcp2210->lock);
	memset(rep, 0, MCP2210_REPORT_SIZE);
	rep->id = MCP2210_SET_SPI_SETTINGS;
	rep->bit_rate = __cpu_to_le32(bit_rate);
	rep->idle_cs_value |= !(mode & SPI_CS_HIGH) << cs;
	rep->active_cs_value |= (mode & SPI_CS_HIGH) << cs;
	rep->bytes_to_tx = __cpu_to_le16(bytes_to_tx);
	rep->mode = mode & 0x3;
	if (mcp2210->last_len != bytes_to_tx) {
		hid_info(mcp2210->hdev, "spi config\n");
		ret = mcp2210_send_report(mcp2210);
	} else {
		hid_info(mcp2210->hdev, "bypass spi config\n");
	}
	mcp2210->last_len = bytes_to_tx;
	mutex_unlock(&mcp2210->lock);

	return ret;
}

static int mcp2210_spi_transfer_one(struct spi_master *master,
				    struct spi_device *dev,
				    struct spi_transfer *t)
{
	struct mcp2210 *mcp2210 = spi_master_get_devdata(master);
	struct mcp2210_spi_transfer_report *rep =
		(struct mcp2210_spi_transfer_report*)mcp2210->out_buf;
	unsigned int len;

	hid_info(mcp2210->hdev, "transfer... len %d\n", t->len);

	if (mcp2210->message_bypass) {
		hid_info(mcp2210->hdev, "message bypass!!\n");
		return 0;
	}

	mutex_lock(&mcp2210->lock);
	mcp2210->rx_offset = 0;
	mcp2210->tx_sent = 0;
	mcp2210->rx_buf = t->rx_buf;
	len = 0;

	while (mcp2210->rx_offset < t->len)
	{
		/* Si el core SPI pasa de la respuesta del mensaje pasamos nosotros tambien */
		if (!mcp2210->rx_buf && mcp2210->tx_sent == t->len && !mcp2210->spi_busy) {
			mcp2210->discount = t->len - mcp2210->rx_offset;
			break;
		}

		if (((!mcp2210->rx_buf && !mcp2210->spi_busy) || mcp2210->rx_offset == mcp2210->tx_sent) &&
		    mcp2210->tx_sent < t->len) {
			len = t->len - mcp2210->tx_sent;
			if (len > MCP2210_SPI_MAX_XFERLEN)
				len = MCP2210_SPI_MAX_XFERLEN;

			if (t->tx_buf)
				memcpy(rep->data, t->tx_buf + mcp2210->tx_sent, len);

			mcp2210->tx_sent += len;
		} else if (!mcp2210->spi_busy) {
			len = 0;
		}

		mcp2210->spi_busy = false;

		rep->id = MCP2210_SPI_TRANSFER;
		rep->len = len;
		hid_info(mcp2210->hdev, "ID 0x%x, num bits %d", rep->id, rep->len);
		mcp2210_send_report(mcp2210);
	}
	mutex_unlock(&mcp2210->lock);

	hid_info(mcp2210->hdev, "done...\n");

	return 0;
}

static int mcp2210_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);
	return 0;
}

static void mcp2210_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);
}

static int mcp2210_gpio_get_all(struct gpio_chip *chip)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);

	return 0;
}

static int mcp2210_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret;

	ret = mcp2210_gpio_get_all(chip);
	if (ret < 0)
		return ret;

	return (ret >> offset) & 1;
}

static int mcp2210_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);

	return 0;
}

static int mcp2210_probe(struct hid_device *hdev,
			 const struct hid_device_id *id)
{
	struct list_head *report_list =
		&hdev->report_enum[HID_OUTPUT_REPORT].report_list;
	struct mcp2210 *mcp2210;
	int ret;

	hid_info(hdev, "entering probe!!!\n");
	mcp2210 = devm_kzalloc(&hdev->dev, sizeof(*mcp2210), GFP_KERNEL);
	if (!mcp2210)
		return -ENOMEM;

	mcp2210->spi_master = spi_alloc_master(&hdev->dev, 0);
	if (!mcp2210->spi_master)
		return -ENOMEM;

	mutex_init(&mcp2210->lock);
	init_completion(&mcp2210->report_completion);
	hid_set_drvdata(hdev, mcp2210);
	mcp2210->hdev = hdev;

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

	mcp2210->report = list_first_entry(report_list, struct hid_report, list);
	if (mcp2210->report->id != 0) {
		hid_err(hdev, "unexpected report id\n");
		ret = -ENODEV;
		goto err_power_normal;
	}

	mcp2210->out_buf = mcp2210->report->field[0]->value;
	mcp2210->out_buf_size = mcp2210->report->field[0]->report_count *
				mcp2210->report->field[0]->report_size;

	mcp2210->spi_master->num_chipselect = MCP2210_NUM_GPIOS;
	mcp2210->spi_master->bus_num = 34;
	mcp2210->spi_master->mode_bits = SPI_CPOL | SPI_CPHA;
	mcp2210->spi_master->max_speed_hz = MCP2210_SPI_MAX_SPEED;
	mcp2210->spi_master->min_speed_hz = MCP2210_SPI_MIN_SPEED;
	mcp2210->spi_master->setup = mcp2210_spi_setup;
	mcp2210->spi_master->cleanup = mcp2210_spi_cleanup;
	mcp2210->spi_master->prepare_message = mcp2210_spi_prepare_message;
	mcp2210->spi_master->transfer_one = mcp2210_spi_transfer_one;
	spi_master_set_devdata(mcp2210->spi_master, mcp2210);
	ret = devm_spi_register_master(&hdev->dev, mcp2210->spi_master);
	if (ret) {
		hid_err(hdev, "failed to register spi master\n");
		goto err_power_normal;
	}

	mcp2210->gc.label = "mcp2210-gpio";
	mcp2210->gc.direction_input = mcp2210_gpio_direction_input;
	mcp2210->gc.direction_output = mcp2210_gpio_direction_output;
	mcp2210->gc.set	= mcp2210_gpio_set;
	mcp2210->gc.get	= mcp2210_gpio_get;
	mcp2210->gc.base = -1;
	mcp2210->gc.ngpio = MCP2210_NUM_GPIOS;
	mcp2210->gc.can_sleep = 1;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,6,0)
	ret = gpiochip_add(&mcp2210->gc);
#else
	ret = gpiochip_add_data(&mcp2210->gc, mcp2210);
#endif
	if (ret < 0) {
		hid_err(hdev, "error registering gpio chip\n");
		goto err_power_normal;
	}
	hid_info(hdev, "MCP2210 USB/SPI driver registered");

	return 0;

err_power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
err_hid_close:
	hid_hw_close(hdev);
err_hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void mcp2210_remove(struct hid_device *hdev)
{
	struct mcp2210 *mcp2210 = hid_get_drvdata(hdev);

	gpiochip_remove(&mcp2210->gc);
	hid_hw_stop(hdev);
}

static int mcp2210_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct mcp2210 *mcp2210 = hid_get_drvdata(hdev);

	print_hex_dump(KERN_INFO, "raw event data in: ", DUMP_PREFIX_OFFSET, 16, 1,
					 data, size, true);

	switch(data[0]) {
	case MCP2210_SET_SPI_SETTINGS:
	case MCP2210_SET_CHIP_SETTINGS:
		mcp2210->report_status = data[1];
		complete(&mcp2210->report_completion);
		break;
	case MCP2210_GET_SPI_SETTINGS:
	case MCP2210_GET_CHIP_SETTINGS:
		mcp2210->report_status = data[1];
		memcpy(mcp2210->in_buf, data, size);
		complete(&mcp2210->report_completion);
		break;
	case MCP2210_SPI_TRANSFER:
		if (data[1] == MCP2210_STATUS_TX_IN_PROGRESS) {
			/* resend report */
			hid_info(hdev, "spi bus not available\n");
			mcp2210->spi_busy = true;
			complete(&mcp2210->report_completion);
		} else if (data[1] == MCP2210_STATUS_BUS_NOT_AVAILABLE) {
			/* something very wrong happened we cancel the transfer*/
			hid_err(hdev, "spi transfer not properly configured\n");
		} else {
			/* the data was accepted */
			hid_info(hdev, "ID 0x%x, Status 0x%x, num bits %d", data[0], data[3], data[2]);
			if (data[2]) {
				if (mcp2210->discount) {
					mcp2210->discount = 0;
				} else {
					if (mcp2210->rx_buf) {
						memcpy(mcp2210->rx_buf + mcp2210->rx_offset, &data[4], data[2]);
					}
					mcp2210->rx_offset += data[2];
				}
			}

			complete(&mcp2210->report_completion);
		}

		break;
	default:
		hid_err(hdev, "Unexpected id report %d\n", data[0]);
		break;
	}

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
