#include <linux/gpio/driver.h>
#include <linux/hid.h>
#include <linux/hidraw.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include "../linux/drivers/hid/hid-ids.h"

#define USB_DEVICE_ID_MCP2210			0x00de
#define MCP2210_REPORT_SIZE			64
#define MCP2210_SET_SPI_SETTINGS		0x40
#define MCP2210_SPI_MIN_SPEED			1500
#define MCP2210_SPI_MAX_SPEED			12000000
#define MCP2210_NUM_GPIOS			9

struct mcp2210 {
	struct hid_device *hdev;
	wait_queue_head_t wait;
	struct gpio_chip gc;
	struct mutex lock;
	struct hid_report *report;
	s32 *out_buf;
	size_t out_buf_size;
	char in_buf[64];

	struct spi_master *spi_master;
	unsigned int requested_gpio_pins;
	atomic_t transfer_pending;
};

static int mcp2210_send_report(struct mcp2210 *mcp2210, char *buf)
{
	int ret;

	hid_hw_request(mcp2210->hdev, mcp2210->report, HID_REQ_SET_REPORT);
	ret = wait_event_timeout(&mcp2210->wait,
			   atomic_get(&transfer_pending),
			   msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	if (buf)
		memcpy(mcp2210->in_buf, buf, MCP2210_REPORT_SIZE);

	return 0;
}

static int mcp2210_set_gpio_as_cs(struct mcp2210 *mcp2210, u8 cs)
{
}

static int mcp2210_spi_setup(struct spi_device *spi)
{
	struct mcp2210 *mcp2210;
	int ret = 0;
	u8 cs;

	mcp2210 = spi_master_get_devdata(spi->master);
	if (!spi->controller_state)
		spi->controller_state = mcp2210;
	cs = spi->chip_select;

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
	mutex_unlock(&mcp2210->lock);

exit:
	return ret;
}

static void mcp2210_spi_cleanup(struct spi_device *spi)
{
	mutex_lock(&mcp2210->lock);
	mcp2210->requested_gpio_pins &= ~BIT(cs);
	mutex_unlock(&mcp2210->lock);
	spi->controller_state = NULL;
}

static int mcp2210_spi_prepare_message(struct spi_master *master,
				       struct spi_message *message)
{
	struct mcp2210 *mcp2210 = spi_master_get_devdata(master);
	struct spi_device *spi = message->spi;
	u16 idle_cs_val;
	u16 active_cs_val;
	u16 bytes_to_tx;
	u32 bit_rate;
	u8 spi_mode;
	int ret;
	u8 cs;

	spi_mode = spi->mode;
	cs = spi->chip_select;
	bit_rate = spi->max_speed_hz;
	bytes_to_tx = message->frame_length;

	mutex_lock(&mcp2210->lock);
	memset(mcp2210->out_buf, 0, mcp2210->out_buf_size);
	mcp2210->out_buf[0] = MCP2210_SET_SPI_SETTINGS;
	mcp2210->out_buf[4] = (bit_rate >> 24) & 0xff;
	mcp2210->out_buf[5] = (bit_rate >> 16) & 0xff;
	mcp2210->out_buf[6] = (bit_rate >> 8) & 0xff;
	mcp2210->out_buf[7] = bit_rate & 0xff;
	mcp2210->out_buf[8] = 0xff;
	mcp2210->out_buf[9] = 0xff;
	mcp2210->out_buf[10] = ~BIT(cs) & 0xff;
	mcp2210->out_buf[11] = ~BIT(cs) & 0xff;
	mcp2210->out_buf[18] = bytes_to_tx & 0xff;
	mcp2210->out_buf[19] = (bytes_to_tx >> 8) & 0xff;
	mcp2210->out_buf[20] = spi_mode * 0xff;
	mcp2210_send_report(mcp2210);
	mutex_unlock(&mcp2210->lock);
}

static int mcp2210_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);
	struct hid_device *hdev = mcp2210->hdev;
	u8 *buf = mcp2210->in_out_buffer;
	int ret;

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);
	return 0;
}

static void mcp2210_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);
	struct hid_device *hdev = mcp2210->hdev;
	u8 *buf = mcp2210->in_out_buffer;
	int ret;

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);
}

static int mcp2210_gpio_get_all(struct gpio_chip *chip)
{
	struct mcp2210 *mcp2210 = gpiochip_get_data(chip);
	struct hid_device *hdev = mcp2210->hdev;
	u8 *buf = mcp2210->in_out_buffer;
	int ret;

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);

	return ret;
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
	struct hid_device *hdev = mcp2210->hdev;
	u8 *buf = mcp2210->in_out_buffer;
	int ret;

	mutex_lock(&mcp2210->lock);
	mutex_unlock(&mcp2210->lock);

	return 0;
}

static int mcp2210_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct list_head *report_list =
		&hdev->report_enum[HID_OUTPUT_REPORT].report_list;
	struct mcp2210 *mcp2210;
	s32 *buf;
	int ret;

	mcp2210 = devm_kzalloc(&hdev->dev, sizeof(*mcp2210), GFP_KERNEL);
	if (!mcp2210)
		return -ENOMEM;

	mcp2210->spi_master = spi_alloc_master(&hdev->dev, 0);
	if (!mcp2210->spi_master)
		return -ENOMEM;

	mutex_init(&mcp2210->lock);
	init_waitqueue_head(&mcp2210->wait);
	hid_set_drvdata(hdev, mcp2210);
	atomic_set(&mcp2210->transfer_pending, 0);

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
	mcp2210->out_buf = report->field[0]->value;
	mcp2210->out_buf_size = report->field[0]->report_count *
				report_enum->field[0]->report_size;

	mcp2210->spi_master->num_chipselect = MCP2210_NUM_GPIOS;
	mcp2210->spi_master->bus_num = -1;
	mcp2210->spi_master->mode_bits = SPI_CPOL | SPI_CPHA;
	mcp2210->spi_master->max_speed_hz = MCP2210_SPI_MAX_SPEED;
	mcp2210->spi_master->min_speed_hz = MCP2210_SPI_MIN_SPEED;
	mcp2210->spi_master->setup = mcp2210_spi_setup;
	mcp2210->spi_master->cleanup = mcp2210_spi_cleanup;
	mcp2210->spi_master->prepare_message = mcp2210_spi_prepare_message;
	mcp2210->spi_master->transfer_one = mcp2210_spi_transfer_one;
	ret = devm_spi_register_master(&hdev->dev, mcp2210->master);
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
	mcp2210->gc.parent = &hdev->dev;

	ret = gpiochip_add_data(&mcp2210->gc, mcp2210);
	if (ret < 0) {
		hid_err(hdev, "error registering gpio chip\n");
		goto err_power_normal;
	}

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

	print_hex_dump(KERN_INFO, "raw event data: ", DUMP_PREFIX_OFFSET, 16, 1,
		       data, size, true);

	switch(data[0]) {
	case MCP2210_SET_SPI_SETTINGS:
		memcpy(mcp2210->in_buf, data, size);
		wake_up_interruptible(&mcp2210->wait);
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
MODULE_AUTHOR("Daniel Pérez de Andrés <>");
MODULE_LICENSE("GPL");

