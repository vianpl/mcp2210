#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/hid.h>
#include <linux/hidraw.h>
#include <linux/module.h>
#include "../linux/drivers/hid/hid-ids.h"

#define USB_DEVICE_ID_MCP2210			0x1111
#define MCP2210_REPORT_LENGTH			0x64

struct mcp2210 {
	struct hid_device *hdev;
	wait_queue_head_t wait;
	u8 read_data[61];
	u8 read_length;
	int xfer_status;
	atomic_t read_avail;
	atomic_t xfer_avail;
	struct gpio_chip gc;
	u8 *in_out_buffer;
	struct mutex lock;

	struct gpio_desc *desc[8];
	bool gpio_poll;
	struct delayed_work gpio_poll_worker;
	unsigned long irq_mask;
	u8 gpio_prev_state;
};

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

static void mcp2210_gpio_irq_ack(struct irq_data *d)
{
}

static void mcp2210_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mcp2210 *mcp2210 = gpiochip_get_data(gc);

	__clear_bit(d->hwirq, &mcp2210->irq_mask);
}

static void mcp2210_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mcp2210 *mcp2210 = gpiochip_get_data(gc);

	__set_bit(d->hwirq, &mcp2210->irq_mask);
}

static unsigned int mcp2210_gpio_irq_startup(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mcp2210 *mcp2210 = gpiochip_get_data(gc);

	return 0;
}

static void mcp2210_gpio_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mcp2210 *mcp2210 = gpiochip_get_data(gc);
}

static int mcp2210_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	return 0;
}

static struct irq_chip mcp2210_gpio_irqchip = {
	.name = "mcp2210-irq",
	.irq_startup = mcp2210_gpio_irq_startup,
	.irq_shutdown = mcp2210_gpio_irq_shutdown,
	.irq_ack = mcp2210_gpio_irq_ack,
	.irq_mask = mcp2210_gpio_irq_mask,
	.irq_unmask = mcp2210_gpio_irq_unmask,
	.irq_set_type = mcp2210_gpio_irq_type,
};

static int mcp2210_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct mcp2210 *mcp2210;
	int ret;

	mcp2210 = devm_kzalloc(&hdev->dev, sizeof(*mcp2210), GFP_KERNEL);
	if (!mcp2210)
		return -ENOMEM;

	mcp2210->in_out_buffer = devm_kzalloc(&hdev->dev, MCP2210_REPORT_LENGTH,
					      GFP_KERNEL);
	if (!mcp2210->in_out_buffer)
		return -ENOMEM;

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

	hid_set_drvdata(hdev, mcp2210);
	init_waitqueue_head(&mcp2210->wait);

	mcp2210->gc.label = "mcp2210-gpio";
	mcp2210->gc.direction_input = mcp2210_gpio_direction_input;
	mcp2210->gc.direction_output = mcp2210_gpio_direction_output;
	mcp2210->gc.set	= mcp2210_gpio_set;
	mcp2210->gc.get	= mcp2210_gpio_get;
	mcp2210->gc.base = -1;
	mcp2210->gc.ngpio = 9;
	mcp2210->gc.can_sleep = 1;
	mcp2210->gc.parent = &hdev->dev;

	ret = gpiochip_add_data(&mcp2210->gc, mcp2210);
	if (ret < 0) {
		hid_err(hdev, "error registering gpio chip\n");
		goto err_hid_stop;
	}

	ret = gpiochip_irqchip_add(&mcp2210->gc, &mcp2210_gpio_irqchip, 0,
				   handle_simple_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(mcp2210->gc.parent, "failed to add IRQ chip\n");
		goto err_gpiochip_remove;
	}

	return ret;

err_gpiochip_remove:
	gpiochip_remove(&mcp2210->gc);
err_hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void mcp2210_remove(struct hid_device *hdev)
{
	struct mcp2210 *mcp2210 = hid_get_drvdata(hdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(mcp2210->desc); i++) {
		gpiochip_unlock_as_irq(&mcp2210->gc, i);
		gpiochip_free_own_desc(mcp2210->desc[i]);
	}

	gpiochip_remove(&mcp2210->gc);
	hid_hw_stop(hdev);
}

static int mcp2210_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct mcp2210 *mcp2210 = hid_get_drvdata(hdev);

	switch (data[0]) {
	default:
		hid_err(hdev, "unknown report\n");

		return 0;
	}

	wake_up_interruptible(&mcp2210->wait);
	return 1;
}

static const struct hid_device_id mcp2210_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MCP2210) },
	{ }
};
MODULE_DEVICE_TABLE(hid, mcp2210_devices);

static struct hid_driver mcp2210_driver = {
	.name = "mcp2210",
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

