// SPDX-License-Identifier: GPL-2.0-only
/*
 * DA217 3-Axis Accelerometer Input Driver
 * SoraNeko fells gravity!
 *
 * Uses delayed workqueue for periodic polling (I2C reads must sleep).
 * Reports EV_ABS events for android.hardware.sensors@1.0.
 *
 * xr-gsensor sysfs shim (added for w527/ums6152 GSI port):
 * The OEM sensors.ums312.so HAL only talks to the Unisoc "xr-gsensor"
 * sysfs class, NOT plain input events. This driver now additionally
 * exports:
 *   /sys/class/xr-gsensor/device/gsensor    -> "X Y Z\n"   (raw 12-bit)
 *   /sys/class/xr-gsensor/device/delay_acc  -> poll period in ms (RW)
 *   /sys/class/xr-gsensor/device/enable     -> 0/1 poll on/off (RW)
 * Same raw values are fed to EV_ABS so both paths stay consistent.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/mutex.h>

#define DA217_REG_SPI_CFG      0x00
#define DA217_REG_CHIP_ID      0x01
#define DA217_REG_ACC_X_LSB    0x02
#define DA217_REG_MODE_BW      0x11

#define DA217_CHIP_ID          0x13
#define DA217_MODE_ENABLE      0x1E   /* Normal mode, BW=500Hz */
#define DA217_MODE_DISABLE     0x9E   /* Suspend mode */
#define DA217_SOFT_RESET_BITS  ((1 << 2) | (1 << 5))

#define DA217_POLL_INTERVAL    10     /* ms */

struct da217_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct delayed_work work;
    struct mutex lock;
    u8 enabled;
    unsigned int delay_ms;
    s16 sx, sy, sz;
};

/* --- xr-gsensor sysfs shim (global, mirrors stock Unisoc layout) --- */
static struct class *da217_gsensor_class;
static struct device *da217_gsensor_dev;

static int da217_soft_reset(struct i2c_client *client)
{
    int ret;
    u8 val;

    ret = i2c_smbus_read_byte_data(client, DA217_REG_SPI_CFG);
    if (ret < 0)
        return ret;
    val = ret | DA217_SOFT_RESET_BITS;
    ret = i2c_smbus_write_byte_data(client, DA217_REG_SPI_CFG, val);
    if (ret < 0)
        return ret;
    msleep(20);
    return 0;
}

static int da217_enable(struct i2c_client *client, bool enable)
{
    u8 data = enable ? DA217_MODE_ENABLE : DA217_MODE_DISABLE;
    return i2c_smbus_write_byte_data(client, DA217_REG_MODE_BW, data);
}

static int da217_read_xyz(struct i2c_client *client, s16 *x, s16 *y, s16 *z)
{
    int ret;

    ret = i2c_smbus_read_word_data(client, DA217_REG_ACC_X_LSB);
    if (ret < 0)
        goto read_error;
    *x = (s16)ret >> 2;

    ret = i2c_smbus_read_word_data(client, DA217_REG_ACC_X_LSB + 2);
    if (ret < 0)
        goto read_error;
    *y = (s16)ret >> 2;

    ret = i2c_smbus_read_word_data(client, DA217_REG_ACC_X_LSB + 4);
    if (ret < 0)
        goto read_error;
    *z = (s16)ret >> 2;

    return 0;

read_error:
    dev_err(&client->dev, "XYZ read error: %d\n", ret);
    return ret;
}

static void da217_work_handler(struct work_struct *work)
{
    struct da217_data *data = container_of(work, struct da217_data, work.work);
    s16 x, y, z;

    if (da217_read_xyz(data->client, &x, &y, &z) == 0) {
        mutex_lock(&data->lock);
        data->sx = x;
        data->sy = y;
        data->sz = z;
        mutex_unlock(&data->lock);

        input_report_abs(data->input, ABS_X, x);
        input_report_abs(data->input, ABS_Y, y);
        input_report_abs(data->input, ABS_Z, z);
        input_sync(data->input);
    }

    mutex_lock(&data->lock);
    if (data->enabled)
        schedule_delayed_work(&data->work,
                              msecs_to_jiffies(data->delay_ms));
    mutex_unlock(&data->lock);
}

static void da217_start_poll(struct da217_data *data)
{
    mutex_lock(&data->lock);
    data->enabled = 1;
    schedule_delayed_work(&data->work,
                          msecs_to_jiffies(data->delay_ms));
    mutex_unlock(&data->lock);
}

static void da217_stop_poll(struct da217_data *data)
{
    mutex_lock(&data->lock);
    data->enabled = 0;
    cancel_delayed_work_sync(&data->work);
    mutex_unlock(&data->lock);
}

/* ---- xr-gsensor sysfs attrs ---- */

static ssize_t gsensor_enable_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", data->enabled ? 1 : 0);
}

static ssize_t gsensor_enable_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct da217_data *data = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val)
        da217_start_poll(data);
    else
        da217_stop_poll(data);

    return count;
}

static ssize_t gsensor_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    s16 x, y, z;

    mutex_lock(&data->lock);
    x = data->sx;
    y = data->sy;
    z = data->sz;
    mutex_unlock(&data->lock);

    return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", x, y, z);
}

static ssize_t gsensor_delay_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n", data->delay_ms);
}

static ssize_t gsensor_delay_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct da217_data *data = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    if (val < 5)
        val = 5;                       /* clamp: >=5ms to stay sane on I2C */
    else if (val > 1000)
        val = 1000;

    mutex_lock(&data->lock);
    data->delay_ms = (unsigned int)val;
    if (data->enabled) {
        /* restart work with new cadence */
        cancel_delayed_work_sync(&data->work);
        schedule_delayed_work(&data->work,
                              msecs_to_jiffies(data->delay_ms));
    }
    mutex_unlock(&data->lock);

    return count;
}

static DEVICE_ATTR(gsensor,    0444, gsensor_show,          NULL);
static DEVICE_ATTR(enable,     0644, gsensor_enable_show,   gsensor_enable_store);
static DEVICE_ATTR(delay_acc,  0644, gsensor_delay_show,    gsensor_delay_store);

static struct attribute *da217_gsensor_attrs[] = {
    &dev_attr_gsensor.attr,
    &dev_attr_enable.attr,
    &dev_attr_delay_acc.attr,
    NULL,
};

static struct attribute_group da217_gsensor_attr_group = {
    .attrs = da217_gsensor_attrs,
};

static int da217_gsensor_class_init(struct da217_data *data)
{
    int ret;

    da217_gsensor_class = class_create(THIS_MODULE, "xr-gsensor");
    if (IS_ERR(da217_gsensor_class)) {
        ret = PTR_ERR(da217_gsensor_class);
        da217_gsensor_class = NULL;
        dev_err(&data->client->dev, "xr-gsensor: class_create failed: %d\n", ret);
        return ret;
    }

    da217_gsensor_dev = device_create(da217_gsensor_class, NULL,
                                      MKDEV(0, 0), NULL, "device");
    if (IS_ERR(da217_gsensor_dev)) {
        ret = PTR_ERR(da217_gsensor_dev);
        da217_gsensor_dev = NULL;
        class_destroy(da217_gsensor_class);
        da217_gsensor_class = NULL;
        dev_err(&data->client->dev,
                "xr-gsensor: device_create failed: %d\n", ret);
        return ret;
    }

    /* Make the gsensor sysfs attrs live on the HAL's expected device node */
    sysfs_remove_group(&da217_gsensor_dev->kobj, &da217_gsensor_attr_group);
    ret = sysfs_create_group(&da217_gsensor_dev->kobj,
                             &da217_gsensor_attr_group);
    if (ret) {
        dev_err(&data->client->dev,
                "xr-gsensor: device file create failed: %d\n", ret);
        device_destroy(da217_gsensor_class, MKDEV(0, 0));
        class_destroy(da217_gsensor_class);
        da217_gsensor_class = NULL;
        da217_gsensor_dev = NULL;
        return ret;
    }

    dev_set_drvdata(da217_gsensor_dev, data);
    return 0;
}

static void da217_gsensor_class_cleanup(void)
{
    if (da217_gsensor_dev) {
        sysfs_remove_group(&da217_gsensor_dev->kobj,
                           &da217_gsensor_attr_group);
        device_destroy(da217_gsensor_class, MKDEV(0, 0));
        da217_gsensor_dev = NULL;
    }
    if (da217_gsensor_class) {
        class_destroy(da217_gsensor_class);
        da217_gsensor_class = NULL;
    }
}

static int da217_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    struct da217_data *data;
    struct input_dev *input;
    int ret;

    ret = i2c_smbus_read_byte_data(client, DA217_REG_CHIP_ID);
    if (ret != DA217_CHIP_ID) {
        dev_err(&client->dev, "Invalid chip ID: 0x%02x\n", ret);
        return (ret < 0) ? ret : -ENODEV;
    }

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    input = devm_input_allocate_device(&client->dev);
    if (!input)
        return -ENOMEM;

    data->client = client;
    data->input = input;
    mutex_init(&data->lock);
    data->enabled = 1;
    data->delay_ms = DA217_POLL_INTERVAL;

    ret = da217_soft_reset(client);
    if (ret)
        return ret;

    ret = da217_enable(client, true);
    if (ret)
        return ret;

    input->name = "accelerometer";
    input->id.bustype = BUS_I2C;
    input->dev.parent = &client->dev;

    __set_bit(EV_ABS, input->evbit);
    input_set_abs_params(input, ABS_X, -8192, 8191, 0, 0);
    input_set_abs_params(input, ABS_Y, -8192, 8191, 0, 0);
    input_set_abs_params(input, ABS_Z, -8192, 8191, 0, 0);

    ret = input_register_device(input);
    if (ret) {
        da217_enable(client, false);
        return ret;
    }

    INIT_DELAYED_WORK(&data->work, da217_work_handler);
    schedule_delayed_work(&data->work, msecs_to_jiffies(data->delay_ms));

    i2c_set_clientdata(client, data);

    ret = da217_gsensor_class_init(data);
    if (ret)
        dev_warn(&client->dev,
                 "xr-gsensor class init failed (%d), accel still works via input\n",
                 ret);

    dev_info(&client->dev, "DA217 accelerometer probed successfully\n");
    return 0;
}

static int da217_remove(struct i2c_client *client)
{
    struct da217_data *data = i2c_get_clientdata(client);

    da217_gsensor_class_cleanup();
    da217_stop_poll(data);
    da217_enable(client, false);
    return 0;
}

static int da217_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct da217_data *data = i2c_get_clientdata(client);

    da217_stop_poll(data);
    da217_enable(client, false);
    return 0;
}

static int da217_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct da217_data *data = i2c_get_clientdata(client);
    int ret;

    ret = da217_enable(client, true);
    if (ret)
        return ret;

    da217_start_poll(data);
    return 0;
}

static SIMPLE_DEV_PM_OPS(da217_pm_ops, da217_suspend, da217_resume);

static const struct of_device_id da217_of_match[] = {
    { .compatible = "da,da217", },
    { }
};
MODULE_DEVICE_TABLE(of, da217_of_match);

static struct i2c_driver da217_driver = {
    .driver = {
        .name           = "da217",
        .of_match_table = da217_of_match,
        .pm             = &da217_pm_ops,
    },
    .probe      = da217_probe,
    .remove     = da217_remove,
    .id_table   = NULL,
};
module_i2c_driver(da217_driver);

MODULE_AUTHOR("ZeroDreamCat <neko@0w0.cafe>");
MODULE_DESCRIPTION("DA217 3-Axis Accelerometer Input Driver (xr-gsensor shim)");
MODULE_LICENSE("GPL v2");
