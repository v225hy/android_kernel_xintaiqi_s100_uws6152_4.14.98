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

/* ---- DA217 embedded step-counter registers (datasheet, MiraMEMS) ---- */
#define DA217_REG_STEPS_MSB    0x0D    /* R: cumulative steps [15:8] */
#define DA217_REG_STEPS_LSB    0x0E    /* R: cumulative steps [7:0] */
#define DA217_REG_STEP_FILTER  0x33    /* RW: bit0 = Step_en */
#define DA217_STEP_EN          BIT(0)

#define DA217_CHIP_ID          0x13
#define DA217_MODE_ENABLE      0x1E   /* Normal mode, BW=500Hz */
#define DA217_MODE_DISABLE     0x9E   /* Suspend mode */
#define DA217_SOFT_RESET_BITS  ((1 << 2) | (1 << 5))

#define DA217_POLL_INTERVAL    10     /* ms */

/*
 * scsensor (step counter) - software pedometer fed by this driver.
 * The OEM sensors.ums312.so ScSensor opens an input device named
 * "scsensor" and takes EV_ABS + code 0 (ABS_X) value as the float step
 * count.  We detect steps from the accelerometer and report total_steps via
 * input_report_abs(sc_input, ABS_X, total).
 *
 * Algorithm: faithful fixed-point port of the validated TraX / Better Health
 * Tracker Pedometer.java ("ailife/better-health-tracker").  Raw magnitude is
 * fed through gravity-removal (slow EMA high-pass, ~0.5 Hz) then a ~3 Hz
 * low-pass; both use a rate-adaptive coefficient alpha = dt/(RC+dt) so the
 * behaviour is identical at 10 Hz or 100 Hz sampling.  Steps are online
 * local-maxima whose trough-to-peak prominence exceeds a threshold, gated by
 * a refractory period and a cadence/regularity test that rejects irregular
 * arm swinging.  (Two earlier kernel attempts - an EMA-baseline delta
 * crossing and a slowed /1024 low-pass crossing - never fired because on this
 * device normal walking barely moves the *unfiltered* magnitude; removing
 * gravity first and looking at the filtered *oscillation* is the fix.)
 *
 * Units: magnitude and filters are in LSB (raw accel).  The original app uses
 * m/s^2 and a prominence threshold ~0.5 m/s^2 = ~0.05 g.  DA217 static
 * magnitude here is ~5800 LSB = 1 g, so 0.05 g ~ 290 LSB; threshold below is
 * in LSB.
 */
#define STEP_PROM_MIN_LSB   260     /* trough-to-peak prominence (LSB) */
#define STEP_REF_MS         280     /* refractory between peaks (ms)   */
#define STEP_CAD_MIN_MS     300     /* cadence window [min,max] ms      */
#define STEP_CAD_MAX_MS     2000
#define STEP_CAD_TOL        50      /* +/-% of median to be "regular"  */
#define STEP_CAD_WARMUP     8       /* rhythmic peaks before counting   */
#define STEP_GRAV_CUT_HZ    5       /* gravity HP corner (x0.1 Hz=0.5)  */
#define STEP_LP_CUT_HZ      30      /* low-pass corner (x0.1 Hz=3.0)    */

struct da217_step_state {
    bool inited;
    bool rising;        /* lp currently on an up-slope               */
    int grav;           /* slow EMA of magnitude (~gravity)          */
    int lp;             /* band-passed (gravity-removed) low-pass    */
    int prev_lp;
    int valley;         /* running min of lp since last peak         */
    unsigned long last_sample_jiffies;
    unsigned long last_peak_jiffies;
    int streak;         /* consecutive rhythmic peaks                 */
    int recent_iv[6];   /* ring of recent inter-peak intervals (ms)  */
    int recent_n;
    int recent_i;
    u32 count;          /* cumulative steps reported                 */
    /* diagnostics (visible via step_debug sysfs) */
    int last_mag;       /* most recent magnitude            */
    int last_ac;        /* most recent gravity-removed     */
    int last_lp;        /* most recent filtered             */
    int last_prom;      /* last peak prominence             */
    int peaks;          /* total local maxima found         */
    int peaks_cnt;      /* peaks that passed prominence     */
    unsigned long calls;/* times da217_step_process ran     */
};

struct da217_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct input_dev *sc_input;
    struct delayed_work work;
    struct mutex lock;
    u8 enabled;
    bool sc_on;             /* scsensor step reporting enable (HAL sysfs) */
    unsigned int delay_ms;
    s16 sx, sy, sz;
    struct da217_step_state st;
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

/* --- TraX Pedometer fixed-point helpers -------------------------------- */

/*
 * Rate-adaptive one-pole RC coefficient alpha = dt/(RC+dt), RC=1/(2*pi*f).
 * Returned as a Q10 (x1024) fraction.  dt_ms is the real inter-sample time,
 * hz_x10 the corner frequency in units of 0.1 Hz.  RC_ms = 10000/(2*pi*hz)
 * with hz = hz_x10/10  =>  RC_ms = 1591550/(hz_x10*100).  All integer.
 */
static int step_alpha_ms(unsigned long dt_ms, int hz_x10)
{
    unsigned long rc;      /* RC in ms, x100 */
    unsigned long num;

    rc = 159155UL / (unsigned long)hz_x10;    /* RC in ms, x100 */
    num = (unsigned long)dt_ms * 100UL * 1024UL;
    num /= rc + (unsigned long)dt_ms * 100UL;
    if (num > 1024UL)
        num = 1024UL;
    return (int)num;
}

/* update the cadence ring + return true if interval is "regular" */
static bool step_regular(struct da217_step_state *s, int interval_ms)
{
    /* ring of recent inter-peak intervals; keep sorted copy for median */
    int tmp[6];
    int n = s->recent_n, med, i, j;
    bool regular = true;

    if (n > 0) {
        for (i = 0; i < n; i++)
            tmp[i] = s->recent_iv[i];
        /* insertion sort ascending */
        for (i = 1; i < n; i++) {
            int key = tmp[i];
            for (j = i - 1; j >= 0 && tmp[j] > key; j--)
                tmp[j + 1] = tmp[j];
            tmp[j + 1] = key;
        }
        med = (n % 2) ? tmp[n / 2]
                      : (tmp[n / 2 - 1] + tmp[n / 2]) / 2;
        regular = (abs(interval_ms - med) <= (STEP_CAD_TOL * med) / 100);
    }
    s->recent_iv[s->recent_i] = interval_ms;
    s->recent_i = (s->recent_i + 1) % 6;
    if (s->recent_n < 6)
        s->recent_n++;
    return regular;
}

/* a band-pass local maximum of value peak_lp: refractory, prominence,
 * cadence/regularity gate -> possibly count a step. */
static void da217_step_onpeak(struct da217_data *data, int peak_lp,
                              unsigned long now)
{
    struct da217_step_state *s = &data->st;
    int prominence, iv;
    unsigned long since_ms;

    if (peak_lp <= 0)
        return;
    prominence = peak_lp - s->valley;
    s->last_prom = prominence;

    /* refractory - but not for the very first peak (last_peak==0) */
    since_ms = (s->last_peak_jiffies == 0) ? 0
               : jiffies_to_msecs(now - s->last_peak_jiffies);
    if (s->last_peak_jiffies != 0 && since_ms < STEP_REF_MS)
        return;
    if (prominence < STEP_PROM_MIN_LSB)
        return;
    s->peaks_cnt++;

    iv = (s->last_peak_jiffies == 0) ? 0
         : (int)jiffies_to_msecs(now - s->last_peak_jiffies);
    s->last_peak_jiffies = now;

    /* cadence gate: only count peaks that fit the recent rhythm */
    if (iv < STEP_CAD_MIN_MS || iv > STEP_CAD_MAX_MS) {
        s->streak = 0;
        s->recent_n = 0;
        s->recent_i = 0;
        return;
    }
    if (step_regular(s, iv)) {
        s->streak++;
        if (s->streak == STEP_CAD_WARMUP) {
            s->count += STEP_CAD_WARMUP; /* credit warmup steps */
        } else if (s->streak > STEP_CAD_WARMUP) {
            s->count++;
        }
    } else {
        s->streak = 1;
    }
}

/*
 * Software step detection - TraX Pedometer port.  Called under data->lock
 * from the poll work at data->delay_ms cadence (~10-66 ms -> 15-100 Hz).
 */
static void da217_step_process(struct da217_data *data, s16 x, s16 y, s16 z)
{
    struct da217_step_state *s = &data->st;
    int mag, ac;
    unsigned long now = jiffies;
    unsigned long dt_ms;
    int ag, al;             /* rate-adaptive alpha x1024 for grav/lp  */

    mag = int_sqrt((unsigned long)x * x + (unsigned long)y * y +
                   (unsigned long)z * z);
    s->calls++;
    s->last_mag = mag;

    /* scsensor data flow is gated by the HAL enable (writes the sysfs
     * 'scsensor' node).  While disabled we keep the accel polling but do
     * not accumulate or report steps; on re-enable the filter re-seeds. */
    if (!data->sc_on) {
        s->inited = false;
        return;
    }

    /* first sample seeds the gravity estimate */
    if (!s->inited) {
        s->grav = mag;
        s->lp = 0;
        s->prev_lp = 0;
        s->valley = 0;
        s->inited = true;
        s->last_sample_jiffies = now;
        s->last_peak_jiffies = 0;
        return;
    }

    dt_ms = jiffies_to_msecs(now - s->last_sample_jiffies);
    s->last_sample_jiffies = now;
    if (dt_ms == 0)
        dt_ms = 1;
    if (dt_ms > 1000)
        dt_ms = 1000;      /* guard a long stall between re-registrations */

    ag = step_alpha_ms(dt_ms, STEP_GRAV_CUT_HZ);   /* slow, ~0.5 Hz */
    al = step_alpha_ms(dt_ms, STEP_LP_CUT_HZ);     /* ~3 Hz         */

    /* gravity removal (high-pass): ac centres on 0 regardless of tilt */
    s->grav += ((mag - s->grav) * ag) >> 10;
    ac = mag - s->grav;
    s->last_ac = ac;

    /* low-pass the oscillating component into one clean peak per step */
    s->lp += ((ac - s->lp) * al) >> 10;
    s->last_lp = s->lp;

    /* online local maximum: an up-run that turns down peaked at prev_lp */
    if (s->lp > s->prev_lp) {
        s->rising = true;
    } else if (s->lp < s->prev_lp && s->rising) {
        s->peaks++;
        da217_step_onpeak(data, s->prev_lp, now);
        s->rising = false;
        s->valley = s->lp;
    }
    if (s->lp < s->valley)
        s->valley = s->lp;
    s->prev_lp = s->lp;

    /* report cumulative steps on the scsensor ABS_X channel */
    if (data->sc_input) {
        input_report_abs(data->sc_input, ABS_X, (int)s->count);
        input_sync(data->sc_input);
    }
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
        da217_step_process(data, x, y, z);
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

static ssize_t step_counter_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    u32 c;

    mutex_lock(&data->lock);
    c = data->st.count;
    mutex_unlock(&data->lock);

    return scnprintf(buf, PAGE_SIZE, "%u\n", c);
}

/* live diagnostics: TraX-port filter/detector state - for tuning */
static ssize_t step_debug_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    struct da217_step_state *s;
    int grav, ac, lp, prom, mag;
    u32 c;
    unsigned long calls;
    int peaks, peaks_cnt, streak;

    mutex_lock(&data->lock);
    s = &data->st;
    grav = s->grav; ac = s->last_ac; lp = s->last_lp;
    prom = s->last_prom; mag = s->last_mag;
    c = s->count; calls = s->calls;
    peaks = s->peaks; peaks_cnt = s->peaks_cnt; streak = s->streak;
    mutex_unlock(&data->lock);

    return scnprintf(buf, PAGE_SIZE,
                     "count=%u mag=%d grav=%d ac=%d lp=%d "
                     "peak=%d pcnt=%d prom=%d streak=%d calls=%lu\n",
                     c, mag, grav, ac, lp,
                     peaks, peaks_cnt, prom, streak, calls);
}

/* scsensor enable node - the OEM sensors.ums312.so ScSensor HAL writes
 * /sys/class/xr-gsensor/device/scsensor to enable/disable the step
 * counter.  Without this node activate() fails with ENOSYS and no app can
 * read steps.  Write 1 to arm step reporting (re-seeds the detector so the
 * enable instant does not create a spurious step), 0 to pause it. */
static ssize_t scsensor_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct da217_data *data = dev_get_drvdata(dev);
    int on;

    mutex_lock(&data->lock);
    on = data->sc_on ? 1 : 0;
    mutex_unlock(&data->lock);

    return scnprintf(buf, PAGE_SIZE, "%d\n", on);
}

static ssize_t scsensor_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct da217_data *data = dev_get_drvdata(dev);
    unsigned long val;
    int ret;

    ret = kstrtoul(buf, 10, &val);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->sc_on = val ? true : false;
    if (data->sc_on) {
        /* force a fresh filter seed on the next sample so the enable
         * instant does not inject a false peak into the cadence state */
        data->st.inited = false;
    }
    mutex_unlock(&data->lock);

    return count;
}

static DEVICE_ATTR(gsensor,    0644, gsensor_show,          gsensor_enable_store);   /* Bug3(#13): HAL AccSensor::setEnable writes "0"/"1" to gsensor node; was 0444 -> write returns -EIO(-5), activate fails, no xyz for user apps. Reuse enable store: any non-zero starts polling, 0 stops it. */
static DEVICE_ATTR(enable,     0644, gsensor_enable_show,   gsensor_enable_store);
static DEVICE_ATTR(delay_acc,  0644, gsensor_delay_show,    gsensor_delay_store);
static DEVICE_ATTR(step_counter, 0444, step_counter_show,   NULL);
static DEVICE_ATTR(step_debug, 0444, step_debug_show,       NULL);
static DEVICE_ATTR(scsensor,   0644, scsensor_show,         scsensor_store);

static struct attribute *da217_gsensor_attrs[] = {
    &dev_attr_gsensor.attr,
    &dev_attr_enable.attr,
    &dev_attr_delay_acc.attr,
    &dev_attr_step_counter.attr,
    &dev_attr_step_debug.attr,
    &dev_attr_scsensor.attr,
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
    data->sc_on = false;
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

    /*
     * scsensor - software step counter input device for the OEM
     * sensors.ums312.so ScSensor HAL.  It opens an input device by the
     * name "scsensor" and reads EV_ABS + ABS_X value as the float step
     * count.  We feed cumulative steps there from da217_step_process().
     */
    data->sc_input = devm_input_allocate_device(&client->dev);
    if (data->sc_input) {
        data->sc_input->name = "step_counter";
        data->sc_input->id.bustype = BUS_I2C;
        data->sc_input->dev.parent = &client->dev;
        __set_bit(EV_ABS, data->sc_input->evbit);
        input_set_abs_params(data->sc_input, ABS_X, 0, 0x7fffffff, 0, 0);
        ret = input_register_device(data->sc_input);
        if (ret) {
            dev_err(&client->dev, "scsensor input register failed: %d\n", ret);
            data->sc_input = NULL;
        } else {
            dev_info(&client->dev,
                     "scsensor step-counter input registered\n");
        }
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
