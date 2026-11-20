// Support for reading acceleration data from LSM6DSV16X over I2C
#include <string.h> // memcpy
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "i2ccmds.h" // i2cdev_s
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report


// Accelerometer register addresses
#define LSM6DSV16X_FIFO_STATUS1      0x1B
#define LSM6DSV16X_FIFO_DATA_OUT_TAG 0x78

// Accelerometer flag values
#define TAG_ACCEL_NC             0x02
#define LSM6DSV16X_FIFO_WTM_IA   0x80 // FIFO is currently above the watermark threshold
#define LSM6DSV16X_FIFO_OVR_IA   0x40 // FIFO currently overrun
#define LSM6DSV16X_FIFO_FULL_IA  0x20 // FIFO is full now
#define LSM6DSV16X_FIFO_LATCHED  0x08 // FIFO overrun since register was last checked
#define LSM6DSV16X_DIFF_FIFO_8   0x01 // MSB of FIFO length in units of 1 TAG + 6 DATA bytes

#define BYTES_PER_FIFO_WORD   7
#define BYTES_PER_SAMPLE      6
#define SAMPLES_PER_BLOCK     8 // Limited by fitting 6 bytes into the 51 byte block

struct lsm6dsv16x {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    uint16_t unread_words;
    uint8_t fifo_bytes_pending;
    uint32_t time1;
    uint32_t time2;
    struct sensor_bulk sb;
};

enum {
    LSM_PENDING = 1<<0,
};

static struct task_wake lsm6dsv16x_wake;

static uint_fast8_t
lsm6dsv16x_event(struct timer *timer)
{
    struct lsm6dsv16x *ax = container_of(timer, struct lsm6dsv16x, timer);
    ax->flags |= LSM_PENDING;
    sched_wake_task(&lsm6dsv16x_wake);
    return SF_DONE;
}

void
command_config_lsm6dsv16x(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_alloc(args[0], command_config_lsm6dsv16x, sizeof(*ax));
    ax->timer.func = lsm6dsv16x_event;
    ax->i2c = i2cdev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_lsm6dsv16x, "config_lsm6dsv16x oid=%c i2c_oid=%c");

// Schedule collecting data from the accelerometer FIFO
static void
lsm6dsv16x_reschedule_timer(struct lsm6dsv16x *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}

// Query fifo status register
static int
query_fifo_status(struct lsm6dsv16x *ax)
{
    uint8_t reg_status = LSM6DSV16X_FIFO_STATUS1;
    uint8_t status[2];  // Also reading LSM6DSV16X_FIFO_STATUS2 at the same time

    // Timed read-out of the FIFO status registers 1 and 2
    ax->time1 = timer_read_time();
    int ret = i2c_dev_read(ax->i2c, 1, &reg_status, 2, status);
    ax->time2 = timer_read_time();

    // Problem with reading the FIFO status registers
    if (ret != 0) {
        ax->unread_words = 0;
        ax->fifo_bytes_pending = 0;
        return ret;
    }

    // Get the number of words in the FIFO buffer (1 TAG byte + 6 data bytes)
    ax->unread_words = status[0] | ((status[1] & LSM6DSV16X_DIFF_FIFO_8) << 8);

    // Convert to number of bytes pending
    ax->fifo_bytes_pending = ax->unread_words * BYTES_PER_SAMPLE;

    // Check the status bit for FIFO overrun (the immediate, not the latching flag)
    if (status[1] & LSM6DSV16X_FIFO_OVR_IA)
        ax->sb.possible_overflows++;

    return 0;
}

static void
lsm6dsv16x_query(struct lsm6dsv16x *ax, uint8_t oid)
{
    if(query_fifo_status(ax) != 0)
        return;

    if(ax->unread_words == 0)
        return;

    // Read up to SAMPLES_PER_BLOCK words per task pass to keep I2C transactions short
    uint8_t words_to_read = ax->unread_words >= SAMPLES_PER_BLOCK ? SAMPLES_PER_BLOCK : ax->unread_words;
    uint8_t reg_data = LSM6DSV16X_FIFO_DATA_OUT_TAG;
    uint8_t raw_buf[SAMPLES_PER_BLOCK * BYTES_PER_FIFO_WORD];
    uint8_t read_bytes = words_to_read * BYTES_PER_FIFO_WORD;

    if (i2c_dev_read(ax->i2c, 1, &reg_data, read_bytes, raw_buf) != 0)
        return;

    // Strip the 1-byte FIFO tag and pack 6-byte XYZ samples into sensor_bulk
    for (uint8_t i = 0; i < words_to_read; i++) {
        uint8_t *word = &raw_buf[i * BYTES_PER_FIFO_WORD];
        uint8_t sensor_tag = (word[0] >> 3) & 0x1F;
        if (sensor_tag == TAG_ACCEL_NC) {
            memcpy(&ax->sb.data[ax->sb.data_count], &word[1], BYTES_PER_SAMPLE);
            ax->sb.data_count += BYTES_PER_SAMPLE;
            if (ax->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ax->sb.data))
                sensor_bulk_report(&ax->sb, oid);
        }
    }

    // Reschedule immediately if more words remain in FIFO
    if (ax->unread_words >= words_to_read) {
        sched_wake_task(&lsm6dsv16x_wake);
    } else {
        ax->flags &= ~LSM_PENDING;
        lsm6dsv16x_reschedule_timer(ax);
    }
}

void
command_query_lsm6dsv16x(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_lookup(args[0], command_config_lsm6dsv16x);

    sched_del_timer(&ax->timer);
    ax->flags = 0;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ax->rest_ticks = args[1];
    ax->fifo_bytes_pending = 0;
    ax->unread_words = 0;
    sensor_bulk_reset(&ax->sb);

    // Schedule collecting data from the accelerometer FIFO
    lsm6dsv16x_reschedule_timer(ax);
}
DECL_COMMAND(command_query_lsm6dsv16x, "query_lsm6dsv16x oid=%c rest_ticks=%u");

void
command_query_lsm6dsv16x_status(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_lookup(args[0], command_config_lsm6dsv16x);
    output("MCU LSM6DSV16X Driver: command_query_lsm6dsv16x_status() called.");
    if(query_fifo_status(ax)==0) {
        output("MCU LSM6DSV16X Driver: command_query_lsm6dsv16x_status() successfully queried FIFO status.");
        sensor_bulk_status(&ax->sb, args[0], ax->time1, ax->time2 - ax->time1, ax->fifo_bytes_pending);
        output("MCU LSM6DSV16X Driver: command_query_lsm6dsv16x_status() sent sensor_bulk_status report.");
    }
    output("MCU LSM6DSV16X Driver: command_query_lsm6dsv16x_status() encountered problem querying FIFO status.");
}
DECL_COMMAND(command_query_lsm6dsv16x_status, "query_lsm6dsv16x_status oid=%c");

void
lsm6dsv16x_task(void)
{
    if (!sched_check_wake(&lsm6dsv16x_wake))
        return;
    uint8_t oid;
    struct lsm6dsv16x *ax;
    foreach_oid(oid, ax, command_config_lsm6dsv16x) {
        uint_fast8_t flags = ax->flags;
        if (flags & LSM_PENDING)
            lsm6dsv16x_query(ax, oid);
    }
}
DECL_TASK(lsm6dsv16x_task);
