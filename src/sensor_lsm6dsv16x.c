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

// Accelerometer data TAG values
#define TAG_FIFO_EMPTY           0x00 // Empty - FIFO empty condition
#define TAG_GYRO_NC              0x01 // Gyroscope NC Main Gyroscope uncompressed data
#define TAG_ACCEL_NC             0x02 // Accelerometer NC Main Accelerometer uncompressed data
#define TAG_TEMPERATURE          0x03 // Temperature Auxiliary Temperature data
#define TAG_TIMESTAMP            0x04 // Timestamp Auxiliary Timestamp data
#define TAG_CFG_CHANGE           0x05 // CFG_Change Auxiliary Meta-information data
#define TAG_ACCEL_NC_T2          0x06 // Accelerometer NC_T_2 Main Accelerometer uncompressed batched at two times the previous time slot
#define TAG_ACCEL_NC_T1          0x07 // Accelerometer NC_T_1 Main Accelerometer uncompressed data batched at theprevious time slot
#define TAG_ACCEL_2C             0x08 // Accelerometer 2xC Main Accelerometer 2x compressed data
#define TAG_ACCEL_3C             0x09 // Accelerometer 3xC Main Accelerometer 3x compressed data
#define TAG_GYRO_NC_T2           0x0A // Gyroscope NC_T_2 Main Gyroscope uncompressed data batched at two times the previous time slot
#define TAG_GYRO_NC_T1           0x0B // Gyroscope NC_T_1 Main Gyroscope uncompressed data batched at the previous time slot
#define TAG_GYRO_2C              0x0C // Gyroscope 2xC Main Gyroscope 2x compressed data
#define TAG_GYRO_3C              0x0D // Gyroscope 3xC Main Gyroscope 3x compressed data
#define TAG_SENSOR_HUB_0         0x0E // Sensor Hub Slave 0 Virtual Sensor hub data from slave 0
#define TAG_SENSOR_HUB_1         0x0F // Sensor Hub Slave 1 Virtual Sensor hub data from slave 1
#define TAG_SENSOR_HUB_2         0x10 // Sensor Hub Slave 2 Virtual Sensor hub data from slave 2
#define TAG_SENSOR_HUB_3         0x11 // Sensor Hub Slave 3 Virtual Sensor hub data from slave 3
#define TAG_STEP_COUNT           0x12 // Step Counter Virtual Step counter data
#define TAG_SFLP_GAME_VECTOR     0x13 // Game rotation vector Virtual SFLP-generated game rotation vector
#define TAG_SFLP_GYRO_BIAS       0x16 // Gyroscope bias Virtual SFLP-generated gyroscope bias
#define TAG_SFLP_GRAVITY_VECTOR  0x17 // Gravity vector Virtual SFLP-generated gravity vector
#define TAG_SENSOR_HUB_NACK      0x19 // Sensor Hub Nack Virtual Sensor hub nack from slave 0/1/2/3
#define TAG_MLC_RESULT           0x1A // MLC result Virtual Machine learning core generated result
#define TAG_MLC_FILTER           0x1B // MLC filter Virtual Machine learning core generated filter
#define TAG_MLC_FEATURE          0x1C // MLC feature Virtual Machine learning core generated feature
#define TAG_ACCEL_DUAL_CH2       0x1D // Accelerometer DualC Main Accelerometer channel 2 data
#define TAG_GYRO_EIS             0x1E // Gyroscope EIS Main Gyroscope EIS channel data

// Accelerometer flag values
#define LSM6DSV16X_FIFO_WTM_IA   0x80 // FIFO is currently above the watermark threshold
#define LSM6DSV16X_FIFO_OVR_IA   0x40 // FIFO currently overrun
#define LSM6DSV16X_FIFO_FULL_IA  0x20 // FIFO is full now
#define LSM6DSV16X_FIFO_LATCHED  0x08 // FIFO overrun since register was last checked
#define LSM6DSV16X_DIFF_FIFO_8   0x01 // MSB of FIFO length in units of 1 TAG + 6 DATA bytes

#define BYTES_PER_FIFO_WORD   7
#define BYTES_PER_SAMPLE      6

struct lsm6dsv16x {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    uint16_t unread_words;
    uint8_t fifo_bytes_pending;
    uint32_t time1;
    uint32_t time2;
    uint8_t samples_per_read;
    uint8_t samples_over;
    uint8_t *raw_buff;
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
    uint8_t sensor_tag = TAG_FIFO_EMPTY;

    // Only query number of unread words if this is a new round of read-out passes.
    // Saves bandwidth on I2C bus and reduces time the Klipper MCU is blocked.
    if(ax->unread_words == 0) {
        if(query_fifo_status(ax) != 0)
            return;
    }

    // Read up to SAMPLES_PER_BLOCK words per task pass to keep I2C transactions short
    uint8_t words_to_read = ax->unread_words >= ax->samples_per_read ? ax->samples_per_read : ax->unread_words;
    uint8_t reg_data = LSM6DSV16X_FIFO_DATA_OUT_TAG;
    uint8_t read_bytes = words_to_read * BYTES_PER_FIFO_WORD;

    if (read_bytes > 0) {
        if (i2c_dev_read(ax->i2c, 1, &reg_data, read_bytes, ax->raw_buff) != 0)
            return;
    }

    // Strip the 1-byte FIFO tag and pack 6-byte XYZ samples into sensor_bulk
    for (uint8_t i = 0; i < words_to_read; i++) {
        uint8_t *word = &ax->raw_buff[i * BYTES_PER_FIFO_WORD];
        sensor_tag = (word[0] >> 3) & 0x1F;
        switch (sensor_tag) {
            // Sensor hub
            case TAG_SENSOR_HUB_0:
            case TAG_SENSOR_HUB_1:
            case TAG_SENSOR_HUB_2:
            case TAG_SENSOR_HUB_3:
            case TAG_SENSOR_HUB_NACK:
            // Machine Learning Core
            case TAG_MLC_RESULT:
            case TAG_MLC_FILTER:
            case TAG_MLC_FEATURE:
            // Electronic Image Stabilisation (EIS) motion tracking
            case TAG_GYRO_EIS:
            // Step counter motion tracking
            case TAG_STEP_COUNT:
            // Sensor Fusion Low Power motion tracking
            case TAG_SFLP_GAME_VECTOR:
            case TAG_SFLP_GYRO_BIAS:
            case TAG_SFLP_GRAVITY_VECTOR:
            // Compressed readings (handle here or on host?)
            case TAG_ACCEL_NC_T2:
            case TAG_ACCEL_NC_T1:
            case TAG_ACCEL_2C:
            case TAG_ACCEL_3C:
            case TAG_GYRO_NC_T2:
            case TAG_GYRO_NC_T1:
            case TAG_GYRO_2C:
            case TAG_GYRO_3C:
            // Complete readings to send back
            case TAG_ACCEL_NC:
            case TAG_ACCEL_DUAL_CH2:
            case TAG_GYRO_NC:
                memcpy(&ax->sb.data[ax->sb.data_count], &word[1], BYTES_PER_SAMPLE);
                ax->sb.data_count += BYTES_PER_SAMPLE;
                if (ax->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ax->sb.data))
                    sensor_bulk_report(&ax->sb, oid);
                break;
            case TAG_CFG_CHANGE:
                // This case is very important because all following data is probably invalidated
            case TAG_TIMESTAMP:
                // Precise timing data could be made available here
            case TAG_TEMPERATURE:
                // Accelerometer temperature reading
            case TAG_FIFO_EMPTY:
                // No more samples left, need to keep processing because reading operations
                // take time: the following word could be valid data. Use the final sensor_tag
                // value as a flag to finish this burst of read operations.
        }
    }

    // Note the number of FIFO words read-out on this pass
    ax->unread_words -= words_to_read;

    // Reschedule immediately if more words remain in FIFO
    if (sensor_tag != TAG_FIFO_EMPTY) {
        if (ax->unread_words > 0) {
            sched_wake_task(&lsm6dsv16x_wake);
        }
    } else {
        ax->flags &= ~LSM_PENDING;
        lsm6dsv16x_reschedule_timer(ax);
    }
}

void
command_config_lsm6dsv16x(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_alloc(args[0], command_config_lsm6dsv16x, sizeof(*ax));
    ax->timer.func = lsm6dsv16x_event;
    ax->i2c = i2cdev_oid_lookup(args[1]);

    ax->samples_per_read = args[2];
    ax->samples_over = args[3];
    ax->raw_buff = alloc_chunk(ax->samples_per_read * BYTES_PER_FIFO_WORD);
}
DECL_COMMAND(command_config_lsm6dsv16x, "config_lsm6dsv16x oid=%c i2c_oid=%c samples=%c over=%c");

void
command_query_lsm6dsv16x(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_lookup(args[0], command_config_lsm6dsv16x);

    sched_del_timer(&ax->timer);
    ax->flags = 0;
    if (!args[1]) {
        // End measurements
        return;
    }

    // Start new measurements query
    ax->rest_ticks = args[1];
    ax->fifo_bytes_pending = 0;
    ax->unread_words = 0;
    ax->time1 = timer_read_time();
    ax->time2 = ax->time1;
    sensor_bulk_reset(&ax->sb);

    // Schedule collecting data from the accelerometer FIFO
    lsm6dsv16x_reschedule_timer(ax);
}
DECL_COMMAND(command_query_lsm6dsv16x, "query_lsm6dsv16x oid=%c rest_ticks=%u");

void
command_query_lsm6dsv16x_status(uint32_t *args)
{
    struct lsm6dsv16x *ax = oid_lookup(args[0], command_config_lsm6dsv16x);
    sensor_bulk_status(&ax->sb, args[0], ax->time1, ax->time2 - ax->time1, ax->fifo_bytes_pending);
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
