# Support for reading acceleration data from a LSM6DSV16X chip
#
# Copyright (C) 2026  Peter Spencer <peter@quantized.art>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, collections
from . import bus, bulk_sensor, adxl345

LSM6DSV16X_ADDRESS = 0x6B
LSM6DSV16X_ADDRESS_ALT = 0x6A
LSM6DSV16X_DEVICE_ID = 0x70

# LSM6DSV16X registers
LSM6DSV16X_FUNC_CFG_ACCESS = 0x1
LSM6DSV16X_WHO_AM_I = 0x0F
LSM6DSV16X_FIFO_CTRL1 = 0x07
LSM6DSV16X_FIFO_CTRL2 = 0x08
LSM6DSV16X_FIFO_CTRL3 = 0x09
LSM6DSV16X_FIFO_CTRL4 = 0x0A
LSM6DSV16X_CTRL1 = 0x10
LSM6DSV16X_CTRL2 = 0x11
LSM6DSV16X_CTRL3 = 0x12
LSM6DSV16X_CTRL4 = 0x13
LSM6DSV16X_CTRL5 = 0x14
LSM6DSV16X_CTRL6 = 0x15
LSM6DSV16X_CTRL7 = 0x16
LSM6DSV16X_CTRL8 = 0x17
LSM6DSV16X_CTRL9 = 0x18
LSM6DSV16X_CTRL10 = 0x19
LSM6DSV16X_CTRL_STATUS = 0x1A
LSM6DSV16X_FIFO_STATUS1 = 0x1B
LSM6DSV16X_FIFO_STATUS2 = 0X1C
LSM6DSV16X_HAODR_CFG = 0x62

LSM6DSV16X_DATA_RATES = {
    0: 0x0, 1.875: 0x1, 7.5: 0x2, 15: 0x3, 30: 0x4, 60: 0x5, 120: 0x6, 240: 0x7,
    480: 0x8, 960: 0x9, 1920: 0xA, 3840: 0xB, 7680: 0xC, 15.625: 0x3,
    31.25: 0x4, 62.5: 0x5, 125: 0x6, 250: 0x7, 500: 0x8, 1000: 0x9,
    2000: 0xA, 4000: 0xB, 8000: 0xC, 12.5: 0x3, 25: 0x4, 50: 0x5,
    100: 0x6, 200: 0x7, 400: 0x8, 800: 0x9, 1600: 0xA, 3200: 0xB, 6400: 0xC
}

LSM6DSV16X_HAODR_ENABLE = 0x10
LSM6DSV16X_HAODR_CFGS = {
    15: 0, 30: 0, 60: 0, 120: 0, 240: 0,
    480: 0, 960: 0, 1920: 0, 3840: 0, 7680: 0, 15.625: 1,
    31.25: 1, 62.5: 1, 125: 1, 250: 1, 500: 1, 1000: 1,
    2000: 1, 4000: 1, 8000: 1, 12.5: 2, 25: 2, 50: 2,
    100: 2, 200: 2, 400: 2, 800: 2, 1600: 2, 3200: 2, 6400: 2
}

LSM6DSV16X_MODE = { 'BYPASS': 0, 'FIFO': 1, 'STREAM_WTM_TO_FULL': 2,
                   'STREAM_TO_FIFO': 3, 'BYPASS TO STREAM': 4, 'STREAM': 6,
                   'BYPASS_TO_FIFO': 7 }

FREEFALL_ACCEL = 9.80665 * 1000.0
SCALE_4G = 0.000122 * FREEFALL_ACCEL

BATCH_UPDATES = 0.100

Gyro_Measurement = collections.namedtuple(
    'Gyro_Measurement', ('time', 'omega_p', 'omega_r', 'omega_y'))

class LSM6DSV16XQueryHelper(adxl345.AccelQueryHelper):
    def __init__(self, printer):
        super().__init__(printer)

        # Store readings to handle compressed samples
        self.accel_history = [0, 0]
        self.gyro_history = [0, 0]

    # Would be useful to process data tags here for different sensors
    def get_samples(self):
        return super().get_samples()

class LSM6DSV16X:
    def __init__(self, config):
        self.last_error_count = 0
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        self.axes_map = adxl345.read_axes_map(config, SCALE_4G, SCALE_4G, SCALE_4G)

        # Load settings from printer.cfg
        self.data_rate = config.getint('rate', 1600)
        self.samples_per_read = config.getint('samples_per_read', 16)
        self.samples_over = config.getint('extra_samples_per_read', 4)
        self.poll_interval = config.getfloat('poll_interval', 0.0025)

        # Setup I2C
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=LSM6DSV16X_ADDRESS, default_speed=400000
        )
        self.mcu = mcu = self.i2c.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_lsm6dsv16x_cmd = None

        mcu.add_config_cmd(
            "config_lsm6dsv16x oid=%d i2c_oid=%d samples=%d over=%d" % (
                oid, self.i2c.get_oid(), self.samples_per_read, self.samples_over)
        )
        mcu.add_config_cmd(
            "query_lsm6dsv16x oid=%d rest_ticks=0" % (oid,), on_restart=True
        )
        mcu.register_config_callback(self._build_config)

        # Register FixedFreqReader with little-endian signed 16-bit XYZ ('<hhh')
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<hhh")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._stop_measurements,
            batch_interval=BATCH_UPDATES
        )
        self.name = config.get_name().split()[-1]
        
        hdr = ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')
        self.batch_bulk.add_mux_endpoint("lsm6dsv16x/dump_lsm6dsv16x", "sensor",
                                         self.name, {'header': hdr})
        
        self.printer.add_object("lsm6dsv16x " + self.name, self)

    def _build_config(self):
        cmdqueue = self.i2c.get_command_queue()
        self.query_lsm6dsv16x_cmd = self.mcu.lookup_command(
            "query_lsm6dsv16x oid=%c rest_ticks=%u", cq=cmdqueue
        )
        self.ffreader.setup_query_command(
            "query_lsm6dsv16x_status oid=%c", oid=self.oid, cq=cmdqueue
        )

    def start_internal_client(self):
        aqh = LSM6DSV16XQueryHelper(self.printer)
        self.batch_bulk.add_client(aqh.handle_batch)
        return aqh

    def read_reg(self, reg):
        params = self.i2c.i2c_read([reg], 1)
        return bytearray(params['response'])[0]

    def set_reg(self, reg, val, minclock=0):
        self.i2c.i2c_write([reg, val & 0xFF], minclock=minclock)
        stored_val = self.read_reg(reg)
        if stored_val != val:
            raise self.printer.command_error(
                    "Failed to set LSM6DSV16X register [0x%x] to 0x%x: got 0x%x. "
                    "This is generally indicative of connection problems "
                    "(e.g. faulty wiring) or a faulty lsm6dsv16x chip." % (
                        reg, val, stored_val))

    def is_device_id_correct(self, raise_exception=False):
        reported_device_ID = self.read_reg(LSM6DSV16X_WHO_AM_I)
        if raise_exception and reported_device_ID != LSM6DSV16X_DEVICE_ID:
            raise self.printer.command_error("Invalid LSM6DSV16X device ID: got 0x%x "
                    "instead of 0x%x." % (reported_device_ID, LSM6DSV16X_DEVICE_ID))
        return reported_device_ID == LSM6DSV16X_DEVICE_ID

    def reset(self):
        # Software reset by setting bit 0 of CTRL3 to 1, and then waiting for it to reset to 0
        self.set_reg(LSM6DSV16X_CTRL3, 0x01)
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + 0.100
        while self.read_reg(LSM6DSV16X_CTRL3) & 0x01:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error("LSM6DSV16X reset timeout")
            reactor.pause(reactor.monotonic() + 0.002)

        # Ensure that Block data updatea and register increments on multi-byte reads are enabled
        self.i2c.set_reg(LSM6DSV16X_CTRL3, 0x44)  # They should be set by default

    def set_accelerometer(self):
        if self.data_rate in LSM6DSV16X_HAODR_CFGS:
            # Enable High Accuracy Output Data Rate operation
            self.i2c.set_reg(LSM6DSV16X_HAODR_CFG, LSM6DSV16X_HAODR_CFGS[self.data_rate])  # Set for high accuracy data rate if needed
            self.i2c.set_reg(LSM6DSV16X_CTRL1, LSM6DSV16X_DATA_RATES[self.data_rate] | LSM6DSV16X_HAODR_ENABLE)
        else:
            # Enable High Performance operation
            self.i2c.set_reg(LSM6DSV16X_CTRL1, LSM6DSV16X_DATA_RATES[self.data_rate])  # Set accelerometer output data rate

    def stop_accelerometer(self):
            self.i2c.set_reg(LSM6DSV16X_CTRL1, LSM6DSV16X_DATA_RATES[0])
    
    def stop_gyro(self):
            self.i2c.set_reg(LSM6DSV16X_CTRL2, LSM6DSV16X_DATA_RATES[0])
    
    def select_fifo_data(self, enable_accelerometer=True, enable_gyro=False):
        if enable_accelerometer:
            accelerometer_rate = LSM6DSV16X_DATA_RATES[self.data_rate]
        else:
            accelerometer_rate = LSM6DSV16X_DATA_RATES[0]
        if enable_gyro:
            gyro_rate = LSM6DSV16X_DATA_RATES[self.data_rate]
        else:
            gyro_rate = LSM6DSV16X_DATA_RATES[0]
        self.i2c.set_reg(LSM6DSV16X_FIFO_CTRL3, accelerometer_rate | (gyro_rate << 4))

    def start_fifo(self):
        self.i2c.set_reg(LSM6DSV16X_FIFO_CTRL4, LSM6DSV16X_MODE['STREAM'])

    def stop_fifo(self):
        self.i2c.set_reg(LSM6DSV16X_FIFO_CTRL4, LSM6DSV16X_MODE['BYPASS'])

    def _start_measurements(self):
        # Check the chip is present and reset it.
        self.is_device_id_correct(raise_exception=True)
        self.reset()

        # Set the sensitivity of the accelerometer
        self.i2c.set_reg(LSM6DSV16X_CTRL8, 0x01)  # Set full scale +/- 4g

        # Start the accelerometer
        self.set_accelerometer()
        
        # Setup the chip's FIFO buffer and start sending measurements to it
        self.stop_fifo()
        self.select_fifo_data()
        self.start_fifo()

        # Start MCU timer polling every ~2.5 ms (5 samples at 1.92 kHz)
        rest_ticks = self.mcu.seconds_to_clock(self.poll_interval)
        self.query_lsm6dsv16x_cmd.send([self.oid, rest_ticks])
        self.ffreader.note_start()
        self.last_error_count = 0

    def _stop_measurements(self):
        self.query_lsm6dsv16x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        self.i2c.set_reg(LSM6DSV16X_CTRL1, LSM6DSV16X_DATA_RATES[0])  # Power-down accelerometer
        self.i2c.set_reg(LSM6DSV16X_FIFO_CTRL4, LSM6DSV16X_MODE['BYPASS'])  # Bypass mode empties the FIFO buffer

    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        logging.info("Batch processing, %d samples processed, %d errors, %d overflows."%(len(samples), self.last_error_count, self.ffreader.get_last_overflows()))
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}

    def _convert_samples(self, samples):
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        count = 0
        for ptime, rx, ry, rz in samples:
            raw_xyz = (rx, ry, rz)
            samples[count] = (
                round(ptime, 6),
                round(raw_xyz[x_pos] * x_scale, 6),
                round(raw_xyz[y_pos] * y_scale, 6),
                round(raw_xyz[z_pos] * z_scale, 6),
            )
            count += 1

def load_config(config):
    return LSM6DSV16X(config)

def load_config_prefix(config):
    return LSM6DSV16X(config)
