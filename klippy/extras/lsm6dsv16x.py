# Support for reading acceleration data from a LSM6DSV16X chip
#
# Copyright (C) 2026  Peter Spencer <peter@quantized.art>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
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

LSM6DSV16X_DATA_RATES = {
    0: 0x0, 1.875: 0x1, 7.5: 0x2, 15: 0x3, 30: 0x4, 60: 0x5, 120: 0x6, 240: 0x7,
    480: 0x8, 960: 0x9, 1920: 0xA, 3840: 0xB, 7680: 0xC, 15.625: 0x13,
    31.25: 0x14, 62.5: 0x15, 125: 0x16, 250: 0x17, 500: 0x18, 1000: 0x19,
    2000: 0x1A, 4000: 0x1B, 8000: 0x1C, 12.5: 0x23, 25: 0x24, 50: 0x25,
    100: 0x26, 200: 0x27, 400: 0x28, 800: 0x29, 1600: 0x2A, 3200: 0x2B, 6400: 0x2C
}

LSM6DSV16X_ACCEL_RATES = [0,1.875,7.5,15,30,60,120,240,480,960,1920,3840,7680]

FREEFALL_ACCEL = 9.80665 * 1000.0
SCALE_4G = 0.000122 * FREEFALL_ACCEL

class LSM6DSV16X:
    def __init__(self, config):
        self.last_error_count = 0
        self.batch_processed = False
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        self.axes_map = adxl345.read_axes_map(config, SCALE_4G, SCALE_4G, SCALE_4G)
        self.data_rate = 1920

        # Setup I2C (default 400 kHz, address 0x6A)
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=LSM6DSV16X_ADDRESS, default_speed=400000
        )
        self.mcu = mcu = self.i2c.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_lsm6dsv16x_cmd = None

        mcu.add_config_cmd(
            "config_lsm6dsv16x oid=%d i2c_oid=%d" % (oid, self.i2c.get_oid())
        )
        mcu.add_config_cmd(
            "query_lsm6dsv16x oid=%d rest_ticks=0" % (oid,), on_restart=True
        )
        mcu.register_config_callback(self._build_config)

        # Register FixedFreqReader with little-endian signed 16-bit XYZ ('<hhh')
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, self.data_rate, "<hhh")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._stop_measurements,
            batch_interval=0.100
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
        aqh = adxl345.AccelQueryHelper(self.printer)
        self.batch_bulk.add_client(aqh.handle_batch)
        return aqh

    def _start_measurements(self):
        logging.info("Starting accelerometer measurements.")
        
        # Verify WHO_AM_I (0x0F) == 0x70
        params = self.i2c.i2c_read([LSM6DSV16X_WHO_AM_I], 1)
        if bytearray(params['response'])[0] != LSM6DSV16X_DEVICE_ID:
            raise self.printer.command_error("Invalid LSM6DSV16X WHO_AM_I")

        logging.info("Verified the accelerometer chip ID.")

        # Software reset & poll bit 0
        self.i2c.i2c_write([LSM6DSV16X_CTRL3, 0x01])
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + 0.100
        while bytearray(self.i2c.i2c_read([LSM6DSV16X_CTRL3], 1)['response'])[0] & 0x01:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error("LSM6DSV16X reset timeout")
            reactor.pause(reactor.monotonic() + 0.002)

        logging.info("Accelerometer reset successful.")

        # Configure BDU+IF_INC, +/-4g, FIFO continuous mode, 1.92 kHz ODR
        self.i2c.i2c_write([LSM6DSV16X_CTRL3, 0x44])  # CTRL3
        self.i2c.i2c_write([LSM6DSV16X_CTRL8, 0x01])  # CTRL8 (+/- 4g)
        self.i2c.i2c_write([LSM6DSV16X_FIFO_CTRL4, 0x00])  # FIFO_CTRL4: Bypass mode (flush)
        self.i2c.i2c_write([LSM6DSV16X_FIFO_CTRL3, 0x0A])  # FIFO_CTRL3: BDR_XL = 1.92 kHz
        self.i2c.i2c_write([LSM6DSV16X_CTRL4, 0x06])  # FIFO_CTRL4: Continuous mode
        self.i2c.i2c_write([LSM6DSV16X_CTRL1, 0x0A])  # CTRL1: 1.92 kHz High-Performance

        logging.info("Accelerometer setup completed.")

        # Start MCU timer polling every ~2.5 ms (5 samples at 1.92 kHz)
        rest_ticks = self.mcu.seconds_to_clock(4.0 / self.data_rate)
        self.query_lsm6dsv16x_cmd.send([self.oid, rest_ticks])
        self.ffreader.note_start()
        self.last_error_count = 0

    def _stop_measurements(self):
        self.query_lsm6dsv16x_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        self.i2c.i2c_write([LSM6DSV16X_CTRL1, 0x00])  # Power-down accelerometer

    def _process_batch(self, eventtime):
        if not self.batch_processed:
            logging.info("Started first batch processing...")
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        if not self.batch_processed:
            logging.info("Completed first batch processing, %d samples processed, %d errors, %d overflows."%(len(samples), self.last_error_count, self.ffreader.get_last_overflows()))
            self.batch_processed = True
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
