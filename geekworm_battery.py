#!/usr/bin/env python3
"""
geekworm_battery.py - Geekworm X1206 UPS Battery Telemetry Reader

Interfaces with the onboard MAX17043 fuel gauge IC via I2C Bus 1 at address 0x36.
Supports:
  1. smbus2 / smbus library
  2. Pure Linux ioctl (I2C_RDWR / Repeated Start) on /dev/i2c-1
  3. i2cget command-line tool fallback

Outputs JSON formatted telemetry:
  {"ok": true, "voltage": 4.12, "percentage": 88.5, "chip": "MAX17043", "timestamp": 1729267200000}
"""

import os
import sys
import time
import json
import argparse
import subprocess

I2C_BUS = 1
I2C_ADDR = 0x36
REG_VCELL = 0x02
REG_SOC = 0x04
REG_MODE = 0x06
REG_VERSION = 0xFE

def _read_i2c_smbus(bus_num=I2C_BUS, addr=I2C_ADDR):
    """Try reading registers via smbus / smbus2."""
    try:
        try:
            import smbus2 as smbus
        except ImportError:
            import smbus

        bus = smbus.SMBus(bus_num)
        try:
            # Read 2-byte word data from register 0x02 (VCELL)
            v_raw = bus.read_word_data(addr, REG_VCELL)
            # MAX17043 is big-endian; SMBus word read returns little-endian
            swapped_v = ((v_raw & 0xFF) << 8) | (v_raw >> 8)
            voltage = swapped_v * 78.125 / 1000000.0

            # Read 2-byte word data from register 0x04 (SOC)
            soc_raw = bus.read_word_data(addr, REG_SOC)
            swapped_soc = ((soc_raw & 0xFF) << 8) | (soc_raw >> 8)
            percentage = swapped_soc / 256.0

            return voltage, percentage
        finally:
            bus.close()
    except Exception:
        return None

def _read_i2c_ioctl(bus_num=I2C_BUS, addr=I2C_ADDR):
    """Try reading registers via pure Python fcntl.ioctl on /dev/i2c-N."""
    dev_path = f"/dev/i2c-{bus_num}"
    if not os.path.exists(dev_path):
        return None

    try:
        import fcntl
        import ctypes

        I2C_RDWR = 0x0707
        I2C_M_RD = 0x0001

        class i2c_msg(ctypes.Structure):
            _fields_ = [
                ('addr', ctypes.c_uint16),
                ('flags', ctypes.c_uint16),
                ('len', ctypes.c_uint16),
                ('buf', ctypes.POINTER(ctypes.c_uint8))
            ]

        class i2c_rdwr_ioctl_data(ctypes.Structure):
            _fields_ = [
                ('msgs', ctypes.POINTER(i2c_msg)),
                ('nmsgs', ctypes.c_uint32)
            ]

        def read_word_register(fd, reg):
            write_buf = (ctypes.c_uint8 * 1)(reg)
            read_buf = (ctypes.c_uint8 * 2)()

            msgs = (i2c_msg * 2)(
                i2c_msg(
                    addr=addr,
                    flags=0,
                    len=1,
                    buf=ctypes.cast(write_buf, ctypes.POINTER(ctypes.c_uint8))
                ),
                i2c_msg(
                    addr=addr,
                    flags=I2C_M_RD,
                    len=2,
                    buf=ctypes.cast(read_buf, ctypes.POINTER(ctypes.c_uint8))
                )
            )

            payload = i2c_rdwr_ioctl_data(
                msgs=ctypes.cast(msgs, ctypes.POINTER(i2c_msg)),
                nmsgs=2
            )
            res = fcntl.ioctl(fd, I2C_RDWR, payload)
            if res < 0:
                return None
            return read_buf[0], read_buf[1]

        with open(dev_path, "r+b", buffering=0) as f:
            fd = f.fileno()
            v_bytes = read_word_register(fd, REG_VCELL)
            if v_bytes is None:
                return None
            soc_bytes = read_word_register(fd, REG_SOC)
            if soc_bytes is None:
                return None

            raw_v = (v_bytes[0] << 8) | v_bytes[1]
            voltage = raw_v * 78.125 / 1000000.0

            raw_soc = (soc_bytes[0] << 8) | soc_bytes[1]
            percentage = raw_soc / 256.0

            return voltage, percentage
    except Exception:
        return None

def _read_i2c_i2cget(bus_num=I2C_BUS, addr=I2C_ADDR):
    """Fallback using system i2cget utility."""
    try:
        addr_hex = f"0x{addr:02x}"
        res_v = subprocess.run(
            ['i2cget', '-y', str(bus_num), addr_hex, f"0x{REG_VCELL:02x}", 'w'],
            capture_output=True, text=True, timeout=1.0
        )
        if res_v.returncode != 0:
            return None

        res_soc = subprocess.run(
            ['i2cget', '-y', str(bus_num), addr_hex, f"0x{REG_SOC:02x}", 'w'],
            capture_output=True, text=True, timeout=1.0
        )
        if res_soc.returncode != 0:
            return None

        raw_v = int(res_v.stdout.strip(), 16)
        swapped_v = ((raw_v & 0xFF) << 8) | (raw_v >> 8)
        voltage = swapped_v * 78.125 / 1000000.0

        raw_soc = int(res_soc.stdout.strip(), 16)
        swapped_soc = ((raw_soc & 0xFF) << 8) | (raw_soc >> 8)
        percentage = swapped_soc / 256.0

        return voltage, percentage
    except Exception:
        return None

def read_battery_data(bus_num=I2C_BUS, addr=I2C_ADDR):
    """Reads MAX17043 battery voltage and capacity using best available backend."""
    result = _read_i2c_smbus(bus_num, addr)
    method = 'smbus'
    if result is None:
        result = _read_i2c_ioctl(bus_num, addr)
        method = 'ioctl'
    if result is None:
        result = _read_i2c_i2cget(bus_num, addr)
        method = 'i2cget'

    if result is None:
        return {
            'ok': False,
            'error': 'Failed to communicate with MAX17043 fuel gauge at 0x36',
            'timestamp': int(time.time() * 1000)
        }

    voltage, percentage = result
    percentage_clamped = max(0.0, min(100.0, percentage))
    return {
        'ok': True,
        'voltage': round(voltage, 3),
        'percentage': round(percentage_clamped, 1),
        'chip': 'MAX17043',
        'method': method,
        'timestamp': int(time.time() * 1000)
    }

def main():
    parser = argparse.ArgumentParser(description="Geekworm X1206 Battery Monitor")
    parser.add_argument("--stream", type=float, default=None, help="Stream JSON lines every N seconds")
    parser.add_argument("--bus", type=int, default=I2C_BUS, help="I2C bus number (default 1)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=I2C_ADDR, help="I2C address (default 0x36)")
    args = parser.parse_args()

    if args.stream is not None:
        interval = max(0.5, args.stream)
        consecutive_errors = 0
        while True:
            data = read_battery_data(args.bus, args.addr)
            print(json.dumps(data), flush=True)
            if not data['ok']:
                consecutive_errors += 1
                sleep_time = min(30.0, interval * (2 if consecutive_errors > 3 else 1))
            else:
                consecutive_errors = 0
                sleep_time = interval
            time.sleep(sleep_time)
    else:
        data = read_battery_data(args.bus, args.addr)
        print(json.dumps(data))
        if not data['ok']:
            sys.exit(1)

if __name__ == '__main__':
    main()
